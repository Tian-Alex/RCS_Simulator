// =====================================================================
// simulation.js — 模擬引擎（時鐘架構見 docs §8）
//   分離的時鐘：物理積分（RK4，dt）/ 控制器取樣 / 調變器 / 閥門狀態機 /
//   擾動與感測器取樣 / 記錄。RK4 各 stage 內閥門與擾動樣本固定。
// =====================================================================
import { V, Q, M3 } from './mathUtils.js';
import { cloneState, totalMass, rotationalKineticEnergy } from './dynamics.js';
import { rk4Step } from './integrator.js';
import { RcsSystem } from './rcs.js';
import { allocate, axisTorqueCapacity, checkAttainability, buildMatrices } from './allocation.js';
import { Modulator } from './modulation.js';
import { DisturbanceModel } from './disturbance.js';
import { SensorModel } from './sensors.js';
import { PidController } from './controllers/pid.js';
import { LqrController } from './controllers/lqr.js';
import { QuaternionPdController } from './controllers/quaternionPD.js';

export class Simulation {
  constructor(cfg) {
    this.cfg = cfg;
    this.build();
  }

  build() {
    const cfg = this.cfg;
    // 慣量張量與反矩陣（參數更新時重算）
    cfg.vehicle.J = M3.inertiaTensor(
      cfg.vehicle.Ixx, cfg.vehicle.Iyy, cfg.vehicle.Izz,
      cfg.vehicle.Ixy, cfg.vehicle.Ixz, cfg.vehicle.Iyz
    );
    cfg.vehicle.Jinv = M3.inverse(cfg.vehicle.J);
    this.rcs = new RcsSystem(cfg.rcs.thrusters, cfg.vehicle.cg);
    this.rcs.cg = cfg.vehicle.cg.slice();
    this.modulator = new Modulator(cfg.modulator);
    this.disturbance = new DisturbanceModel(cfg.disturbances);
    this.sensor = new SensorModel(cfg.sensors);
    // 控制器
    const c = cfg.controller;
    if (c.type === 'pid') this.controller = new PidController(c.params);
    else if (c.type === 'lqr') {
      this.controller = new LqrController(c.params);
      const res = this.controller.design(cfg.vehicle.Jinv);
      this.lqrStatus = res;
    }
    else this.controller = new QuaternionPdController(c.params);
    // 調變器滿刻度
    this.capacity = axisTorqueCapacity(this.rcs, cfg.allocation.wForce);
    this.attainability = checkAttainability(this.rcs);
    this.modulator.setCapacity(this.capacity);
    // PWM 最小脈衝寬度 = 各推力器最小開啟時間之最小值
    const tmins = this.rcs.thrusters
      .filter((t) => t.enabled && t.maxThrust > 0)
      .map((t) => t.minImpulse / t.maxThrust);
    this.modulator.minPulse = tmins.length ? Math.min(...tmins) : 0;
    this.reset();
  }

  reset() {
    const cfg = this.cfg;
    this.t = 0;
    this.state = {
      r: cfg.initial.position.slice(),
      v: cfg.initial.velocity.slice(),
      q: Q.fromEuler(...cfg.initial.attitude.map((d) => (d * Math.PI) / 180)),
      w: cfg.initial.rates.map((d) => (d * Math.PI) / 180),
      mProp: cfg.vehicle.mProp,
    };
    this.qTarget = Q.fromEuler(...cfg.target.attitude.map((d) => (d * Math.PI) / 180));
    this.rcs.valves.forEach((v) => { v.state = 'CLOSED'; v.window = []; v.cycleCount = 0; v.totalOnTime = 0; v.onHistory = []; v.openedAt = -1; });
    this.modulator.reset();
    this.disturbance.reset();
    this.sensor.reset();
    if (this.controller.reset) this.controller.reset();
    this.tauCmd = [0, 0, 0];
    this.tauActual = [0, 0, 0];
    this.F_I = [0, 0, 0];
    this.dvAccum = 0;                 // ∫‖F_RCS,I‖/m dt
    this.controlEffort = 0;           // ∫‖τ_actual‖² dt
    this.lastControlT = -1;
    this.record = this.makeEmptyRecord();
    this.lastRecordT = -1;
    this.finished = false;
  }

  makeEmptyRecord() {
    return {
      t: [], q: [], euler: [], attErr: [], attErrVec: [], w: [],
      tauCmd: [], tauActual: [], thrusterOn: [], mProp: [], dv: [],
      v: [], r: [], F_I: [],
    };
  }

  // 推進劑耗盡檢查
  propellantOut() { return this.state.mProp <= 1e-9; }

  step(dt) {
    const cfg = this.cfg;
    if (this.finished) return;
    // ---- 1. 擾動與感測器取樣（各自時鐘）----
    this.disturbance.sampleRandom(this.t);
    this.sensor.sample(this.t);
    // ---- 2. 控制器更新（控制週期）----
    if (this.t - this.lastControlT >= cfg.sim.controlPeriod - 1e-12 || this.lastControlT < 0) {
      this.lastControlT = this.t;
      const est = this.sensor.estimate(this.t, this.state.q, this.state.w);
      // 抗飽和信號：以「連續分配」的達成率衡量（非瞬時脈衝力矩），
      // 避免調變模式下積分項被誤凍結（docs §6.1）
      let sat = 1;
      const nCmd = V.norm(this.tauCmd);
      if (nCmd > 1e-9) {
        const fCont = allocate(this.rcs, this.tauCmd, null, cfg.allocation.wForce);
        const { T } = buildMatrices(this.rcs);
        const tauAlloc = [0, 0, 0];
        for (let a = 0; a < 3; a++)
          for (let i = 0; i < fCont.length; i++) tauAlloc[a] += T[a][i] * fCont[i];
        sat = Math.min(1, V.norm(tauAlloc) / nCmd);
      }
      this.tauCmd = this.controller.step(cfg.sim.controlPeriod, est.q, est.w, this.qTarget, sat);
    }
    // ---- 3. 調變 + 分配 ----
    if (cfg.modulator.mode === 'continuous') {
      // 理想連續推力：分配器直接輸出推力比例，閥門保持開啟
      const f = allocate(this.rcs, this.tauCmd, null, cfg.allocation.wForce);
      this.rcs.setContinuousScale(f.map((x, i) => (this.rcs.thrusters[i].maxThrust > 0 ? x / this.rcs.thrusters[i].maxThrust : 0)));
      this.rcs.valves.forEach((v) => { v.state = 'OPEN'; });
    } else {
      const tauMod = this.modulator.step(this.tauCmd, dt);
      const f = allocate(this.rcs, tauMod, null, cfg.allocation.wForce);
      // 推力器開關指令（>1% 滿推力即要求開）
      const cmds = f.map((x, i) => (this.rcs.thrusters[i].enabled && x > 0.01 * this.rcs.thrusters[i].maxThrust));
      this.rcs.valves.forEach((v, i) => v.step(dt, this.t, cmds[i] && !this.propellantOut()));
    }
    // ---- 4. RK4 積分（閥門狀態固定）----
    const rcs = this.rcs;
    const wrenchOf = () => {
      if (this.propellantOut()) return { F: [0, 0, 0], T: [0, 0, 0], mdot: 0 };
      return rcs.wrench();
    };
    const extAccelOf = (t, s) => {
      const tauExt = this.disturbance.torque(t, s.q, this.cfg.vehicle.J);
      return { aExt_I: [0, 0, 0], tauExt_B: tauExt };
    };
    const before = cloneState(this.state);
    this.state = rk4Step(cfg, this.state, this.t, dt, wrenchOf, extAccelOf);
    this.t += dt;
    // ---- 5. 統計量 ----
    const w = wrenchOf();
    this.tauActual = w.T.slice();
    this.F_I = Q.rotateToInertial(this.state.q, w.F);
    const m = totalMass(cfg, this.state);
    this.dvAccum += (V.norm(w.F) / m) * dt;
    this.controlEffort += V.normSq(this.tauActual) * dt;
    // ---- 6. 記錄（recordInterval 間隔取樣，控制圖表資料量）----
    const ri = cfg.sim.recordInterval ?? 0.05;
    if (this.t - this.lastRecordT >= ri - 1e-12) {
      this.lastRecordT = this.t;
      this.recordStep();
    }
    if (this.t >= cfg.sim.duration) this.finished = true;
  }

  recordStep() {
    const R = this.record;
    const s = this.state;
    const qe = Q.error(s.q, this.qTarget);
    const d = Q.log(qe);
    const errAngle = Math.hypot(...d);
    R.t.push(this.t);
    R.q.push(s.q.slice());
    const eu = Q.toEuler(s.q);
    R.euler.push([eu.roll, eu.pitch, eu.yaw]);
    R.attErr.push(errAngle);
    R.attErrVec.push(d.slice());
    R.w.push(s.w.slice());
    R.tauCmd.push(this.tauCmd.slice());
    R.tauActual.push(this.tauActual.slice());
    R.thrusterOn.push(this.rcs.valves.map((v) => (v.state === 'OPEN' ? 1 : 0)));
    R.mProp.push(s.mProp);
    R.dv.push(this.dvAccum);
    R.v.push(s.v.slice());
    R.r.push(s.r.slice());
    R.F_I.push(this.F_I.slice());
  }

  // 效能指標（docs §9）
  metrics() {
    const R = this.record;
    const band = (this.cfg.sim.settleBandDeg * Math.PI) / 180;
    let settleTime = Infinity;
    // 由後往前找最後一次離開帶的時刻
    for (let i = R.attErr.length - 1; i >= 0; i--) {
      if (R.attErr[i] > band) { settleTime = R.t[i]; break; }
    }
    // 最後一筆仍在帶外 → 從未穩定（不得以模擬終點充當穩定時間）
    const n0 = R.attErr.length;
    const finalErr0 = n0 ? R.attErr[n0 - 1] : Infinity;
    if (finalErr0 > band) settleTime = Infinity;
    if (settleTime === Infinity && n0 && R.attErr[0] <= band) settleTime = 0;
    const n = R.attErr.length;
    const lastIdx = Math.max(0, n - 1);
    const finalErr = R.attErr[lastIdx] ?? 0;
    let rmsErr = 0, maxErr = 0;
    for (const e of R.attErr) { rmsErr += e * e; maxErr = Math.max(maxErr, e); }
    rmsErr = n ? Math.sqrt(rmsErr / n) : 0;
    return {
      settleTime,
      finalAttErrDeg: (finalErr * 180) / Math.PI,
      rmsAttErrDeg: (rmsErr * 180) / Math.PI,
      maxAttErrDeg: (maxErr * 180) / Math.PI,
      propellantUsed: this.cfg.vehicle.mProp - Math.max(0, this.state.mProp),
      propellantRemaining: Math.max(0, this.state.mProp),
      dvAccum: this.dvAccum,
      dvVector: V.sub(this.state.v, this.cfg.initial.velocity),
      controlEffort: this.controlEffort,
      cycles: this.rcs.valves.map((v) => v.cycleCount),
      onTimes: this.rcs.valves.map((v) => v.totalOnTime + (v.state === 'OPEN' ? this.t - v.openedAt : 0)),
    };
  }
}
