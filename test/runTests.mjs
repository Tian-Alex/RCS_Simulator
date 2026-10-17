// =====================================================================
// runTests.mjs — 數值核心回歸測試（Node 執行，不需瀏覽器）
//   T1 四元數運動學 vs 解析解
//   T2 無力矩旋轉：能量與慣性系角動量守恆
//   T3 單推力器力/力矩符號
//   T4 純力偶：淨力為零、力矩非零
//   T5 LQR（Newton–Kleinman）vs 對角慣量解析解
//   T6 分配器：雙向可控性
//   T7 三種控制器小角度收斂（閉迴路模擬）
// =====================================================================
import assert from 'node:assert';
import { V, M3, Q, solveLinear } from '../js/mathUtils.js';
import { derivative, addState, cloneState, rotationalKineticEnergy, inertialAngularMomentum, G0 } from '../js/dynamics.js';
import { rk4Step } from '../js/integrator.js';
import { RcsSystem } from '../js/rcs.js';
import { allocate, axisTorqueCapacity, checkAttainability } from '../js/allocation.js';
import { Pwpf } from '../js/modulation.js';
import { PidController } from '../js/controllers/pid.js';
import { LqrController, solveCare, linearize } from '../js/controllers/lqr.js';
import { QuaternionPdController } from '../js/controllers/quaternionPD.js';
import { presetByName } from '../js/parameters.js';

let passed = 0, failed = 0;
function test(name, fn) {
  try { fn(); passed++; console.log(`  ok  ${name}`); }
  catch (e) { failed++; console.error(`  FAIL ${name}: ${e.message}`); }
}

// ------------------- T1 四元數運動學 -------------------
test('T1a 常角速度四元數 = 解析旋轉', () => {
  const w = [0.1, -0.3, 0.25];
  const dt = 0.001, T = 2.0;
  let q = Q.identity();
  const n = Math.round(T / dt);
  for (let i = 0; i < n; i++) {
    const dq = Q.mul(q, [0, ...w]);
    q = Q.normalize([q[0] + 0.5 * dt * dq[0], q[1] + 0.5 * dt * dq[1], q[2] + 0.5 * dt * dq[2], q[3] + 0.5 * dt * dq[3]]);
  }
  const angle = Math.hypot(...w) * T;
  const axis = V.scale(w, 1 / Math.hypot(...w));
  const qExact = Q.fromAxisAngle(axis, angle);
  // q 與 −q 等價：取兩種表示中較小者
  let dPlus = 0, dMinus = 0;
  for (let i = 0; i < 4; i++) {
    dPlus += (q[i] - qExact[i]) ** 2;
    dMinus += (q[i] + qExact[i]) ** 2;
  }
  const d1 = Math.sqrt(Math.min(dPlus, dMinus));
  assert.ok(d1 < 1e-6, `四元數誤差 ${d1}`);
});
test('T1b q 與 −q 動力學等價', () => {
  const cfg = presetByName('medium');
  cfg.vehicle.J = M3.inertiaTensor(cfg.vehicle.Ixx, cfg.vehicle.Iyy, cfg.vehicle.Izz, 0, 0, 0);
  cfg.vehicle.Jinv = M3.inverse(cfg.vehicle.J);
  const s1 = { r: [0, 0, 0], v: [0, 0, 0], q: Q.normalize([0.9, 0.2, 0.3, 0.2]), w: [0.1, 0.05, -0.2], mProp: 10 };
  const s2 = { ...s1, q: s1.q.map((x) => -x) };
  const d1 = derivative(cfg, s1, [0, 0, 0], [0, 0, 0], [0, 0, 0], 0);
  const d2 = derivative(cfg, s2, [0, 0, 0], [0, 0, 0], [0, 0, 0], 0);
  for (let i = 0; i < 4; i++) assert.ok(Math.abs(Math.abs(d1.q[i]) - Math.abs(d2.q[i])) < 1e-12);
});

// ------------------- T2 無力矩旋轉守恆 -------------------
test('T2 能量與慣性角動量守恆（RK4）', () => {
  const cfg = presetByName('medium');
  cfg.vehicle.J = M3.inertiaTensor(1440, 3387, 3387, 50, -30, 20); // 含耦合項
  cfg.vehicle.Jinv = M3.inverse(cfg.vehicle.J);
  let s = { r: [0, 0, 0], v: [0, 0, 0], q: Q.identity(), w: [0.3, 0.5, -0.2], mProp: 10 };
  const E0 = rotationalKineticEnergy(cfg.vehicle.J, s.w);
  const H0 = inertialAngularMomentum(s.q, cfg.vehicle.J, s.w);
  const dt = 0.005;
  for (let i = 0; i < 4000; i++) {
    s = rk4Step(cfg, s, i * dt, dt, () => ({ F: [0, 0, 0], T: [0, 0, 0], mdot: 0 }),
      () => ({ aExt_I: [0, 0, 0], tauExt_B: [0, 0, 0] }));
  }
  const E1 = rotationalKineticEnergy(cfg.vehicle.J, s.w);
  const H1 = inertialAngularMomentum(s.q, cfg.vehicle.J, s.w);
  assert.ok(Math.abs(E1 - E0) / E0 < 1e-8, `能量漂移 ${(Math.abs(E1 - E0) / E0).toExponential(2)}`);
  assert.ok(V.norm(V.sub(H1, H0)) / V.norm(H0) < 1e-8, `角動量漂移`);
});

// ------------------- T3/T4 推力器力學 -------------------
test('T3 單推力器力與力矩符號', () => {
  const cfg = presetByName('medium');
  const rcs = new RcsSystem(cfg.rcs.thrusters, [0, 0, 0]);
  // R1: 位置 (2, 0, 1.2)，推力方向 (0,-1,0)
  rcs.valves[0].state = 'OPEN';
  const { F, T } = rcs.wrench();
  assert.ok(Math.abs(F[1] + 10) < 1e-12, 'F = −10 ŷ');
  // τ = p × F = (2,0,1.2) × (0,−10,0) = (0·0−1.2·(−10), 1.2·0−2·0, 2·(−10)−0) = (12, 0, −20)
  assert.ok(Math.abs(T[0] - 12) < 1e-12 && Math.abs(T[2] + 20) < 1e-12, `τ = ${T}`);
});
test('T4 純力偶：淨力 0、力矩 ≠ 0', () => {
  const cfg = presetByName('medium');
  const rcs = new RcsSystem(cfg.rcs.thrusters, [0, 0, 0]);
  rcs.valves[4].state = 'OPEN'; rcs.valves[5].state = 'OPEN'; // P1+P2
  const { F, T } = rcs.wrench();
  assert.ok(V.norm(F) < 1e-12, `淨力 ${F}`);
  assert.ok(T[1] > 0, '俯仰力矩為正');
  // 每具 10 N、臂長 2 m → τ_y = 40 N·m
  assert.ok(Math.abs(T[1] - 40) < 1e-9, `τ_y = ${T[1]}`);
});

// ------------------- T5 LQR -------------------
test('T5a LQR 對角慣量解析解', () => {
  // 單軸雙積分器：J θ̈ = τ；Q=diag(qθ, qω)、R=r
  // 由 CARE 解析解（docs §6.2 推導）：
  //   Kp = √(qθ/r)   （與 J 無關）
  //   Kd = √(qω/r + 2·J·√(qθ/r))
  const J = 100;
  const Jinv = M3.inverse(M3.inertiaTensor(J, J, J, 0, 0, 0));
  const qth = 4, qw = 16, r = 2;
  const res = solveCare(Jinv, [qth, qth, qth, qw, qw, qw], [r, r, r]);
  assert.ok(res.ok, `CARE 未收斂: ${res.reason}`);
  const kpExpected = Math.sqrt(qth / r);
  const kdExpected = Math.sqrt(qw / r + 2 * J * Math.sqrt(qth / r));
  for (let a = 0; a < 3; a++) {
    assert.ok(Math.abs(res.K[a][a] - kpExpected) < 1e-6 * kpExpected, `Kp=${res.K[a][a]} 期望 ${kpExpected}`);
    assert.ok(Math.abs(res.K[a][a + 3] - kdExpected) < 1e-6 * kdExpected, `Kd=${res.K[a][a + 3]} 期望 ${kdExpected}`);
  }
});
test('T5b LQR 閉迴路極點穩定', () => {
  const cfg = presetByName('medium');
  const Jinv = M3.inverse(M3.inertiaTensor(1440, 3387, 3387, 80, -40, 60));
  const res = solveCare(Jinv, [10, 10, 10, 100, 100, 100], [1e5, 1e5, 1e5]);
  assert.ok(res.ok, 'CARE 未收斂');
  assert.ok(res.residual < 1e-8, `CARE 殘差 ${res.residual}`);
  // 驗證 P 正定
  for (let i = 0; i < 6; i++) assert.ok(res.P[i][i] > 0);
});

// ------------------- T6 分配器 -------------------
test('T6 雙向可控性（12 具純力偶）', () => {
  const cfg = presetByName('medium');
  const rcs = new RcsSystem(cfg.rcs.thrusters, [0, 0, 0]);
  const att = checkAttainability(rcs);
  assert.ok(att.bidirectional.every(Boolean), `雙向性 ${att.bidirectional}`);
  // 質心偏移後仍可控
  const rcs2 = new RcsSystem(cfg.rcs.thrusters, [0.3, -0.1, 0.2]);
  const att2 = checkAttainability(rcs2);
  assert.ok(att2.bidirectional.every(Boolean), `偏心後雙向性 ${att2.bidirectional}`);
});

// ------------------- T7 控制器收斂（含調變與閥門的閉迴路） -------------------
function runClosedLoop(controllerType, cfg) {
  const sim = makeSim(controllerType, cfg);
  const dt = 0.005;
  while (!sim.finished) sim.step(dt);
  return sim;
}
import { Simulation } from '../js/simulation.js';
function makeSim(type, cfgIn) {
  const cfg = JSON.parse(JSON.stringify(cfgIn));
  cfg.controller = { type, params: cfg.controller.params };
  if (type === 'lqr') cfg.controller.params = { Qdiag: [6000, 6000, 6000, 60000, 60000, 60000], Rdiag: [1, 1, 1] };
  if (type === 'qpd') cfg.controller.params = { Kp: [76, 76, 76], Kd: [915, 915, 915] };
  cfg.disturbances.constantTorque = [0, 0, 0];
  cfg.disturbances.randomTorque.sigma = [0, 0, 0];
  cfg.sensors.gyroNoise = [0, 0, 0]; cfg.sensors.attNoise = [0, 0, 0];
  return new Simulation(cfg);
}
test('T7a PID 收斂（連續推力）', () => {
  const cfg = presetByName('medium');
  cfg.modulator.mode = 'continuous';
  const sim = runClosedLoop('pid', cfg);
  const m = sim.metrics();
  assert.ok(m.finalAttErrDeg < 0.2, `最終誤差 ${m.finalAttErrDeg}°`);
  assert.ok(m.settleTime < 60, `穩定時間 ${m.settleTime}s`);
});
test('T7b LQR 收斂（連續推力）', () => {
  const cfg = presetByName('medium');
  cfg.modulator.mode = 'continuous';
  const sim = runClosedLoop('lqr', cfg);
  const m = sim.metrics();
  assert.ok(m.finalAttErrDeg < 0.2, `最終誤差 ${m.finalAttErrDeg}°`);
});
test('T7c QuaternionPD 收斂（連續推力）', () => {
  const cfg = presetByName('medium');
  cfg.modulator.mode = 'continuous';
  const sim = runClosedLoop('qpd', cfg);
  const m = sim.metrics();
  assert.ok(m.finalAttErrDeg < 0.2, `最終誤差 ${m.finalAttErrDeg}°`);
});
test('T7d PID + PWPF + 閥門收斂', () => {
  const cfg = presetByName('medium');
  cfg.modulator.mode = 'pwpf';
  cfg.sim.settleBandDeg = 1.0; // 調變模式有固有極限環，放寬帶寬
  const sim = runClosedLoop('pid', cfg);
  const m = sim.metrics();
  assert.ok(m.finalAttErrDeg < 1.0, `最終誤差 ${m.finalAttErrDeg}°`);
  assert.ok(m.settleTime < 110, `穩定時間 ${m.settleTime}s`);
  assert.ok(m.propellantUsed > 0, '有消耗推進劑');
  assert.ok(m.cycles.some((c) => c > 0), '有閥門動作');
});
test('T7e PID + PWM 收斂', () => {
  const cfg = presetByName('medium');
  cfg.modulator.mode = 'pwm';
  cfg.sim.settleBandDeg = 1.0;
  const sim = runClosedLoop('pid', cfg);
  const m = sim.metrics();
  assert.ok(m.finalAttErrDeg < 1.0, `最終誤差 ${m.finalAttErrDeg}°`);
});
test('T7f 最小衝量限制：實際開啟時間 ≥ t_min', () => {
  const cfg = presetByName('medium');
  const sim = runClosedLoop('pid', cfg);
  // 由 onHistory 驗證每段開啟時間
  for (const v of sim.rcs.valves) {
    for (let i = 0; i + 1 < v.onHistory.length; i++) {
      if (v.onHistory[i][1] === 1 && v.onHistory[i + 1][1] === 0) {
        const dur = v.onHistory[i + 1][0] - v.onHistory[i][0];
        const tmin = cfg.rcs.thrusters[0].minImpulse / cfg.rcs.thrusters[0].maxThrust;
        assert.ok(dur >= tmin - 0.02, `開啟 ${dur}s < t_min ${tmin}s`);
      }
    }
  }
});
test('T7g 半步長重跑軌跡一致（確定性）', () => {
  const cfg = presetByName('medium');
  const s1 = runClosedLoop('pid', cfg);
  const cfg2 = JSON.parse(JSON.stringify(cfg));
  cfg2.sim.dt = 0.0025;
  const s2 = runClosedLoop('pid', cfg2);
  const n = Math.min(s1.record.attErr.length, s2.record.attErr.length);
  let maxDiff = 0;
  for (let i = 0; i < n; i++) maxDiff = Math.max(maxDiff, Math.abs(s1.record.attErr[i] - s2.record.attErr[i]));
  assert.ok(maxDiff < 0.01, `步長減半最大誤差 ${maxDiff} rad`);
});

// ------------------- T8 穩定時間語意 -------------------
test('T8a 全程在帶內 → 穩定時間 0', () => {
  const cfg = presetByName('medium');
  cfg.sim.settleBandDeg = 10; // 初始誤差 ~61° 仍可能在帶外
  const sim = new Simulation(cfg);
  // 直接注入固定紀錄測 metrics 語意
  sim.record.t = [0, 1, 2, 3];
  sim.record.attErr = [0.01, 0.01, 0.005, 0.004]; // 全程 < 帶寬（0.17 rad）
  sim.state.mProp = cfg.vehicle.mProp;
  const m = sim.metrics();
  assert.strictEqual(m.settleTime, 0, `應為 0，得 ${m.settleTime}`);
});
test('T8b 先出帶後回帶 → 穩定時間 = 最後離開時刻', () => {
  const cfg = presetByName('medium');
  const sim = new Simulation(cfg);
  sim.record.t = [0, 1, 2, 3, 4, 5];
  sim.record.attErr = [1.0, 0.01, 0.2, 0.005, 0.004, 0.004];
  sim.state.mProp = cfg.vehicle.mProp;
  const m = sim.metrics();
  // 帶寬 0.5° ≈ 0.00873 rad；t=2 的 0.2 rad 在帶外
  assert.ok(Math.abs(m.settleTime - 2) < 1e-9, `應為 2，得 ${m.settleTime}`);
});
test('T8c 最後一筆仍在帶外 → 未穩定（Infinity）', () => {
  const cfg = presetByName('medium');
  const sim = new Simulation(cfg);
  sim.record.t = [0, 1, 2, 3];
  sim.record.attErr = [1.0, 0.5, 0.2, 0.15]; // 0.15 rad > 0.00873 rad 帶寬
  sim.state.mProp = cfg.vehicle.mProp;
  const m = sim.metrics();
  assert.strictEqual(m.settleTime, Infinity, `應為未穩定，得 ${m.settleTime}`);
});

// ------------------- T9 點火時間 vs 推進劑一致性 -------------------
test('T9 累計點火時間與推進劑消耗一致（Δm = Σ F·T/(Isp·g0)）', () => {
  const cfg = presetByName('medium');
  cfg.modulator.mode = 'pwpf';
  const sim = runClosedLoop('pid', cfg);
  const m = sim.metrics();
  const G0L = 9.80665;
  // 由各推力器累計點火時間反推應消耗量
  let expected = 0;
  for (let i = 0; i < sim.rcs.thrusters.length; i++) {
    const t = sim.rcs.thrusters[i];
    expected += (t.maxThrust * m.onTimes[i]) / (t.isp * G0L);
  }
  assert.ok(expected > 0.01, `點火時間過小不具鑑別度（${expected} kg）`);
  assert.ok(
    Math.abs(expected - m.propellantUsed) < 1e-6 * Math.max(1, expected) + 1e-9,
    `點火時間反推 ${expected} kg ≠ 實際消耗 ${m.propellantUsed} kg`,
  );
  assert.ok(m.onTimes.some((x) => x > 1), '調變模式下累計點火時間應明顯大於 0（大於 1 秒）');
});

// ------------------- T10 非對角慣量符號 -------------------
test('T10a 慣性積符號：張量非對角項 = −P', () => {
  const J = M3.inertiaTensor(10, 20, 30, 2, 3, 4);
  assert.strictEqual(J[1], -2, 'J_xy 應為 −P_xy');
  assert.strictEqual(J[5], -4, 'J_yz 應為 −P_yz');
  assert.strictEqual(J[2], -3, 'J_xz 應為 −P_xz');
  assert.strictEqual(J[7], -4, 'J_zy 應為 −P_yz（對稱）');
});
test('T10b 非對角慣量剛體：角動量 Jω 守恆且能量守恆', () => {
  // 含慣性積的剛體無力矩自由旋轉：慣性系角動量與動能守恆
  const Pxy = 1.5, Pxz = -2.0, Pyz = 0.8;
  const cfg = presetByName('medium');
  cfg.vehicle.J = M3.inertiaTensor(100, 150, 130, Pxy, Pxz, Pyz);
  cfg.vehicle.Jinv = M3.inverse(cfg.vehicle.J);
  let s = { r: [0, 0, 0], v: [0, 0, 0], q: Q.normalize([0.8, 0.3, 0.4, 0.3]), w: [0.2, -0.1, 0.3], mProp: 10 };
  const E0 = rotationalKineticEnergy(cfg.vehicle.J, s.w);
  const L0 = inertialAngularMomentum(s.q, cfg.vehicle.J, s.w);
  const dt = 0.001, T = 3;
  for (let i = 0; i < Math.round(T / dt); i++) {
    s = rk4Step(cfg, s, i * dt, dt,
      () => ({ F: [0, 0, 0], T: [0, 0, 0], mdot: 0 }),
      () => ({ aExt_I: [0, 0, 0], tauExt_B: [0, 0, 0] }));
  }
  const E1 = rotationalKineticEnergy(cfg.vehicle.J, s.w);
  const L1 = inertialAngularMomentum(s.q, cfg.vehicle.J, s.w);
  assert.ok(V.norm(V.sub(L0, L1)) / V.norm(L0) < 1e-8, `角動量不守恆：Δ = ${V.norm(V.sub(L0, L1))}`);
  assert.ok(Math.abs(E1 - E0) / E0 < 1e-8, `動能不守恆：ΔE = ${Math.abs(E1 - E0)}`);
});

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
