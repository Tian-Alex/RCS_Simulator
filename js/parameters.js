// =====================================================================
// parameters.js — 預設參數與三個參考 Case（docs §10）
// 單位：SI（m, kg, s, N, N·m, rad→度由 UI 轉換）
// 座標系：body X = 縱軸（鼻錐方向），Y/Z 橫向；原點 = 幾何中心
// =====================================================================

// 12 具推力器：3 軸 × 雙向 × 純力偶（每對淨力為零）
// 佈局說明見 docs §3.4：
//   Roll  (X)：鼻環 4 具（位置 ±r 側向，推力反向切向）
//   Pitch (Y)：鼻/尾各 2 具（±Z 推力）
//   Yaw   (Z)：鼻/尾各 2 具（±Y 推力）
function makeThrusterSet(L, r, F, minImpulse, isp, delayOpen = 0.015, delayClose = 0.01, duty = 0.9) {
  const xf = L / 2, xt = -L / 2;
  const T = (name, pos, dir) => ({
    name, pos, dir, maxThrust: F, minImpulse, delayOpen, delayClose,
    maxDuty: duty, dutyWindow: 10, isp, enabled: true,
  });
  return [
    // Roll +X / −X（鼻環）
    T('R1(+X)', [xf, 0, r], [0, -1, 0]), T('R2(+X)', [xf, 0, -r], [0, 1, 0]),
    T('R3(−X)', [xf, 0, r], [0, 1, 0]), T('R4(−X)', [xf, 0, -r], [0, -1, 0]),
    // Pitch +Y / −Y
    T('P1(+Y)', [xf, 0, r], [0, 0, -1]), T('P2(+Y)', [xt, 0, -r], [0, 0, 1]),
    T('P3(−Y)', [xf, 0, -r], [0, 0, 1]), T('P4(−Y)', [xt, 0, r], [0, 0, -1]),
    // Yaw +Z / −Z
    T('Y1(+Z)', [xf, -r, 0], [0, 1, 0]), T('Y2(+Z)', [xt, r, 0], [0, -1, 0]),
    T('Y3(−Z)', [xf, r, 0], [0, -1, 0]), T('Y4(−Z)', [xt, -r, 0], [0, 1, 0]),
  ];
}

function baseConfig() {
  return {
    sim: {
      duration: 120,            // s
      dt: 0.005,                 // 物理步長 [s]
      controlPeriod: 0.1,         // 控制器取樣週期 [s]
      recordInterval: 0.05,       // 記錄間隔 [s]
      seed: 12345,
      settleBandDeg: 0.5,        // 穩定判據帶寬
    },
    vehicle: {
      dryMass: 2000, mProp: 40,
      length: 4, radius: 1.2,
      cg: [0, 0, 0],
      Ixx: 1440, Iyy: 3387, Izz: 3387,
      Ixy: 0, Ixz: 0, Iyz: 0,
    },
    initial: {
      attitude: [30, -20, 45],  // roll, pitch, yaw [deg]
      rates: [2, -3, 1],        // [deg/s]
      position: [0, 0, 0],
      velocity: [0, 0, 0],
    },
    target: { attitude: [0, 0, 0] },
    rcs: { thrusters: [] },
    controller: {
      type: 'pid',
      params: {
        gains: [[76, 915, 0.1], [76, 915, 0.1], [76, 915, 0.1]], // [Kp, Kd, Ki] per axis（Ti = Kp/Ki ≈ 760 s）
        intLimit: [5, 5, 5],
      },
    },
    modulator: {
      mode: 'pwpf',
      pwmPeriod: 1.0,
      pwpf: { Km: 2.0, Tm: 0.15, Uon: 0.05, Uoff: 0.025 },
    },
    allocation: { wForce: 0.1 },
    disturbances: {
      constantTorque: [0.2, -0.1, 0.05],
      randomTorque: { sigma: [0.05, 0.05, 0.05], interval: 0.5, seed: 4242 },
      gravityGradient: { enabled: false, altitude: 500e3, phase0: 0 },
      solarPressure: {
        enabled: false, sunDirection: [1, 0, 0], Cr: 1.3,
        area: 5, cpOffset: [0, 0.1, 0],
      },
    },
    sensors: {
      enabled: true,
      gyroNoise: [0.001, 0.001, 0.001],   // rad/s/√Hz
      gyroBias: [0, 0, 0],                 // rad/s
      attNoise: [0.0005, 0.0005, 0.0005], // rad
      interval: 0.1, seed: 777,
    },
  };
}

// 三個參考 Case
export function presetByName(name) {
  const cfg = baseConfig();
  if (name === 'small') {
    Object.assign(cfg.vehicle, {
      dryMass: 60, mProp: 2, length: 0.8, radius: 0.2,
      Ixx: 1.2, Iyy: 3.8, Izz: 3.8,
    });
    cfg.rcs.thrusters = makeThrusterSet(0.8, 0.2, 1, 0.01, 65, 0.008, 0.005, 0.9);
    cfg.controller.params.gains = [[0.09, 1.05, 0.01], [0.09, 1.05, 0.01], [0.09, 1.05, 0.01]];
    cfg.controller.params.intLimit = [10, 10, 10]; // 依小容量縮放（Ki·intLimit ≈ 0.1 N·m）
    // 註：PWPF 閾值為軸容量百分比（調變器輸入已正規化），毋須隨案例縮放
    Object.assign(cfg.disturbances, {
      constantTorque: [0.005, -0.002, 0.001],
      randomTorque: { sigma: [0.001, 0.001, 0.001], interval: 0.5, seed: 4242 },
    });
    cfg.sim.duration = 100;
    cfg.sim.settleBandDeg = 1.0; // 立方衛星級最小衝量較大，極限環振幅約 0.5–1°
  } else if (name === 'large') {
    Object.assign(cfg.vehicle, {
      dryMass: 15000, mProp: 200, length: 12, radius: 1.9,
      Ixx: 27075, Iyy: 193500, Izz: 193500,
    });
    cfg.rcs.thrusters = makeThrusterSet(12, 1.9, 150, 0.75, 65, 0.02, 0.015, 0.8);
    cfg.controller.params.gains = [[610, 7370, 40], [4300, 52000, 300], [4300, 52000, 300]];
    cfg.sim.duration = 200;
  } else {
    // medium（預設）
    cfg.rcs.thrusters = makeThrusterSet(4, 1.2, 10, 0.05, 65, 0.015, 0.01, 0.9);
  }
  return cfg;
}

export const PRESETS = [
  { id: 'small', label: '小型火箭（立方衛星級）' },
  { id: 'medium', label: '中型上面級（預設）' },
  { id: 'large', label: '大型高慣量運載火箭' },
];

export const CONTROLLERS = [
  { id: 'pid', label: 'PID' },
  { id: 'lqr', label: 'LQR' },
  { id: 'qpd', label: 'Quaternion PD' },
];

export const MODULATORS = [
  { id: 'pwpf', label: 'PWPF（脈寬脈頻調變）' },
  { id: 'pwm', label: 'PWM（脈寬調變）' },
  { id: 'continuous', label: '理想連續推力（無調變）' },
];

export function defaultConfig() { return presetByName('medium'); }
