// =====================================================================
// dynamics.js — 剛體 6-DOF 動力學（Newton–Euler + 四元數運動學）
// 方程式編號參照 docs/equations.md §1–§2
//   質心平移（慣性系）：E1.1  ṙ = v,  v̇ = (R(q)·F_B + F_ext,I)/m
//   姿態運動學：        E2.1  q̇ = ½ q ⊗ [0, ω_B]
//   姿態動力學：        E1.3  J ω̇ + ω × (J ω) = τ_B
// 狀態向量 state = { r:[3], v:[3], q:[4], w:[3], mProp:number }
// =====================================================================
import { V, M3, Q } from './mathUtils.js';

export const G0 = 9.80665; // 標準重力加速度 [m/s²]（ISO 80000-3 / docs §1）

// 建立初始狀態
export function makeState() {
  return { r: [0, 0, 0], v: [0, 0, 0], q: Q.identity(), w: [0, 0, 0], mProp: 0 };
}

export function cloneState(s) {
  return { r: s.r.slice(), v: s.v.slice(), q: s.q.slice(), w: s.w.slice(), mProp: s.mProp };
}

// 質量（乾重 + 推進劑）
export function totalMass(cfg, s) {
  return cfg.vehicle.dryMass + Math.max(0, s.mProp);
}

// =====================================================================
// 純粹的狀態導數（不含任何副作用；RK4 各 stage 重複呼叫）
//   cfg      : 全域參數（vehicle、環境…）
//   s        : 當前狀態
//   F_B      : 本體座標系合力（RCS）[N]
//   tau_B    : 本體座標系合力矩（RCS + 擾動已由呼叫者合併）[N·m]
//   aExt_I   : 外部加速度（慣性系）[m/s²]
//   mdot     : 推進劑流量（負值）[kg/s]
// 回傳 dState/dt（新物件）
// =====================================================================
export function derivative(cfg, s, F_B, tau_B, aExt_I, mdot) {
  const m = totalMass(cfg, s);
  const Rq = Q.toMatrix(s.q);
  // v̇ = R(q)F_B/m + a_ext,I        (E1.1)
  const F_I = M3.mulVec(Rq, F_B);
  const dv = V.add(V.scale(F_I, 1 / m), aExt_I);
  // q̇ = ½ q ⊗ [0, ω]              (E2.1)
  const w4 = [0, s.w[0], s.w[1], s.w[2]];
  const dq4 = Q.mul(s.q, w4);
  const dq = [dq4[0] / 2, dq4[1] / 2, dq4[2] / 2, dq4[3] / 2];
  // ω̇ = J⁻¹ (τ − ω × Jω)           (E1.3)
  const Jw = M3.mulVec(cfg.vehicle.J, s.w);
  const gyroTerm = V.cross(s.w, Jw);
  const rhs = V.sub(tau_B, gyroTerm);
  const Jinv = cfg.vehicle.Jinv; // 預先求逆（parameters 更新時計算）
  const dw = M3.mulVec(Jinv, rhs);
  return {
    r: s.v.slice(),
    v: dv,
    q: dq,
    w: dw,
    mProp: mdot,
  };
}

// 狀態相加（RK4 用）：s + h·ds
export function addState(s, ds, h) {
  return {
    r: V.add(s.r, V.scale(ds.r, h)),
    v: V.add(s.v, V.scale(ds.v, h)),
    q: Q.normalize([
      s.q[0] + h * ds.q[0], s.q[1] + h * ds.q[1],
      s.q[2] + h * ds.q[2], s.q[3] + h * ds.q[3],
    ]),
    w: V.add(s.w, V.scale(ds.w, h)),
    mProp: s.mProp + h * ds.mProp,
  };
}

// 剛體動力學不變量（測試用）
export function rotationalKineticEnergy(J, w) {
  return 0.5 * V.dot(w, M3.mulVec(J, w));
}
export function inertialAngularMomentum(q, J, w) {
  const hB = M3.mulVec(J, w);
  return Q.rotateToInertial(q, hB);
}
