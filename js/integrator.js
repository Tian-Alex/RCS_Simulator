// =====================================================================
// integrator.js — 定步長 RK4 積分器（於致動器事件之間）
// 狀態導數函數為純函數：RK4 各 stage 中閥門狀態與已取樣的擾動保持固定，
// 僅重新計算與狀態相關的力／力矩（重力梯度、SRP 依即時姿態計算）
// =====================================================================
import { derivative, addState, cloneState } from './dynamics.js';

export function rk4Step(cfg, state, t, h, wrenchOf, extAccelOf) {
  // wrenchOf(t, state) → {F, tau, mdot}：RCS 力／力矩（閥門固定）
  // extAccelOf(t, state) → {aExt_I, tauExt_B}：外部擾動（連續、純狀態函數）
  const f = (s) => {
    const w = wrenchOf(s);
    const e = extAccelOf(t, s);
    const tau = [
      w.T[0] + e.tauExt_B[0],
      w.T[1] + e.tauExt_B[1],
      w.T[2] + e.tauExt_B[2],
    ];
    return derivative(cfg, s, w.F, tau, e.aExt_I, w.mdot);
  };
  const k1 = f(state);
  const k2 = f(addState(state, k1, h / 2));
  const k3 = f(addState(state, k2, h / 2));
  const k4 = f(addState(state, k3, h));
  // 組合：s += h/6 (k1 + 2k2 + 2k3 + k4)（分別對 r,v,w 線性；q 用相同權重）
  const combine = (a, b, c, d) => [
    (a[0] + 2 * b[0] + 2 * c[0] + d[0]) / 6,
    (a[1] + 2 * b[1] + 2 * c[1] + d[1]) / 6,
    (a[2] + 2 * b[2] + 2 * c[2] + d[2]) / 6,
  ];
  const s2 = cloneState(state);
  s2.r = Vadd(state.r, combine(k1.r, k2.r, k3.r, k4.r), h);
  s2.v = Vadd(state.v, combine(k1.v, k2.v, k3.v, k4.v), h);
  s2.w = Vadd(state.w, combine(k1.w, k2.w, k3.w, k4.w), h);
  const dq = [
    (k1.q[0] + 2 * k2.q[0] + 2 * k3.q[0] + k4.q[0]) / 6,
    (k1.q[1] + 2 * k2.q[1] + 2 * k3.q[1] + k4.q[1]) / 6,
    (k1.q[2] + 2 * k2.q[2] + 2 * k3.q[2] + k4.q[2]) / 6,
    (k1.q[3] + 2 * k2.q[3] + 2 * k3.q[3] + k4.q[3]) / 6,
  ];
  s2.q = Qnorm([
    state.q[0] + h * dq[0], state.q[1] + h * dq[1],
    state.q[2] + h * dq[2], state.q[3] + h * dq[3],
  ]);
  s2.mProp = Math.max(0, state.mProp + h * (k1.mProp + 2 * k2.mProp + 2 * k3.mProp + k4.mProp) / 6);
  return s2;
}

// 局部小工具（避免循環依賴，重複實作極簡版本）
function Vadd(a, d, h) { return [a[0] + h * d[0], a[1] + h * d[1], a[2] + h * d[2]]; }
function Qnorm(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}
