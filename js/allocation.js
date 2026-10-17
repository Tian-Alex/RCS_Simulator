// =====================================================================
// allocation.js — 有推力器飽和約束的力矩分配
// 控制鏈：控制器(連續力矩指令) → 分配器(推力向量 f) → PWPF/PWM 調變 → 閥門
// 求解：min ‖[T; √w_F·F_m]·f − [τ_cmd; 0]‖²,  s.t. 0 ≤ f_i ≤ F_max,i
//   T: 3×N 力矩矩陣（每推力器單位推力的力矩）、F_m: 3×N 力矩陣
//   演算法：Lawson–Hanson 非負最小平方法（NNLS）+ 上界飽和迭代（docs §4）
//   （直接偽逆解的負推力分量經裁剪後會收斂到次優解，故必須用 NNLS）
// =====================================================================
import { solveLinear } from './mathUtils.js';

// 建立分配矩陣
export function buildMatrices(rcs) {
  const N = rcs.thrusters.length;
  const T = [[], [], []];   // T[axis][i]
  const Fm = [[], [], []];
  for (let i = 0; i < N; i++) {
    const w = rcs.wrenchOf(i);
    for (let a = 0; a < 3; a++) { T[a].push(w.tau[a]); Fm[a].push(w.f[a]); }
  }
  return { T, Fm, N };
}

// ------------------- Lawson–Hanson NNLS -------------------
// 解 min ‖C x − d‖², s.t. x ≥ 0
// C: m×n（row-major 二維）、d: m
export function nnls(C, d, maxIter = 30 * C[0].length) {
  const m = C.length, n = C[0].length;
  const x = new Array(n).fill(0);
  const passive = new Array(n).fill(false); // 被動集（允許非零）
  const Ct = C[0].map((_, j) => C.map((row) => row[j])); // Cᵀ (n×m)

  for (let iter = 0; iter < maxIter; iter++) {
    // 梯度 w = Cᵀ(d − Cx)：僅對作用集（x_i = 0）考慮
    const Cx = new Array(m).fill(0);
    for (let i = 0; i < m; i++)
      for (let j = 0; j < n; j++) Cx[i] += C[i][j] * x[j];
    const w = new Array(n).fill(0);
    let wMax = 0, wArg = -1;
    for (let j = 0; j < n; j++) {
      if (passive[j]) continue;
      let g = 0;
      for (let i = 0; i < m; i++) g += Ct[j][i] * (d[i] - Cx[i]);
      if (g > wMax) { wMax = g; wArg = j; }
    }
    if (wArg < 0) break; // 所有梯度 ≤ 0 → 最佳解
    passive[wArg] = true;

    // 內層循環：在被動集上解 LS，並移除負分量
    let inner = true;
    while (inner) {
      const idx = [];
      for (let j = 0; j < n; j++) if (passive[j]) idx.push(j);
      // 建立法方程 (C_PᵀC_P) z = C_Pᵀd
      const k = idx.length;
      const A = Array.from({ length: k }, () => new Array(k).fill(0));
      const b = new Array(k).fill(0);
      for (let a = 0; a < k; a++) {
        for (let c = 0; c < k; c++) {
          let s = 0;
          for (let i = 0; i < m; i++) s += Ct[idx[a]][i] * Ct[idx[c]][i];
          A[a][c] = s;
        }
        let s = 0;
        for (let i = 0; i < m; i++) s += Ct[idx[a]][i] * d[i];
        b[a] = s;
      }
      for (let a = 0; a < k; a++) A[a][a] += 1e-12; // 微正則化
      const z = solveLinear(A, b);
      if (!z) { passive[wArg] = false; inner = false; break; }
      // 檢查負分量
      let neg = false;
      for (let a = 0; a < k; a++) if (z[a] < -1e-12) neg = true;
      if (!neg) {
        for (let a = 0; a < k; a++) x[idx[a]] = z[a];
        inner = false;
        break;
      }
      // 沿 x→z 線性內插至第一個分量歸零
      let alpha = Infinity;
      for (let a = 0; a < k; a++)
        if (z[a] < 0) alpha = Math.min(alpha, x[idx[a]] / (x[idx[a]] - z[a]));
      if (!isFinite(alpha)) { passive[wArg] = false; inner = false; break; }
      for (let a = 0; a < k; a++) x[idx[a]] += alpha * (z[a] - x[idx[a]]);
      for (let a = 0; a < k; a++)
        if (x[idx[a]] < 1e-12) { x[idx[a]] = 0; passive[idx[a]] = false; }
      if (!passive.some(Boolean)) { inner = false; break; }
    }
  }
  return x;
}

// 有上界的分配：NNLS + 飽和迭代
export function allocate(rcs, tauCmd, forceCmd = null, wF = 0.1, maxIter = 12) {
  const { T, Fm, N } = buildMatrices(rcs);
  const fmax = rcs.thrusters.map((t) => (t.enabled ? t.maxThrust : 0));
  if (N === 0) return [];
  // 加權矩陣 C = [T; √wF·F_m]（6×N）、d = [τ; √wF·f_des]
  const C = [];
  const d = [];
  for (let a = 0; a < 3; a++) {
    C.push(T[a].map((v) => v));
    d.push(tauCmd[a]);
  }
  for (let a = 0; a < 3; a++) {
    C.push(Fm[a].map((v) => v * Math.sqrt(wF)));
    d.push((forceCmd ? forceCmd[a] : 0) * Math.sqrt(wF));
  }
  const f = new Array(N).fill(0);
  const fixed = new Array(N).fill(false);
  for (let iter = 0; iter < maxIter; iter++) {
    const dRes = d.slice();
    for (let i = 0; i < N; i++)
      if (fixed[i]) for (let r = 0; r < 6; r++) dRes[r] -= C[r][i] * f[i];
    const freeCols = [];
    const Csub = C.map((row) => row.filter((_, i) => !fixed[i]));
    const x = nnls(Csub, dRes);
    let changed = false;
    let xi = 0;
    for (let i = 0; i < N; i++) {
      if (fixed[i]) continue;
      const c = Math.min(fmax[i], Math.max(0, x[xi++]));
      if (Math.abs(c - f[i]) > 1e-12) changed = true;
      f[i] = c;
      if (c >= fmax[i] - 1e-12 && fmax[i] > 0) { fixed[i] = true; f[i] = fmax[i]; }
    }
    if (!changed) break;
  }
  return f;
}

// 軸向可達力矩：對單軸給極大指令，取實際達成值（作為 PWPF/PWM 滿刻度）
export function axisTorqueCapacity(rcs, wF = 0.1) {
  const cap = [];
  for (let a = 0; a < 3; a++) {
    const tau = [0, 0, 0];
    tau[a] = 1e9; // 遠超飽和 → 得到該軸最大可達力矩
    const f = allocate(rcs, tau, null, wF);
    const { T } = buildMatrices(rcs);
    let achieved = 0;
    for (let i = 0; i < f.length; i++) achieved += T[a][i] * f[i];
    cap.push(Math.max(achieved, 1e-12));
  }
  return cap;
}

// 可控性檢查：非負推力是否能產生雙向力矩（每軸正/負皆可達）
export function checkAttainability(rcs) {
  const cap = axisTorqueCapacity(rcs);
  const neg = [];
  for (let a = 0; a < 3; a++) {
    const tau = [0, 0, 0];
    tau[a] = -1e9;
    const f = allocate(rcs, tau, null, 0.1);
    const { T } = buildMatrices(rcs);
    let achieved = 0;
    for (let i = 0; i < f.length; i++) achieved += T[a][i] * f[i];
    neg.push(-achieved);
  }
  return {
    pos: cap,
    neg,
    bidirectional: cap.map((c, a) => c > 1e-6 && neg[a] > 1e-6),
  };
}
