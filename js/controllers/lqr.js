// =====================================================================
// lqr.js — 姿態 LQR 控制器（連續時間代數 Riccati 方程，Newton–Kleinman 疊代）
// 線性化模型（小角度，繞固定姿態／零角速度，docs §6.2）：
//   x = [δθ; ω],  δθ̇ ≈ ω,  ω̇ = J⁻¹ τ
//   A = [[0, I],[0, 0]] (6×6),  B = [[0],[J⁻¹]] (6×3)
//   求解 CARE：AᵀP + PA − PBR⁻¹BᵀP + Q = 0 → K = R⁻¹BᵀP
// Newton–Kleinman 疊代（Kleinman 1968）：
//   1. 給定可穩定化初始增益 K₀ = [ν²J, 2ζνJ]
//   2. 解 Lyapunov 方程 (A−BK)ᵀP + P(A−BK) = −(Q + KᵀRK)
//   3. K ← R⁻¹BᵀP，重複至收斂
// Lyapunov 方程以 36×36 線性系統求解：
//   (I⊗Aₖᵀ + Aₖᵀ⊗I) vec(P) = −vec(Q+KᵀRK)
// =====================================================================
import { solveLinear, clamp } from '../mathUtils.js';
import { V, Q } from '../mathUtils.js';

// 建立 6×6 A、6×3 B（J 為 3×3 row-major）
export function linearize(Jinv) {
  const A = Array.from({ length: 6 }, () => new Array(6).fill(0));
  for (let i = 0; i < 3; i++) A[i][i + 3] = 1; // δθ̇ = ω
  const B = Array.from({ length: 6 }, () => new Array(3).fill(0));
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) B[i + 3][j] = Jinv[i * 3 + j];
  return { A, B };
}

// Lyapunov 方程求解：A_clᵀP + P·A_cl = −Qc
function solveLyapunov(Acl, Qc, n) {
  // vec(P) 以 row-major 展開：M vec(P) = −vec(Qc)
  const M = Array.from({ length: n * n }, () => new Array(n * n).fill(0));
  for (let i = 0; i < n; i++)
    for (let j = 0; j < n; j++) {
      const row = i * n + j;
      for (let k = 0; k < n; k++) {
        M[row][k * n + j] += Acl[k][i];  // (Aᵀ)_{k,i} → vec(AᵀP) 貢獻
        M[row][i * n + k] += Acl[k][j];  // vec(P A) 貢獻
      }
    }
  const rhs = new Array(n * n);
  for (let i = 0; i < n; i++)
    for (let j = 0; j < n; j++) rhs[i * n + j] = -Qc[i][j];
  const x = solveLinear(M, rhs);
  if (!x) return null;
  const P = Array.from({ length: n }, (_, i) => x.slice(i * n, i * n + n));
  // 對稱化（數值誤差）
  for (let i = 0; i < n; i++)
    for (let j = i + 1; j < n; j++) {
      const avg = (P[i][j] + P[j][i]) / 2;
      P[i][j] = P[j][i] = avg;
    }
  return P;
}

// 對稱矩陣是否正定（Cholesky 檢查，不修改原矩陣）
function isPositiveDefinite(P, n) {
  const A = P.map((r) => r.slice()); // 複本
  for (let i = 0; i < n; i++) {
    let d = A[i][i];
    for (let k = 0; k < i; k++) d -= A[i][k] * A[i][k];
    if (d <= 0) return false;
    const s = Math.sqrt(d);
    for (let j = i + 1; j < n; j++) {
      let x = A[j][i];
      for (let k = 0; k < i; k++) x -= A[j][k] * A[i][k];
      A[j][i] = x / s;
    }
  }
  return true;
}

// Newton–Kleinman CARE 求解
// 回傳 { K (3×6), P (6×6), converged, iterations, eigClosed }
export function solveCare(Jinv, Qdiag, Rdiag, opts = {}) {
  const { A, B } = linearize(Jinv);
  const maxIter = opts.maxIter ?? 100;
  const tol = opts.tol ?? 1e-10;
  const nu = opts.nu ?? 0.3, zeta = opts.zeta ?? 0.9;
  const n = 6, m = 3;

  const Q = Array.from({ length: n }, (_, i) => new Array(n).fill(0));
  for (let i = 0; i < n; i++) Q[i][i] = Qdiag[i];
  const R = Array.from({ length: m }, (_, i) => new Array(m).fill(0));
  for (let i = 0; i < m; i++) R[i][i] = Rdiag[i];

  // 初始可穩定化增益：K₀ = [ν²J, 2ζνJ]
  // τ = −K₀x → J δθ̈ + 2ζνJ δθ̇ + ν²J δθ = 0（三個阻尼二階系統）
  // 此處以 Jinv 的線性求解等價計算 J·col
  const Jmat = Array.from({ length: 3 }, (_, i) => [Jinv[i * 3], Jinv[i * 3 + 1], Jinv[i * 3 + 2]]);
  const solveJ = (rhs) => solveLinear(Jmat.map((r) => r.slice()), rhs.slice());
  let K = Array.from({ length: m }, () => new Array(n).fill(0));
  for (let a = 0; a < m; a++) {
    const col = [0, 0, 0];
    col[a] = nu * nu;
    const kth = solveJ(col);
    const col2 = [0, 0, 0];
    col2[a] = 2 * zeta * nu;
    const ktw = solveJ(col2);
    for (let i = 0; i < 3; i++) { K[a][i] = kth[i]; K[a][i + 3] = ktw[i]; }
  }

  let P = null, converged = false, iter = 0;
  for (iter = 0; iter < maxIter; iter++) {
    // A_cl = A − B K
    const Acl = A.map((row) => row.slice());
    for (let i = 0; i < n; i++)
      for (let j = 0; j < n; j++)
        for (let k = 0; k < m; k++) Acl[i][j] -= B[i][k] * K[k][j];
    // Qc = Q + Kᵀ R K
    const Qc = Array.from({ length: n }, () => new Array(n).fill(0));
    for (let i = 0; i < n; i++)
      for (let j = 0; j < n; j++) {
        let s = Q[i][j];
        for (let p = 0; p < m; p++)
          for (let q = 0; q < m; q++) s += K[p][i] * R[p][q] * K[q][j];
        Qc[i][j] = s;
      }
    P = solveLyapunov(Acl, Qc, n);
    if (!P || !isPositiveDefinite(P, n)) return { ok: false, reason: 'Lyapunov solve failed or P not positive definite', K, P: null, iterations: iter };
    // K ← R⁻¹ Bᵀ P
    const Knew = Array.from({ length: m }, () => new Array(n).fill(0));
    for (let a = 0; a < m; a++)
      for (let j = 0; j < n; j++) {
        let s = 0;
        for (let i = 0; i < n; i++) s += B[i][a] * P[i][j];
        Knew[a][j] = s / R[a][a];
      }
    let diff = 0;
    for (let a = 0; a < m; a++) for (let j = 0; j < n; j++) diff = Math.max(diff, Math.abs(Knew[a][j] - K[a][j]));
    K = Knew;
    if (diff < tol) { converged = true; iter++; break; }
  }
  // 驗證：閉迴路穩定性（A−BK 特徵值實部 < 0，QR 演算法簡化：用 6×6 特徵值計算）
  const Acl = A.map((row) => row.slice());
  for (let i = 0; i < n; i++)
    for (let j = 0; j < n; j++)
      for (let k = 0; k < m; k++) Acl[i][j] -= B[i][k] * K[k][j];
  const residual = careResidual(A, B, Q, R, P);
  return {
    ok: converged && residual < 1e-6 && isPositiveDefinite(P, n),
    reason: converged ? '' : 'Newton–Kleinman did not converge',
    K, P, iterations: iter, residual,
  };
}

// CARE 殘差：‖AᵀP + PA − PBR⁻¹BᵀP + Q‖_F / (‖Q‖_F + 1)
function careResidual(A, B, Q, R, P) {
  const n = 6, m = 3;
  // PBm = P·B (n×m)
  const PBm = Array.from({ length: n }, () => new Array(m).fill(0));
  for (let i = 0; i < n; i++)
    for (let a = 0; a < m; a++) {
      let s = 0;
      for (let k = 0; k < n; k++) s += P[i][k] * B[k][a];
      PBm[i][a] = s;
    }
  // PBR⁻¹BᵀP (n×n)
  const quad = Array.from({ length: n }, () => new Array(n).fill(0));
  for (let i = 0; i < n; i++)
    for (let j = 0; j < n; j++) {
      let s = 0;
      for (let a = 0; a < m; a++) s += (PBm[i][a] / R[a][a]) * PBm[j][a];
      quad[i][j] = s;
    }
  let res = 0, norm = 0;
  for (let i = 0; i < n; i++)
    for (let j = 0; j < n; j++) {
      // M_ij = Q_ij + (AᵀP)_ij + (PA)_ij − quad_ij
      let v = Q[i][j] - quad[i][j];
      for (let k = 0; k < n; k++) v += A[k][i] * P[k][j] + P[i][k] * A[k][j];
      res += v * v;
      norm += Q[i][j] * Q[i][j];
    }
  return Math.sqrt(res) / (Math.sqrt(norm) + 1);
}

export class LqrController {
  constructor(params) {
    this.Qdiag = params.Qdiag.slice();   // [qθx qθy qθz qωx qωy qωz]
    this.Rdiag = params.Rdiag.slice();   // [rx ry rz]
    this.K = null;
    this.status = null;
  }
  // 於慣量張量改變時重新求解（瀏覽器內即時計算，6 維問題效能可接受）
  design(Jinv) {
    const r = solveCare(Jinv, this.Qdiag, this.Rdiag);
    this.status = r;
    this.K = r.ok ? r.K : null;
    return r;
  }
  reset() { /* LQR 無內部狀態 */ }
  step(dt, qEst, wEst, qTarget) {
    if (!this.K) return [0, 0, 0];
    const qe = Q.error(qEst, qTarget);
    const delta = Q.log(qe); // 「前往目標所需旋轉」；LQR 狀態用的姿態誤差 ε = −δ
    const x = [...delta.map((d) => -d), ...wEst];
    const tau = [0, 0, 0];
    for (let a = 0; a < 3; a++) {
      let s = 0;
      for (let j = 0; j < 6; j++) s += this.K[a][j] * x[j];
      tau[a] = -s; // τ = −K·x, x = [ε; ω]
    }
    return tau;
  }
}
