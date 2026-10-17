// =====================================================================
// mathUtils.js — 向量、矩陣、四元數線性代數核心
// 所有方程式編號參照 docs/equations.md（E1.x = 動力學、E3.x = 四元數）
// 四元數慣例：q = [w, x, y, z]，純量在前（scalar-first），
//              代表 body→inertial 的主動旋轉：v_I = R(q) · v_B
// =====================================================================

export const V = {
  zeros: () => [0, 0, 0],
  of: (x, y, z) => [x, y, z],
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ],
  norm: (a) => Math.hypot(a[0], a[1], a[2]),
  normSq: (a) => a[0] * a[0] + a[1] * a[1] + a[2] * a[2],
  clone: (a) => [a[0], a[1], a[2]],
};

// ------------------- 3×3 矩陣（以一維陣列 row-major 儲存） -------------------
export const M3 = {
  // 由慣性張量輸入（工程慣性積，符號約定見 equations.md E1.2）
  inertiaTensor: (Ixx, Iyy, Izz, Ixy, Ixz, Iyz) => [
    Ixx, -Ixy, -Ixz,
    -Ixy, Iyy, -Iyz,
    -Ixz, -Iyz, Izz,
  ],
  mulVec: (m, v) => [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  ],
  // C = A·B（row-major 3×3）
  mul: (a, b) => {
    const c = new Array(9).fill(0);
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++) {
        let s = 0;
        for (let k = 0; k < 3; k++) s += a[i * 3 + k] * b[k * 3 + j];
        c[i * 3 + j] = s;
      }
    return c;
  },
  transpose: (m) => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]],
  add: (a, b) => a.map((x, i) => x + b[i]),
  scale: (m, s) => m.map((x) => x * s),
  identity: () => [1, 0, 0, 0, 1, 0, 0, 0, 1],
  // 高斯消去求逆（含部分樞軸選取）
  inverse: (m) => {
    const a = m.slice();
    const inv = M3.identity();
    for (let col = 0; col < 3; col++) {
      let piv = col;
      for (let r = col + 1; r < 3; r++)
        if (Math.abs(a[r * 3 + col]) > Math.abs(a[piv * 3 + col])) piv = r;
      if (Math.abs(a[piv * 3 + col]) < 1e-14) return null; // 奇異
      if (piv !== col) {
        for (let k = 0; k < 3; k++) {
          [a[col * 3 + k], a[piv * 3 + k]] = [a[piv * 3 + k], a[col * 3 + k]];
          [inv[col * 3 + k], inv[piv * 3 + k]] = [inv[piv * 3 + k], inv[col * 3 + k]];
        }
      }
      const d = a[col * 3 + col];
      for (let k = 0; k < 3; k++) { a[col * 3 + k] /= d; inv[col * 3 + k] /= d; }
      for (let r = 0; r < 3; r++) {
        if (r === col) continue;
        const f = a[r * 3 + col];
        if (f === 0) continue;
        for (let k = 0; k < 3; k++) {
          a[r * 3 + k] -= f * a[col * 3 + k];
          inv[r * 3 + k] -= f * inv[col * 3 + k];
        }
      }
    }
    return inv;
  },
  // 對稱矩陣的 Cholesky（正定檢查用），成功回傳下三角，失敗回傳 null
  cholesky: (m) => {
    const L = new Array(9).fill(0);
    for (let i = 0; i < 3; i++)
      for (let j = 0; j <= i; j++) {
        let s = m[i * 3 + j];
        for (let k = 0; k < j; k++) s -= L[i * 3 + k] * L[j * 3 + k];
        if (i === j) {
          if (s <= 0) return null;
          L[i * 3 + j] = Math.sqrt(s);
        } else L[i * 3 + j] = s / L[j * 3 + j];
      }
    return L;
  },
  // 特徵值（3×3 對稱矩陣，Jacobi 旋轉法）— 用於穩定性驗證
  eigenvaluesSym: (m) => {
    let a = m.slice();
    for (let sweep = 0; sweep < 50; sweep++) {
      // 找最大非對角元素
      let p = 0, q = 1, maxv = 0;
      for (let i = 0; i < 3; i++)
        for (let j = i + 1; j < 3; j++)
          if (Math.abs(a[i * 3 + j]) > maxv) { maxv = Math.abs(a[i * 3 + j]); p = i; q = j; }
      if (maxv < 1e-13) break;
      const app = a[p * 3 + p], aqq = a[q * 3 + q], apq = a[p * 3 + q];
      const theta = (aqq - app) / (2 * apq);
      const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      const b = a.slice();
      for (let k = 0; k < 3; k++) {
        b[p * 3 + k] = c * a[p * 3 + k] - s * a[q * 3 + k];
        b[q * 3 + k] = s * a[p * 3 + k] + c * a[q * 3 + k];
      }
      a = new Array(9).fill(0);
      for (let k = 0; k < 3; k++) {
        a[k * 3 + p] = c * b[k * 3 + p] - s * b[k * 3 + q];
        a[k * 3 + q] = s * b[k * 3 + p] + c * b[k * 3 + q];
      }
      for (let k = 0; k < 3; k++) {
        if (k !== p && k !== q) {
          a[p * 3 + k] = a[k * 3 + p];
          a[q * 3 + k] = a[k * 3 + q];
        }
      }
      a[p * 3 + q] = a[q * 3 + p] = 0;
    }
    return [a[0], a[4], a[8]];
  },
};

// ------------------- 四元數 q = [w, x, y, z]（純量在前） -------------------
export const Q = {
  identity: () => [1, 0, 0, 0],
  norm: (q) => Math.hypot(q[0], q[1], q[2], q[3]),
  normalize: (q) => {
    const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
    return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
  },
  // Hamilton 積
  mul: (a, b) => [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ],
  conjugate: (q) => [q[0], -q[1], -q[2], -q[3]],
  // R(q)：v_I = R·v_B（body→inertial）
  toMatrix: (q) => {
    const [w, x, y, z] = q;
    return [
      1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
      2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
      2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
    ];
  },
  fromMatrix: (Rm) => {
    const tr = Rm[0] + Rm[4] + Rm[8];
    let q;
    if (tr > 0) {
      const S = Math.sqrt(tr + 1) * 2;
      q = [S / 4, (Rm[7] - Rm[5]) / S, (Rm[2] - Rm[6]) / S, (Rm[3] - Rm[1]) / S];
    } else if (Rm[0] > Rm[4] && Rm[0] > Rm[8]) {
      const S = Math.sqrt(1 + Rm[0] - Rm[4] - Rm[8]) * 2;
      q = [(Rm[7] - Rm[5]) / S, S / 4, (Rm[1] + Rm[3]) / S, (Rm[2] + Rm[6]) / S];
    } else if (Rm[4] > Rm[8]) {
      const S = Math.sqrt(1 + Rm[4] - Rm[0] - Rm[8]) * 2;
      q = [(Rm[2] - Rm[6]) / S, (Rm[1] + Rm[3]) / S, S / 4, (Rm[5] + Rm[7]) / S];
    } else {
      const S = Math.sqrt(1 + Rm[8] - Rm[0] - Rm[4]) * 2;
      q = [(Rm[3] - Rm[1]) / S, (Rm[2] + Rm[6]) / S, (Rm[5] + Rm[7]) / S, S / 4];
    }
    return Q.normalize(q);
  },
  // 軸角（axis 為單位向量）
  fromAxisAngle: (axis, angle) => {
    const h = angle / 2;
    const s = Math.sin(h);
    return [Math.cos(h), axis[0] * s, axis[1] * s, axis[2] * s];
  },
  // 由 roll-pitch-yaw（3-2-1：yaw→pitch→roll）建立 q，角度為徑度
  fromEuler: (roll, pitch, yaw) => {
    const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
    const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
    const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
    // q = q_yaw(z) ⊗ q_pitch(y) ⊗ q_roll(x)
    return Q.normalize([
      cy * cp * cr + sy * sp * sr,
      cy * cp * sr - sy * sp * cr,
      cy * sp * cr + sy * cp * sr,
      sy * cp * cr - cy * sp * sr,
    ]);
  },
  // 轉為 roll(x), pitch(y), yaw(z)（3-2-1 序）
  toEuler: (q) => {
    const [w, x, y, z] = q;
    const sinr = 2 * (w * x + y * z);
    const cosr = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr, cosr);
    const sinp = 2 * (w * y - z * x);
    const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
    const siny = 2 * (w * z + x * y);
    const cosy = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny, cosy);
    return { roll, pitch, yaw };
  },
  // 誤差四元數 q_e = q⁻¹ ⊗ q_d（目前→目標，表達於 body 系）
  error: (q, qd) => Q.mul(Q.conjugate(q), qd),
  // 四元數的旋轉向量（log map）：返回 axis·angle（3 向量）
  log: (q) => {
    const v = [q[1], q[2], q[3]];
    const nv = V.norm(v);
    if (nv < 1e-12) return [0, 0, 0];
    const w = Math.min(1, Math.max(-1, q[0]));
    const angle = 2 * Math.atan2(nv, Math.abs(w)); // 取短旋轉（w<0 時等效 -q）
    const sign = q[0] >= 0 ? 1 : -1;
    return V.scale(v, (sign * angle) / nv);
  },
  // 以旋轉向量（3）建立四元數（exp map）
  exp: (v) => {
    const a = V.norm(v);
    if (a < 1e-12) return Q.identity();
    return Q.fromAxisAngle(V.scale(v, 1 / a), a);
  },
  // 將 v_B 轉到 inertial：v_I = R(q) v_B
  rotateToInertial: (q, vB) => M3.mulVec(Q.toMatrix(q), vB),
  // 將 v_I 轉到 body：v_B = R(q)ᵀ v_I
  rotateToBody: (q, vI) => M3.mulVec(M3.transpose(Q.toMatrix(q)), vI),
};

// ------------------- 一般線性代數（LQR / Lyapunov 求解用） -------------------
// 高斯消去解 A x = b（A: n×n row-major，b: n），含部分樞軸選取
export function solveLinear(A, b) {
  const n = b.length;
  const a = A.map((r) => r.slice());
  const x = b.slice();
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++)
      if (Math.abs(a[r][col]) > Math.abs(a[piv][col])) piv = r;
    if (Math.abs(a[piv][col]) < 1e-14) return null;
    if (piv !== col) { [a[col], a[piv]] = [a[piv], a[col]]; [x[col], x[piv]] = [x[piv], x[col]]; }
    const d = a[col][col];
    for (let k = col; k < n; k++) a[col][k] /= d;
    x[col] /= d;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = a[r][col];
      if (f === 0) continue;
      for (let k = col; k < n; k++) a[r][k] -= f * a[col][k];
      x[r] -= f * x[col];
    }
  }
  return x;
}

// 可重現的偽隨機數（mulberry32）— 供擾動與感測器雜訊取樣
export function makeRng(seed = 12345) {
  let s = seed >>> 0;
  return function rng() {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Box–Muller 常態樣本
export function gauss(rng) {
  let u = 0, v = 0;
  while (u === 0) u = rng();
  while (v === 0) v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

export const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
