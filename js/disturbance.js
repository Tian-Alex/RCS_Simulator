// =====================================================================
// disturbance.js — 外部擾動力矩
//   1. 常值力矩（body 系）
//   2. 隨機力矩：以固定取樣間隔的零均值高斯白雜訊（種子化，可重現）
//   3. 重力梯度：τ_gg = 3μ/r³ · r̂_B × (J·r̂_B)（docs §7.1，Wertz §16）
//      採「給定圓軌道」近似：軌道半徑由高度設定，近地方向以軌道率 n 旋轉
//   4. 太陽輻射壓：F_srp = −P_sr·Cr·A·Ŝ_B（docs §7.2）
//      P_sr = 4.56×10⁻⁶ N/m²（1 AU），常數面積、無蝕近似
// =====================================================================
import { V, Q, M3, makeRng, gauss } from './mathUtils.js';

export const MU_EARTH = 3.986004418e14; // 地球重力參數 [m³/s²]
export const P_SRP_1AU = 4.56e-6;       // 太陽輻射壓 @1AU [N/m²]

export class DisturbanceModel {
  constructor(params) {
    this.p = params;
    this.rng = makeRng(params.seed ?? 12345);
    this.randomSample = [0, 0, 0];
    this.nextSampleT = 0;
  }
  reset() {
    this.rng = makeRng(this.p.seed ?? 12345);
    this.randomSample = [0, 0, 0];
    this.nextSampleT = 0;
  }
  // 隨機力矩取樣（固定間隔，與渲染/RK4 呼叫頻率無關）
  sampleRandom(t) {
    if (t >= this.nextSampleT) {
      const s = Math.sqrt(this.p.randomTorque.interval);
      this.randomSample = [
        gauss(this.rng) * this.p.randomTorque.sigma[0] * s,
        gauss(this.rng) * this.p.randomTorque.sigma[1] * s,
        gauss(this.rng) * this.p.randomTorque.sigma[2] * s,
      ];
      this.nextSampleT = t + this.p.randomTorque.interval;
    }
  }
  // 近地方向（慣性系）：給定圓軌道，初始相位 θ0，軌道率 n = √(μ/r³)
  nadirInertial(t) {
    const r = this.p.gravityGradient.orbitRadius;
    const n = Math.sqrt(MU_EARTH / (r * r * r));
    const th = this.p.gravityGradient.phase0 + n * t;
    return [Math.cos(th), Math.sin(th), 0];
  }
  // 總擾動力矩（body 系）
  torque(t, q, J) {
    let tau = [0, 0, 0];
    // 常值
    const c = this.p.constantTorque;
    tau = V.add(tau, [c[0], c[1], c[2]]);
    // 隨機（已取樣）
    tau = V.add(tau, this.randomSample);
    // 重力梯度
    if (this.p.gravityGradient.enabled) {
      const rHatI = this.nadirInertial(t);
      const rHatB = Q.rotateToBody(q, rHatI);
      const r = this.p.gravityGradient.orbitRadius;
      const coef = 3 * MU_EARTH / (r * r * r);
      tau = V.add(tau, V.cross(rHatB, M3.mulVec(J, rHatB)).map((x) => x * coef));
    }
    // 太陽輻射壓：τ = r_cp × F_srp
    if (this.p.solarPressure.enabled) {
      const sp = this.p.solarPressure;
      const sunB = Q.rotateToBody(q, sp.sunDirection);
      // F = −P·Cr·A·Ŝ（力背離太陽方向）
      const F = V.scale(sunB, -P_SRP_1AU * sp.Cr * sp.area);
      const rcp = [sp.cpOffset[0], sp.cpOffset[1], sp.cpOffset[2]];
      tau = V.add(tau, V.cross(rcp, F));
    }
    return tau;
  }
}
