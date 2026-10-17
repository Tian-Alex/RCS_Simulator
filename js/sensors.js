// =====================================================================
// sensors.js — 感測器模型（陀螺儀雜訊/偏置、姿態量測誤差）
// 控制器使用「估計值」q̂, ω̂（= 真值 + 誤差），而非真值
//   ω̂ = ω + bias + noise（零均值高斯，固定取樣間隔）
//   q̂ = δq ⊗ q（δq 為小隨機旋轉）
// =====================================================================
import { V, Q, makeRng, gauss } from './mathUtils.js';

export class SensorModel {
  constructor(params) {
    this.p = params;
    this.rng = makeRng((params.seed ?? 777) + 1);
    this.rateNoise = [0, 0, 0];
    this.attDelta = [0, 0, 0];
    this.nextSampleT = 0;
  }
  reset() {
    this.rng = makeRng((this.p.seed ?? 777) + 1);
    this.rateNoise = [0, 0, 0];
    this.attDelta = [0, 0, 0];
    this.nextSampleT = 0;
  }
  sample(t) {
    if (t >= this.nextSampleT) {
      const s = Math.sqrt(this.p.interval);
      this.rateNoise = [
        gauss(this.rng) * this.p.gyroNoise[0] * s,
        gauss(this.rng) * this.p.gyroNoise[1] * s,
        gauss(this.rng) * this.p.gyroNoise[2] * s,
      ];
      this.attDelta = [
        gauss(this.rng) * this.p.attNoise[0],
        gauss(this.rng) * this.p.attNoise[1],
        gauss(this.rng) * this.p.attNoise[2],
      ];
      this.nextSampleT = t + this.p.interval;
    }
  }
  estimate(t, q, w) {
    const wHat = V.add(w, this.rateNoise).map((x, i) => x + this.p.gyroBias[i]);
    const qMeas = Q.mul(q, Q.exp(this.attDelta));
    return { q: qMeas, w: wHat };
  }
}
