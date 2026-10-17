// =====================================================================
// pid.js — 三軸 PID 姿態控制器
// 誤差使用旋轉向量 δ = log(q⁻¹⊗q_d)（「前往目標所需的旋轉」，避免 Euler 角奇異，docs §6.1）
//   τ_cmd = Kp·δ − Kd·ω + Ki·∫δ dt（積分項含抗飽和）
//   注意 δ̇ ≈ −ω（轉向目標時誤差遞減），故微分項為 −Kd·ω
// =====================================================================
import { V, Q, clamp } from '../mathUtils.js';

export class PidController {
  constructor(params) {
    // 每軸增益：[[Kp_x, Kd_x, Ki_x], ...]
    this.gains = params.gains.map((g) => [g[0], g[1], g[2]]);
    this.intLimit = params.intLimit ?? [5, 5, 5]; // 積分飽和上限 [N·m·s]
    this.intWindow = params.intWindow ?? 0.1;      // 條件積分視窗 [rad]（約 5.7°）
    this.reset();
  }
  reset() {
    this.integral = [0, 0, 0];
    this.cmd = [0, 0, 0];
  }
  // q_est, w_est：感測器提供之估計值；q_target：目標姿態
  // sat：飽和資訊（分配器達成率 0..1），用於積分抗飽和
  step(dt, qEst, wEst, qTarget, sat = 1) {
    const qe = Q.error(qEst, qTarget);
    const delta = Q.log(qe); // 旋轉向量誤差（body 系，指向目標）
    // 積分（前向歐拉）+ 條件積分抗飽和（docs §6.1）：
    //   1. 力矩飽和時凍結（sat < 0.98）
    //   2. 大角度機動期間不積分（|δ| > intWindow）
    //   避免機動過程積分上捲（windup）造成反向慢速振盪
    for (let a = 0; a < 3; a++) {
      if (sat >= 0.98 && Math.abs(delta[a]) < this.intWindow)
        this.integral[a] = clamp(this.integral[a] + delta[a] * dt, -this.intLimit[a], this.intLimit[a]);
    }
    const tau = [0, 0, 0];
    for (let a = 0; a < 3; a++) {
      tau[a] = this.gains[a][0] * delta[a] - this.gains[a][1] * wEst[a] + this.gains[a][2] * this.integral[a];
    }
    this.cmd = tau;
    return tau;
  }
}
