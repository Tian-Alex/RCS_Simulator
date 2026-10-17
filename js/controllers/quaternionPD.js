// =====================================================================
// quaternionPD.js — 四元數 PD 姿態控制器（全域／大角度機動適用）
// τ_cmd = 2·Kp·sign(qe_w)·qe_vec − Kd·ω         （docs §6.3）
// qe = q⁻¹ ⊗ q_d（「前往目標所需旋轉」的誤差四元數，向量部分指向目標）
// sign(qe_w) 處理雙重覆蓋（q 與 −q 等價）；此非線性回授具 Lyapunov 穩定性
// （Wie, Space Vehicle Dynamics and Control §8.5）
// =====================================================================
import { Q } from '../mathUtils.js';

export class QuaternionPdController {
  constructor(params) {
    this.Kp = params.Kp;   // 每軸 [Kp_x, Kp_y, Kp_z]
    this.Kd = params.Kd;   // 每軸 [Kd_x, Kd_y, Kd_z]
    this.reset();
  }
  reset() { this.cmd = [0, 0, 0]; }
  step(dt, qEst, wEst, qTarget) {
    const qe = Q.mul(Q.conjugate(qEst), qTarget); // q⁻¹ ⊗ q_d
    const s = qe[0] >= 0 ? 1 : -1;               // 處理雙重覆蓋
    const tau = [0, 0, 0];
    for (let a = 0; a < 3; a++) {
      tau[a] = 2 * this.Kp[a] * s * qe[a + 1] - this.Kd[a] * wEst[a];
    }
    this.cmd = tau;
    return tau;
  }
}
