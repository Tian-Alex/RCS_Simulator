// =====================================================================
// modulation.js — PWPF / PWM 調變器
// 連續力矩指令 → 開關指令（docs §5）
//   PWPF：低通濾波 + 遲滯繼電器（Schmitt trigger），
//         參數 Km, Tm, U_on, U_off（Sci Rep 2023 Fig.7 標準結構）
//   PWM ：固定週期、變寬度（duty = τ_cmd/τ̂）
//   理想連續模式（不經調變，推力可連續調節）→ 用於驗證控制器本身
// =====================================================================
import { clamp } from './mathUtils.js';

// ------------------- 單軸 PWPF -------------------
export class Pwpf {
  constructor(params) {
    this.Km = params.Km;        // 前置濾波增益
    this.Tm = params.Tm;        // 濾波時間常數 [s]
    this.Uon = params.Uon;    // 開啟門檻（正規化）
    this.Uoff = params.Uoff;  // 關閉門檻（遲滯帶，Uoff < Uon）
    this.reset();
  }
  reset() {
    this.uf = 0;   // 濾波後訊號
    this.state = 0; // -1 | 0 | +1
  }
  // u：正規化輸入（τ_cmd/τ̂，−1..1）；dt：步長
  step(u, dt) {
    // 一階低通濾波（標準 PWPF 拓撲含輸出回授）：
    //   濾波器輸入 = u − s（s 為目前輸出狀態）→ u̇_f = (Km(u−s) − u_f)/Tm
    //   點火時 s=±1 使 u_f 反向衰減 → 關閉；關閉時 u_f 朝指令回升 → 再點火，
    //   形成脈寬（濾波時間常數決定）與脈頻（遲滯帶決定）之自然調變
    this.uf += ((this.Km * (u - this.state) - this.uf) / this.Tm) * dt;
    // 遲滯繼電器（對稱三態）
    if (this.uf >= this.Uon) this.state = 1;
    else if (this.uf <= -this.Uon) this.state = -1;
    else if (Math.abs(this.uf) <= this.Uoff) this.state = 0;
    return this.state;
  }
}

// 三軸 PWPF/PWM 調變群（每軸獨立調變器）
export class Modulator {
  constructor(params) {
    this.mode = params.mode; // 'pwpf' | 'pwm' | 'continuous'
    this.pwmPeriod = params.pwmPeriod ?? 0.5; // s
    this.pwpf = [0, 1, 2].map(() => new Pwpf(params.pwpf ?? { Km: 1.0, Tm: 0.2, Uon: 0.12, Uoff: 0.06 }));
    this.tauCap = [1, 1, 1];   // 軸向滿刻度力矩（由 allocation 提供）
    this.minPulse = params.minPulse ?? 0; // 最小脈衝寬度 [s]（低於此採殘差追蹤）
    this.pwmPhase = 0;
    this.reset();
  }
  reset() {
    this.pwpf.forEach((m) => m.reset());
    this.pwmPhase = 0;
    this.pwmResidual = [0, 0, 0]; // PWM 殘差追蹤累積器 [s]
    this.pulseRemain = [0, 0, 0]; // 跳脫脈衝剩餘點火時間 [s]
  }
  setCapacity(tauCap) { this.tauCap = tauCap.slice(); }

  // 輸入：τ_cmd（body）；輸出：調變後力矩請求 τ_m（body）
  // 呼叫頻率 = 物理步長
  step(tauCmd, dt) {
    if (this.mode === 'continuous') return tauCmd.slice();
    const u = tauCmd.map((t, a) => clamp(t / this.tauCap[a], -1.5, 1.5));
    let states;
    if (this.mode === 'pwpf') {
      states = [0, 1, 2].map((a) => this.pwpf[a].step(u[a], dt));
    } else {
      // PWM：每軸獨立、同相位
      this.pwmPhase = (this.pwmPhase + dt) % this.pwmPeriod;
      states = u.map((x, a) => {
        const duty = clamp(Math.abs(x), 0, 1);
        if (duty < 1e-9) { this.pwmResidual[a] = 0; return 0; }
        // 殘差追蹤：當需求 on-time（duty·T_pwm [s]）低於最小脈衝寬度時，
        // 以時間為單位累積需求（duty·dt，步長無關），累積至 ≥ minPulse 才點火
        // （pulse skipping），避免直接捨去造成死區。跳脫脈衝以倒數計時器
        // 交付「完整累積需求」（不受 PWM 相位窗口影響、步長無關）。
        if (this.pulseRemain[a] > 0) {
          this.pulseRemain[a] -= dt;
          return Math.sign(x);
        }
        if (this.minPulse > 0 && duty * this.pwmPeriod < this.minPulse) {
          this.pwmResidual[a] += duty * dt;
          if (this.pwmResidual[a] < this.minPulse) return 0;
          this.pulseRemain[a] = this.pwmResidual[a]; // 交付完整累積需求
          this.pwmResidual[a] = 0;
          return Math.sign(x);
        }
        return this.pwmPhase < duty * this.pwmPeriod ? Math.sign(x) : 0;
      });
    }
    return states.map((s, a) => s * this.tauCap[a]);
  }
}
