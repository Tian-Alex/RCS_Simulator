// =====================================================================
// rcs.js — 冷氣 RCS 推力器模型
//   ・幾何：安裝位置 p_B（本體座標，m）、推力方向 d_B（單位向量，
//     「作用於箭體的力」方向；排氣方向為 −d_B）— docs §3
//   ・閥門狀態機：CLOSED / PENDING_OPEN / OPEN / PENDING_CLOSE，
//     含開/關延遲、最小開啟時間（最小衝量）、最大占空比
//   ・推進劑：ṁ = −F/(Isp·g0)（docs §3.3）
// =====================================================================
import { V, clamp } from './mathUtils.js';
import { G0 } from './dynamics.js';

// 由參數建立推力器（d_B 會正規化）
export function makeThruster(p) {
  const d = V.norm([p.dir[0], p.dir[1], p.dir[2]]) || 1;
  return {
    id: p.id,
    name: p.name,
    pos: [p.pos[0], p.pos[1], p.pos[2]],      // m，本體座標（原點 = 參考點）
    dir: [p.dir[0] / d, p.dir[1] / d, p.dir[2] / d], // 單位向量
    maxThrust: p.maxThrust,                     // N
    minImpulse: p.minImpulse,                  // N·s
    delayOpen: p.delayOpen,                     // s
    delayClose: p.delayClose,                  // s
    maxDuty: p.maxDuty,                        // 0–1（滑動視窗）
    dutyWindow: p.dutyWindow ?? 10,            // s
    isp: p.isp,                                // s
    enabled: p.enabled !== false,
  };
}

// 最小開啟時間 t_min = I_min / F_max（docs §3.2）
export function minOnTime(t) {
  return t.maxThrust > 0 ? t.minImpulse / t.maxThrust : 0;
}

// ------------------- 閥門狀態機 -------------------
// cmd: true = 要求開；false = 要求關
// 回傳實際開關狀態。狀態轉換圖見 docs §3.2 圖 3-1
export class Valve {
  constructor(t) {
    this.t = t;
    this.state = 'CLOSED';        // CLOSED | PENDING_OPEN | OPEN | PENDING_CLOSE
    this.timer = 0;               // 延遞倒數
    this.openedAt = -1;           // 開啟時刻（-1 = 未開）
    this.lastCmd = false;
    // 占空比紀錄：[t_on, t_off] 事件對（滑動視窗）
    this.window = [];
    this.cycleCount = 0;
    this.totalOnTime = 0;
    this.onHistory = [];          // 記錄通道 [t, 1/0]
  }
  duty(t) {
    // 最近 dutyWindow 秒內的開啟時間比例
    this.window = this.window.filter((iv) => iv[1] > t - this.t.dutyWindow);
    let acc = 0;
    const t0 = t - this.t.dutyWindow;
    for (const [a, b] of this.window) acc += Math.min(b, t) - Math.max(a, t0);
    return acc / this.t.dutyWindow;
  }
  step(dt, t, cmd) {
    this.lastCmd = cmd;
    let out = this.state === 'OPEN' ? 1 : 0;
    switch (this.state) {
      case 'CLOSED':
        if (cmd) {
          // 占空比檢查：超過上限則禁止新的點火
          if (this.duty(t) >= this.t.maxDuty - 1e-9 && this.t.maxDuty < 1) {
            out = 0;
            break;
          }
          this.state = 'PENDING_OPEN';
          this.timer = this.t.delayOpen;
        }
        break;
      case 'PENDING_OPEN':
        this.timer -= dt;
        if (!cmd) { this.state = 'CLOSED'; break; }
        if (this.timer <= 0) {
          this.state = 'OPEN';
          this.openedAt = t;
          this.cycleCount++;
          out = 1;
        }
        break;
      case 'OPEN':
        out = 1;
        if (!cmd) {
          // 最小開啟時間檢查：未達 t_min 則保持開啟（docs §3.2）
          const elapsed = t - this.openedAt;
          if (elapsed >= minOnTime(this.t) - 1e-9) {
            // 推力終止：累計點火時間
            this.totalOnTime += t - this.openedAt;
            this.state = 'PENDING_CLOSE';
            this.timer = this.t.delayClose;
            out = 0; // 進入關閉程序即視為關閉輸出（推力終止由 PENDING_CLOSE 起算）
          }
        }
        break;
      case 'PENDING_CLOSE':
        this.timer -= dt;
        out = 0;
        if (cmd) {
          // 在關閉途中又要求開啟 → 直接回到 OPEN（無需再等開延遲）
          this.state = 'OPEN';
          this.openedAt = t; // 新的點火段（前一段已在 OPEN→PENDING_CLOSE 累計）；
          out = 1;            // openedAt === t 使下方紀錄區自動補上 window/onHistory
        } else if (this.timer <= 0) {
          this.closeAt(t);
        }
        break;
    }
    if (out === 1 && this.state === 'OPEN' && this.openedAt === t) {
      this.window.push([t, t]);
      this.onHistory.push([t, 1]);
    } else if (out === 0 && this.onHistory.length && this.onHistory[this.onHistory.length - 1][1] === 1 && this.state !== 'OPEN') {
      // 記錄關閉
      const iv = this.window[this.window.length - 1];
      if (iv) iv[1] = t;
      this.onHistory.push([t, 0]);
    }
    return out;
  }
  closeAt(t) {
    const iv = this.window[this.window.length - 1];
    if (iv && iv[1] === iv[0]) iv[1] = t;
    if (this.state === 'OPEN') { this.totalOnTime += t - this.openedAt; }
    this.state = 'CLOSED';
    this.openedAt = -1;
    if (this.onHistory.length && this.onHistory[this.onHistory.length - 1][1] === 1)
      this.onHistory.push([t, 0]);
  }
}

// RCS 群組：管理全部推力器閥門 + 產生合力/合力矩
export class RcsSystem {
  constructor(thrusters, cg) {
    this.thrusters = thrusters.map(makeThruster);
    this.cg = cg.slice();          // 質心位置（本體座標，m）
    this.valves = this.thrusters.map((t) => new Valve(t));
    this.forceScale = this.thrusters.map(() => 1); // 理想連續模式的推力比例
  }
  setContinuousScale(scales) {
    this.forceScale = scales;
  }
  // 單位推力（1 N）的力與力矩貢獻
  wrenchOf(i) {
    const t = this.thrusters[i];
    const r = V.sub(t.pos, this.cg);
    return { f: t.dir.slice(), tau: V.cross(r, t.dir) };
  }
  // 目前各推力器輸出推力 [N]
  thrusts() {
    return this.thrusters.map((t, i) => {
      if (!t.enabled) return 0;
      const on = this.valves[i].state === 'OPEN' ? 1 : 0;
      return on * t.maxThrust * this.forceScale[i];
    });
  }
  // 合力（body）、合力矩（body）、質量流率
  wrench() {
    let F = [0, 0, 0], T = [0, 0, 0], mdot = 0;
    for (let i = 0; i < this.thrusters.length; i++) {
      const t = this.thrusters[i];
      if (!t.enabled) continue;
      const on = this.valves[i].state === 'OPEN' ? 1 : 0;
      const f = on * t.maxThrust * this.forceScale[i];
      if (f === 0) continue;
      const r = V.sub(t.pos, this.cg);
      const F_i = V.scale(t.dir, f);
      F = V.add(F, F_i);
      T = V.add(T, V.cross(r, F_i));
      mdot -= f / (t.isp * G0);   // docs §3.3
    }
    return { F, T, mdot };
  }
}
