// =====================================================================
// ui.js — 參數編輯器（宣告式面板）＋驗證＋案例匯入匯出＋指標顯示
// 參數變更於「未運行」時立即重建模擬；運行中鎖定輸入
// =====================================================================
import { M3 } from './mathUtils.js';
import { PRESETS, CONTROLLERS, MODULATORS } from './parameters.js';
import { Simulation } from './simulation.js';

const DEG = Math.PI / 180;

// ------------------- 宣告式面板結構 -------------------
const PANELS = [
  {
    id: 'sim', title: '模擬設定',
    fields: [
      { path: 'sim.duration', label: '模擬時長 [s]', tip: '模擬總時長。模擬會計算到此時間後停止。', step: 1 },
      { path: 'sim.dt', label: '物理步長 [s]', tip: '物理積分步長（RK4）。越小越精確但越慢；建議 ≤ 控制週期的 1/10。', step: 0.001 },
      { path: 'sim.controlPeriod', label: '控制週期 [s]', tip: '控制器與調變器的取樣週期。閥門指令每此週期更新一次。', step: 0.01 },
      { path: 'sim.recordInterval', label: '記錄間隔 [s]', tip: '圖表資料的取樣間隔。越小圖越細緻，資料量越大。', step: 0.01 },
      { path: 'sim.settleBandDeg', label: '穩定判據帶寬 [°]', tip: '穩定判據帶寬：姿態誤差進入此角度內且不再離開，即視為「已穩定」。', step: 0.1 },
    ],
  },
  {
    id: 'vehicle', title: '載具與慣量',
    fields: [
      { path: 'vehicle.dryMass', label: '乾質量 [kg]', tip: '不含推進劑的箭體質量 [kg]。', step: 0.1 },
      { path: 'vehicle.mProp', label: '推進劑 [kg]', tip: '冷氣推進劑（如氮氣）總質量 [kg]，點火時消耗。', step: 0.1 },
      { path: 'vehicle.length', label: '箭體長度 [m]', tip: '箭體總長 [m]，縱軸為 X（鼻錐方向）。', step: 0.1 },
      { path: 'vehicle.radius', label: '箭體半徑 [m]', tip: '箭體半徑 [m]。', step: 0.05 },
      { path: 'vehicle.cg', label: '質心位置 [m]', tip: '質心在本體座標的位置 [m]。推力器力矩臂 = 安裝位置 − 質心。', vec: 3, step: 0.01 },
      { path: 'vehicle.Ixx', label: 'Ixx [kg·m²]', tip: '縱軸（滾轉軸）慣量 [kg·m²]。', step: 1 },
      { path: 'vehicle.Iyy', label: 'Iyy [kg·m²]', tip: '俯仰軸慣量 [kg·m²]。', step: 1 },
      { path: 'vehicle.Izz', label: 'Izz [kg·m²]', tip: '偏航軸慣量 [kg·m²]。', step: 1 },
      { path: 'vehicle.Ixy', label: 'Ixy [kg·m²]', tip: '慣性積 P_xy = ∫xy dm [kg·m²]，張量自動取負號。設 0 表示軸對稱。', step: 1 },
      { path: 'vehicle.Ixz', label: 'Ixz [kg·m²]', tip: '慣性積 P_xz = ∫xz dm [kg·m²]。', step: 1 },
      { path: 'vehicle.Iyz', label: 'Iyz [kg·m²]', tip: '慣性積 P_yz = ∫yz dm [kg·m²]。', step: 1 },
      { note: '輸入慣性積 P = ∫xy dm（正號），張量非對角項自動取 −P，見 docs/equations.md §1' },
    ],
  },
  {
    id: 'initial', title: '初始條件',
    fields: [
      { path: 'initial.attitude', label: '初始姿態 φ/θ/ψ [°]', tip: '初始姿態（roll/pitch/yaw，3-2-1 Euler 角）[°]。', vec: 3, step: 1 },
      { path: 'initial.rates', label: '初始角速度 [°/s]', tip: '初始角速度 [°/s]。設定非零可模擬翻滾中的箭體。', vec: 3, step: 0.1 },
      { path: 'initial.position', label: '初始位置 [m]', tip: '質心初始位置 [m]（慣性系）。', vec: 3, step: 1 },
      { path: 'initial.velocity', label: '初始速度 [m/s]', tip: '質心初始速度 [m/s]（慣性系）。', vec: 3, step: 0.01 },
    ],
  },
  {
    id: 'target', title: '目標姿態',
    fields: [
      { path: 'target.attitude', label: '目標 φ/θ/ψ [°]', tip: '希望指向的目標姿態（roll/pitch/yaw）[°]。', vec: 3, step: 1 },
    ],
  },
  {
    id: 'controller', title: '控制器',
    fields: [
      { path: 'controller.type', label: '控制律', tip: '控制律：PID（含積分抗擾）、LQR（最佳回授）、Quaternion PD（大角度機動）。', select: CONTROLLERS },
      { path: 'controller.params.gains', label: 'PID 增益 Kp/Kd/Ki（每軸）', tip: '每軸 [Kp, Kd, Ki]：Kp 越大轉得越快、Kd 越大阻尼越強、Ki 抑制穩態誤差。', matrix: [3, 3], showIf: 'pid', step: 0.1 },
      { path: 'controller.params.intLimit', label: 'PID 積分上限 [N·m·s]', tip: '積分項飽和上限，防止積分上捲（windup）。', vec: 3, showIf: 'pid', step: 0.5 },
      { path: 'controller.params.intWindow', label: 'PID 積分視窗 [rad]', tip: '誤差小於此值才積分，避免大角度機動時上捲。', scalar: true, showIf: 'pid', step: 0.01 },
      { path: 'controller.params.Qdiag', label: 'LQR Q（θx θy θz ωx ωy ωz）', tip: 'LQR 狀態權重 Q = [θx θy θz ωx ωy ωz]：越大跟蹤越積極。', vec: 6, showIf: 'lqr', step: 10 },
      { path: 'controller.params.Rdiag', label: 'LQR R（每軸）', tip: 'LQR 控制能量權重 R：越大越省推進劑但響應慢。', vec: 3, showIf: 'lqr', step: 0.1 },
      { path: 'controller.params.Kp', label: 'QPD Kp（每軸）', tip: 'Quaternion PD 位置增益：越大轉得越快。', vec: 3, showIf: 'qpd', step: 1 },
      { path: 'controller.params.Kd', label: 'QPD Kd（每軸）', tip: 'Quaternion PD 阻尼增益：越大越不震盪。', vec: 3, showIf: 'qpd', step: 1 },
    ],
  },
  {
    id: 'modulator', title: '調變器',
    fields: [
      { path: 'modulator.mode', label: '調變模式', tip: 'PWPF/PWM：將連續力矩指令轉為閥門開關；連續＝理想推力（對照組）。', select: MODULATORS },
      { path: 'modulator.pwmPeriod', label: 'PWM 週期 [s]', tip: 'PWM 週期。必須遠大於閥門開關延遲總和。', scalar: true, step: 0.05 },
      { path: 'modulator.pwpf.Km', label: 'PWPF 濾波增益 Km', tip: 'PWPF 濾波器回授增益，影響脈衝頻率。', scalar: true, step: 0.1 },
      { path: 'modulator.pwpf.Tm', label: 'PWPF 時間常數 Tm [s]', tip: 'PWPF 濾波時間常數 [s]。', scalar: true, step: 0.01 },
      { path: 'modulator.pwpf.Uon', label: 'PWPF 開啟門檻 Uon', tip: '開啟門檻（軸容量百分比）。越大越不靈敏。', scalar: true, step: 0.005 },
      { path: 'modulator.pwpf.Uoff', label: 'PWPF 關閉門檻 Uoff', tip: '關閉門檻（需小於 Uon，形成遲滯）。', scalar: true, step: 0.005 },
    ],
  },
  {
    id: 'allocation', title: '推力分配',
    fields: [
      { path: 'allocation.wForce', label: '淨力懲罰權重 wF', tip: '分配器對淨推力的懲罰：越大越接近純力偶（不產生平移）。', scalar: true, step: 0.01 },
      { note: '分配器求解：min ‖Tf−τ‖² + wF‖Ff‖²（0 ≤ f ≤ Fmax），見 docs/equations.md §4' },
    ],
  },
  {
    id: 'disturbances', title: '擾動',
    fields: [
      { path: 'disturbances.constantTorque', label: '常值力矩 [N·m]', tip: '持續作用的外部力矩 [N·m]，用來測試控制器抗擾能力。', vec: 3, step: 0.01 },
      { path: 'disturbances.randomTorque.sigma', label: '隨機力矩 σ [N·m/√Hz]', tip: '隨機力矩的標準差（每軸）。', vec: 3, step: 0.005 },
      { path: 'disturbances.randomTorque.interval', label: '隨機取樣間隔 [s]', tip: '隨機力矩的更新間隔。', scalar: true, step: 0.1 },
      { path: 'disturbances.randomTorque.seed', label: '隨機種子', tip: '亂數種子：同值重現相同擾動序列。', scalar: true, step: 1 },
      { path: 'disturbances.gravityGradient.enabled', label: '重力梯度', tip: '重力梯度力矩（需設定軌道高度）。', bool: true },
      { path: 'disturbances.gravityGradient.altitude', label: '軌道高度 [m]', tip: '圓軌道高度 [m]（自海平面起算）。', scalar: true, step: 10000 },
      { path: 'disturbances.gravityGradient.phase0', label: '初始軌道相位 [rad]', tip: '初始軌道相位 [rad]。', scalar: true, step: 0.1 },
      { path: 'disturbances.solarPressure.enabled', label: '太陽輻射壓', tip: '太陽輻射壓力矩。', bool: true },
      { path: 'disturbances.solarPressure.sunDirection', label: '太陽方向（慣性系）', tip: '太陽方向單位向量（慣性系）。', vec: 3, step: 0.1 },
      { path: 'disturbances.solarPressure.Cr', label: '反射係數 Cr', tip: '表面反射係數（1.0 全吸收～2.0 全反射）。', scalar: true, step: 0.05 },
      { path: 'disturbances.solarPressure.area', label: '有效面積 [m²]', tip: '受照有效面積 [m²]。', scalar: true, step: 0.1 },
      { path: 'disturbances.solarPressure.cpOffset', label: '壓力中心偏移 [m]', tip: '壓力中心相對質心的偏移 [m]。', vec: 3, step: 0.01 },
    ],
  },
  {
    id: 'sensors', title: '感測器',
    fields: [
      { path: 'sensors.enabled', label: '啟用感測誤差', tip: '開啟後控制器使用含誤差的估計值（更接近真實）。', bool: true },
      { path: 'sensors.gyroNoise', label: '陀螺儀雜訊 [rad/s/√Hz]', tip: '陀螺儀速率雜訊密度 [rad/s/√Hz]。', vec: 3, step: 0.0002 },
      { path: 'sensors.gyroBias', label: '陀螺儀偏置 [rad/s]', tip: '陀螺儀固定偏置 [rad/s]。', vec: 3, step: 0.0001 },
      { path: 'sensors.attNoise', label: '姿態量測雜訊 [rad]', tip: '姿態量測角度雜訊 [rad]。', vec: 3, step: 0.0001 },
      { path: 'sensors.interval', label: '取樣間隔 [s]', tip: '感測器取樣間隔 [s]。', scalar: true, step: 0.01 },
    ],
  },
];

// ------------------- 深層存取工具 -------------------
function get(obj, path) {
  return path.split('.').reduce((o, k) => o[k], obj);
}
function set(obj, path, v) {
  const ks = path.split('.');
  let o = obj;
  ks.slice(0, -1).forEach((k) => (o = o[k]));
  o[ks[ks.length - 1]] = v;
}

export class UI {
  constructor(cfg, callbacks) {
    this.cfg = cfg;
    this.cb = callbacks; // { onRebuild(), onPreset(name) }
    this.container = document.getElementById('param-panels');
    this.msg = document.getElementById('validate-msg');
    this.build();
  }

  setLocked(locked) {
    this.container.querySelectorAll('input,select').forEach((el) => (el.disabled = locked));
  }

  build() {
    this.container.innerHTML = '';
    this.inputs = [];
    this.conditional = [];
    PANELS.forEach((p) => {
      const panel = document.createElement('div');
      panel.className = 'panel';
      panel.innerHTML = `
        <div class="panel-head" role="button" tabindex="0"><span>${p.title}</span><span class="chev">▾</span></div>
        <div class="panel-body"></div>`;
      const body = panel.querySelector('.panel-body');
      panel.querySelector('.panel-head').addEventListener('click', () => panel.classList.toggle('collapsed'));
      p.fields.forEach((f) => this.buildField(body, f));
      this.container.appendChild(panel);
      this.inputs.push({ panel, id: p.id });
    });
    this.buildThrusterTable();
    this.refreshVisibility();
  }

  buildField(body, f) {
    // 僅建立「符合當前控制器型別」的條件欄位（切換型別時會重建面板）
    if (f.showIf && f.showIf !== this.cfg.controller.type) return;
    if (f.note) {
      const n = document.createElement('p');
      n.className = 'axis-note';
      n.textContent = f.note;
      body.appendChild(n);
      return;
    }
    if (f.select) {
      const wrap = document.createElement('div');
      wrap.className = 'field';
      wrap.innerHTML = `<label${f.tip ? ` data-tip="${f.tip}"` : ''}>${f.label}</label>`;
      const sel = document.createElement('select');
      f.select.forEach((o) => {
        const opt = document.createElement('option');
        opt.value = o.id; opt.textContent = o.label;
        sel.appendChild(opt);
      });
      sel.value = get(this.cfg, f.path);
      sel.dataset.path = f.path;
      sel.addEventListener('change', () => this.onChange(f, sel.value));
      wrap.appendChild(sel);
      body.appendChild(wrap);
      this.conditional.push({ el: wrap, showIf: f.showIf });
      return;
    }
    if (f.bool) {
      const wrap = document.createElement('div');
      wrap.className = 'field';
      wrap.style.flexDirection = 'row';
      wrap.style.alignItems = 'center';
      wrap.style.gap = '6px';
      const cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.checked = get(this.cfg, f.path);
      cb.dataset.path = f.path;
      cb.addEventListener('change', () => this.onChange(f, cb.checked));
      wrap.appendChild(cb);
      const lab = document.createElement('label');
      if (f.tip) { lab.dataset.tip = f.tip; lab.style.cursor = 'help'; lab.style.borderBottom = '1px dotted var(--text-faint)'; }
      lab.textContent = f.label;
      wrap.appendChild(lab);
      body.appendChild(wrap);
      return;
    }
    const val = get(this.cfg, f.path);
    const n = f.vec ? f.vec : 1;
    const wrap = document.createElement('div');
    wrap.className = 'field';
    wrap.innerHTML = `<label${f.tip ? ` data-tip="${f.tip}"` : ''}>${f.label}</label>`;
    if (f.matrix) {
      // 矩陣欄位（如 PID 增益 [3×3]）：每軸一列
      const rows = document.createElement('div');
      for (let a = 0; a < f.matrix[0]; a++) {
        const row = document.createElement('div');
        row.className = 'grid3';
        row.style.marginBottom = '3px';
        for (let b = 0; b < f.matrix[1]; b++) {
          const inp = document.createElement('input');
          inp.type = 'number';
          inp.step = f.step ?? 0.1;
          inp.value = +val[a][b].toFixed(4);
          inp.dataset.path = f.path;
          inp.dataset.mrow = a;
          inp.dataset.mcol = b;
          inp.addEventListener('change', () => {
            const m = get(this.cfg, f.path);
            m[a][b] = parseFloat(inp.value);
            this.rebuild();
          });
          row.appendChild(inp);
        }
        rows.appendChild(row);
      }
      wrap.appendChild(rows);
      body.appendChild(wrap);
      return;
    }
    const row = document.createElement('div');
    row.className = n === 3 ? 'grid3' : 'grid2';
    for (let i = 0; i < n; i++) {
      const inp = document.createElement('input');
      inp.type = 'number';
      inp.step = f.step ?? 0.1;
      const v = n === 1 ? val : val[i];
      inp.value = typeof v === 'number' ? +v.toFixed(6) : v;
      inp.dataset.path = f.path;
      inp.dataset.idx = n === 1 ? -1 : i;
      inp.addEventListener('change', () => this.onChange(f, parseFloat(inp.value), inp.dataset.idx));
      row.appendChild(inp);
    }
    wrap.appendChild(row);
    body.appendChild(wrap);
    if (f.showIf) this.conditional.push({ el: wrap, showIf: f.showIf });
  }

  // 推力器卡片（可新增／刪除；位置提供「站位＋時鐘角」與 xyz 兩種輸入，雙向同步）
  buildThrusterTable() {
    const panel = document.createElement('div');
    panel.className = 'panel';
    panel.innerHTML = `
      <div class="panel-head" role="button" tabindex="0"><span>RCS 推力器（${this.cfg.rcs.thrusters.length} 具，可點選 3D 視圖同步）</span><span class="chev">▾</span></div>
      <div class="panel-body"><div class="thruster-cards"></div></div>`;
    panel.querySelector('.panel-head').addEventListener('click', () => panel.classList.toggle('collapsed'));
    const list = panel.querySelector('.thruster-cards');
    // 功能性分組高亮（依名稱前綴 R/P/Y 分類）
    const groupBar = document.createElement('div');
    groupBar.className = 'sub';
    groupBar.style.cssText = 'display:flex;gap:6px;flex-wrap:wrap;margin:0 0 8px;align-items:center';
    groupBar.innerHTML = `
      <span style="font-size:11px;color:var(--text-faint)">軸向分組高亮：</span>
      <button class="btn btn-small grp-btn" data-grp="R">Roll 組</button>
      <button class="btn btn-small grp-btn" data-grp="P">Pitch 組</button>
      <button class="btn btn-small grp-btn" data-grp="Y">Yaw 組</button>
      <button class="btn btn-small grp-btn" data-grp="">清除</button>`;
    groupBar.querySelectorAll('.grp-btn').forEach((b) =>
      b.addEventListener('click', () => this.setGroupHighlight(b.dataset.grp)));
    const body = panel.querySelector('.panel-body');
    body.insertBefore(groupBar, list);
    const DIRS = {
      '+X': [1, 0, 0], '−X': [-1, 0, 0], '+Y': [0, 1, 0], '−Y': [0, -1, 0], '+Z': [0, 0, 1], '−Z': [0, 0, -1],
    };
    const L = this.cfg.vehicle.length;
    this.cfg.rcs.thrusters.forEach((t, ti) => {
      const card = document.createElement('div');
      card.className = 'thruster-card' + (t.enabled ? '' : ' disabled');
      card.dataset.ti = ti;
      // 時鐘角：0° = +Y 軸，逆時針（朝 +Z）
      const clock = Math.atan2(t.pos[2], t.pos[1]) * 180 / Math.PI;
      const radius = Math.hypot(t.pos[1], t.pos[2]);
      card.innerHTML = `
        <div class="thruster-card-head">
          <span class="tname"><input data-f="name" value="${t.name}"></span>
          <label style="font-size:11px;color:var(--text-muted);display:flex;align-items:center;gap:4px">
            <input type="checkbox" data-f="enabled" ${t.enabled ? 'checked' : ''}>啟用</label>
          <button class="thruster-del" data-f="del" title="刪除此推力器">✕</button>
        </div>
        <div class="sub">
          <label>站位 X</label><input type="number" step="0.05" data-f="station" value="${t.pos[0].toFixed(2)}">
          <label>時鐘角°</label><input type="number" step="15" data-f="clock" value="${clock.toFixed(0)}">
          <label>半徑</label><input type="number" step="0.05" min="0" data-f="rad" value="${radius.toFixed(2)}">
          <label>px</label><input type="number" step="0.05" data-f="px" value="${t.pos[0].toFixed(2)}">
          <label>py</label><input type="number" step="0.05" data-f="py" value="${t.pos[1].toFixed(2)}">
          <label>pz</label><input type="number" step="0.05" data-f="pz" value="${t.pos[2].toFixed(2)}">
        </div>
        <div class="station-note">時鐘角 0° = +Y、90° = +Z；站位沿縱軸（+X = 鼻錐方向），範圍約 ±${(L / 2).toFixed(1)} m</div>
        <div class="sub" style="margin-top:4px">
          <label>推力方向</label>
          ${Object.entries(DIRS).map(([k, d]) => `<button class="dirbtn${t.dir.every((c, i) => Math.abs(c - d[i]) < 1e-6) ? ' active' : ''}" data-dir="${k}">${k}</button>`).join('')}
          <label>dx</label><input type="number" step="0.1" data-f="dx" value="${t.dir[0].toFixed(2)}">
          <label>dy</label><input type="number" step="0.1" data-f="dy" value="${t.dir[1].toFixed(2)}">
          <label>dz</label><input type="number" step="0.1" data-f="dz" value="${t.dir[2].toFixed(2)}">
        </div>
        <div class="sub" style="margin-top:4px">
          <label>F[N]</label><input type="number" step="0.5" min="0" data-f="maxThrust" value="${t.maxThrust}">
          <label>Imin[N·s]</label><input type="number" step="0.005" min="0" data-f="minImpulse" value="${t.minImpulse}">
          <label>t_on[s]</label><input type="number" step="0.005" min="0" data-f="delayOpen" value="${t.delayOpen}">
          <label>t_off[s]</label><input type="number" step="0.005" min="0" data-f="delayClose" value="${t.delayClose}">
          <label>占空比</label><input type="number" step="0.05" min="0" max="1" data-f="maxDuty" value="${t.maxDuty}">
          <label>Isp[s]</label><input type="number" step="1" min="1" data-f="isp" value="${t.isp}">
        </div>`;
      list.appendChild(card);

      // 卡片點擊 → 選取（同步 3D）
      card.addEventListener('click', (e) => {
        if (e.target.closest('input,button')) return;
        this.selectThruster(ti);
      });
      card.querySelectorAll('input,button').forEach((inp) => {
        const f = inp.dataset.f || inp.dataset.dir;
        if (!f) return;
        if (inp.tagName === 'BUTTON') inp.addEventListener('click', () => this.editThruster(ti, f, inp));
        else inp.addEventListener('change', () => this.editThruster(ti, f, inp));
      });
    });
    // 新增按鈕
    const addBtn = document.createElement('button');
    addBtn.className = 'thruster-add';
    addBtn.textContent = '＋ 新增推力器';
    addBtn.addEventListener('click', () => {
      const R = this.cfg.vehicle.radius;
      this.cfg.rcs.thrusters.push({
        name: `T${this.cfg.rcs.thrusters.length + 1}`,
        pos: [L * 0.45, R, 0],
        dir: [0, 0, -1],
        maxThrust: 10, minImpulse: 0.05,
        delayOpen: 0.015, delayClose: 0.01,
        maxDuty: 0.9, dutyWindow: 10, isp: 65, enabled: true,
      });
      this.rebuild();
      this.build();
    });
    list.appendChild(addBtn);
    this.container.appendChild(panel);
    this.refreshThrusterSelection();
  }

  // 編輯推力器欄位（站位/時鐘角與 xyz 雙向換算）
  editThruster(ti, f, inp) {
    const t = this.cfg.rcs.thrusters[ti];
    const num = (x) => { const v = parseFloat(x); return isFinite(v) ? v : 0; };
    if (f === 'name') t.name = inp.value;
    else if (f === 'enabled') t.enabled = inp.checked;
    else if (f === 'del') {
      if (this.cfg.rcs.thrusters.length <= 1) return; // 至少保留一具
      this.cfg.rcs.thrusters.splice(ti, 1);
      this.rebuild(); this.build();
      return;
    }
    else if (f === 'station' || f === 'clock' || f === 'rad') {
      // 站位/時鐘角/半徑 → xyz
      const clockRad = num(this.cardInput(ti, 'clock')?.value) * Math.PI / 180;
      const r = Math.max(0, num(this.cardInput(ti, 'rad')?.value));
      t.pos = [num(this.cardInput(ti, 'station')?.value), r * Math.cos(clockRad), r * Math.sin(clockRad)];
      this.rebuild(); this.build();
      return;
    }
    else if (f === 'px' || f === 'py' || f === 'pz') {
      const i = { px: 0, py: 1, pz: 2 }[f];
      t.pos[i] = num(inp.value);
      this.rebuild(); this.build();
      return;
    }
    else if (f === 'dx' || f === 'dy' || f === 'dz') {
      const i = { dx: 0, dy: 1, dz: 2 }[f];
      t.dir[i] = num(inp.value);
      const n = Math.hypot(...t.dir);
      if (n > 1e-9) t.dir = t.dir.map((c) => c / n); // 正規化
      this.rebuild(); this.build();
      return;
    }
    else if (inp.dataset.dir) {
      // 方向快選按鈕
      const m = { '+X': [1, 0, 0], '−X': [-1, 0, 0], '+Y': [0, 1, 0], '−Y': [0, -1, 0], '+Z': [0, 0, 1], '−Z': [0, 0, -1] }[f];
      t.dir = m.slice();
      this.rebuild(); this.build();
      return;
    }
    else t[f] = num(inp.value);
    this.rebuild();
  }

  // 由 3D 點選時僅更新卡片高亮（不回呼，避免循環）
  setThrusterSelection(ti) {
    this.selectedThruster = ti;
    this.groupHighlight = null;
    this.refreshThrusterSelection();
    // 捲動至對應卡片
    const card = this.container.querySelector(`.thruster-card[data-ti="${ti}"]`);
    if (card) card.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
  }

  // 功能性分組高亮（R/P/Y 前綴 → 卡片＋3D 同步）
  setGroupHighlight(prefix) {
    if (!prefix) {
      this.groupHighlight = null;
      this.selectedThruster = null; // 清除時一併取消單選
      this.refreshThrusterSelection();
      if (this.cb.onGroupHighlight) this.cb.onGroupHighlight(null);
      return;
    }
    this.groupHighlight = this.cfg.rcs.thrusters
      .map((t, i) => (t.name.trim().toUpperCase().startsWith(prefix) ? i : -1))
      .filter((i) => i >= 0);
    this.selectedThruster = null; // 分組高亮時清除單選
    this.refreshThrusterSelection();
    if (this.cb.onGroupHighlight) this.cb.onGroupHighlight(this.groupHighlight);
  }

  // 目前選取的推力器（卡片點選 → 同步 3D）
  selectThruster(ti) {
    this.selectedThruster = ti;
    this.groupHighlight = null; // 單選時清除分組高亮
    this.refreshThrusterSelection();
    if (this.cb.onSelectThruster) this.cb.onSelectThruster(ti);
  }
  refreshThrusterSelection() {
    const inGroup = (ti) => this.groupHighlight?.includes(ti) ?? false;
    this.container.querySelectorAll('.thruster-card').forEach((c) => {
      const ti = +c.dataset.ti;
      c.classList.toggle('selected', ti === this.selectedThruster || inGroup(ti));
    });
  }

  // 查詢指定推力器卡片內的輸入元素
  cardInput(ti, f) {
    return this.container.querySelector(`.thruster-card[data-ti="${ti}"] input[data-f="${f}"]`);
  }

  onChange(f, value, idx) {
    if (f.path === 'controller.type') {
      set(this.cfg, f.path, value); // 先更新型別，再重建面板
      // 切換控制器時補上該型別預設參數（依案例尺度）
      const presetDefaults = {
        medium: {
          pid: { gains: [[600, 2570, 16], [600, 2570, 16], [600, 2570, 16]], intLimit: [15, 15, 15], intWindow: 0.25 },
          lqr: { Qdiag: [360000, 360000, 360000, 2540000, 2540000, 2540000], Rdiag: [1, 1, 1] },
          qpd: { Kp: [600, 600, 600], Kd: [2570, 2570, 2570] },
        },
        small: {
          pid: { gains: [[3, 6.1, 0.08], [3, 6.1, 0.08], [3, 6.1, 0.08]], intLimit: [2, 2, 2], intWindow: 0.25 },
          lqr: { Qdiag: [9, 9, 9, 14.2, 14.2, 14.2], Rdiag: [1, 1, 1] },
          qpd: { Kp: [3, 3, 3], Kd: [6.1, 6.1, 6.1] },
        },
        large: {
          pid: { gains: [[11280, 48300, 300], [34200, 146500, 912], [34200, 146500, 912]], intLimit: [2000, 2000, 2000], intWindow: 0.25 },
          lqr: { Qdiag: [1.27e8, 1.17e9, 1.17e9, 1.72e9, 8.23e9, 8.23e9], Rdiag: [1, 1, 1] },
          qpd: { Kp: [11280, 34200, 34200], Kd: [48300, 146500, 146500] },
        },
      };
      this.cfg.controller.params = (presetDefaults[this.cfg.presetName] ?? presetDefaults.medium)[value];
      this.build();          // 重建參數面板（顯示新型別的欄位）
      this.refreshVisibility();
      this.rebuild();
      return;
    }
    const val = get(this.cfg, f.path);
    if (f.vec && idx >= 0) val[idx] = value;
    else set(this.cfg, f.path, value);
    this.refreshVisibility();
    this.rebuild();
  }

  refreshVisibility() {
    (this.conditional || []).forEach(({ el, showIf }) => {
      el.style.display = !showIf || showIf === this.cfg.controller.type ? '' : 'none';
    });
  }

  // 驗證 + 重建模擬
  rebuild() {
    const err = this.validate();
    if (err) {
      this.msg.className = 'validate-msg err';
      this.msg.textContent = '⚠ ' + err;
      return;
    }
    this.msg.className = 'validate-msg ok';
    this.msg.textContent = '參數已更新';
    this.cb.onRebuild();
  }

  validate() {
    const v = this.cfg.vehicle;
    if (v.dryMass <= 0) return '乾質量須為正';
    if (v.mProp < 0) return '推進劑量不可為負';
    const J = M3.inertiaTensor(v.Ixx, v.Iyy, v.Izz, v.Ixy, v.Ixz, v.Iyz);
    if (!M3.cholesky(J)) return '慣量張量非正定（檢查 Ixy/Ixz/Iyz 是否過大）';
    const eigs = M3.eigenvaluesSym(J);
    if (eigs.some((e) => e <= 0)) return '慣量張量特徵值非正';
    const [a, b, c] = eigs.map(Math.abs).sort((x, y) => y - x);
    if (a >= b + c) return '主慣量違反三角不等式（最大慣量 ≥ 兩者之和），剛體不可實現';
    if (this.cfg.rcs.thrusters.filter((t) => t.enabled).length === 0) return '至少需要一具啟用的推力器';
    for (const t of this.cfg.rcs.thrusters) {
      if (t.enabled && t.maxThrust <= 0) return `推力器 ${t.name} 推力須為正`;
    }
    const s = this.cfg.sim;
    if (!(s.duration > 0) || !isFinite(s.duration)) return '模擬時長須為有限正數';
    if (!(s.dt > 0) || !isFinite(s.dt)) return '物理步長須為有限正數';
    if (!(s.controlPeriod >= s.dt)) return '控制週期不得小於物理步長';
    if (!(s.recordInterval > 0)) return '記錄間隔須為正';
    if (!(s.settleBandDeg > 0)) return '穩定判據帶寬須為正';
    for (const t of this.cfg.rcs.thrusters) {
      if (t.enabled) {
        if (!(t.isp > 0)) return `推力器 ${t.name} 比衝須為正`;
        if (!(t.minImpulse >= 0)) return `推力器 ${t.name} 最小衝量不可為負`;
        if (!(t.delayOpen >= 0) || !(t.delayClose >= 0)) return `推力器 ${t.name} 延遲不可為負`;
        if (t.maxDuty <= 0 || t.maxDuty > 1) return `推力器 ${t.name} 占空比須在 (0, 1]`;
        if (Math.hypot(t.dir[0], t.dir[1], t.dir[2]) < 1e-9) return `推力器 ${t.name} 方向向量不可為零`;
      }
    }
    return null;
  }

  // 套用預設（重建整個 UI）
  applyPreset(name) {
    this.cb.onPreset(name);
  }
}
