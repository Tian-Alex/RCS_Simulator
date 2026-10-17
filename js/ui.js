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
      { path: 'sim.duration', label: '模擬時長 [s]', step: 1 },
      { path: 'sim.dt', label: '物理步長 [s]', step: 0.001 },
      { path: 'sim.controlPeriod', label: '控制週期 [s]', step: 0.01 },
      { path: 'sim.recordInterval', label: '記錄間隔 [s]', step: 0.01 },
      { path: 'sim.settleBandDeg', label: '穩定判據帶寬 [°]', step: 0.1 },
    ],
  },
  {
    id: 'vehicle', title: '載具與慣量',
    fields: [
      { path: 'vehicle.dryMass', label: '乾質量 [kg]', step: 0.1 },
      { path: 'vehicle.mProp', label: '推進劑 [kg]', step: 0.1 },
      { path: 'vehicle.length', label: '箭體長度 [m]', step: 0.1 },
      { path: 'vehicle.radius', label: '箭體半徑 [m]', step: 0.05 },
      { path: 'vehicle.cg', label: '質心位置 [m]', vec: 3, step: 0.01 },
      { path: 'vehicle.Ixx', label: 'Ixx [kg·m²]', step: 1 },
      { path: 'vehicle.Iyy', label: 'Iyy [kg·m²]', step: 1 },
      { path: 'vehicle.Izz', label: 'Izz [kg·m²]', step: 1 },
      { path: 'vehicle.Ixy', label: 'Ixy [kg·m²]', step: 1 },
      { path: 'vehicle.Ixz', label: 'Ixz [kg·m²]', step: 1 },
      { path: 'vehicle.Iyz', label: 'Iyz [kg·m²]', step: 1 },
      { note: '輸入慣性積 P = ∫xy dm（正號），張量非對角項自動取 −P，見 docs/equations.md §1' },
    ],
  },
  {
    id: 'initial', title: '初始條件',
    fields: [
      { path: 'initial.attitude', label: '初始姿態 φ/θ/ψ [°]', vec: 3, step: 1 },
      { path: 'initial.rates', label: '初始角速度 [°/s]', vec: 3, step: 0.1 },
      { path: 'initial.position', label: '初始位置 [m]', vec: 3, step: 1 },
      { path: 'initial.velocity', label: '初始速度 [m/s]', vec: 3, step: 0.01 },
    ],
  },
  {
    id: 'target', title: '目標姿態',
    fields: [
      { path: 'target.attitude', label: '目標 φ/θ/ψ [°]', vec: 3, step: 1 },
    ],
  },
  {
    id: 'controller', title: '控制器',
    fields: [
      { path: 'controller.type', label: '控制律', select: CONTROLLERS },
      { path: 'controller.params.gains', label: 'PID 增益 Kp/Kd/Ki（每軸）', matrix: [3, 3], showIf: 'pid', step: 0.1 },
      { path: 'controller.params.intLimit', label: 'PID 積分上限 [N·m·s]', vec: 3, showIf: 'pid', step: 0.5 },
      { path: 'controller.params.intWindow', label: 'PID 積分視窗 [rad]', scalar: true, showIf: 'pid', step: 0.01 },
      { path: 'controller.params.Qdiag', label: 'LQR Q（θx θy θz ωx ωy ωz）', vec: 6, showIf: 'lqr', step: 10 },
      { path: 'controller.params.Rdiag', label: 'LQR R（每軸）', vec: 3, showIf: 'lqr', step: 0.1 },
      { path: 'controller.params.Kp', label: 'QPD Kp（每軸）', vec: 3, showIf: 'qpd', step: 1 },
      { path: 'controller.params.Kd', label: 'QPD Kd（每軸）', vec: 3, showIf: 'qpd', step: 1 },
    ],
  },
  {
    id: 'modulator', title: '調變器',
    fields: [
      { path: 'modulator.mode', label: '調變模式', select: MODULATORS },
      { path: 'modulator.pwmPeriod', label: 'PWM 週期 [s]', scalar: true, step: 0.05 },
      { path: 'modulator.pwpf.Km', label: 'PWPF 濾波增益 Km', scalar: true, step: 0.1 },
      { path: 'modulator.pwpf.Tm', label: 'PWPF 時間常數 Tm [s]', scalar: true, step: 0.01 },
      { path: 'modulator.pwpf.Uon', label: 'PWPF 開啟門檻 Uon', scalar: true, step: 0.005 },
      { path: 'modulator.pwpf.Uoff', label: 'PWPF 關閉門檻 Uoff', scalar: true, step: 0.005 },
    ],
  },
  {
    id: 'allocation', title: '推力分配',
    fields: [
      { path: 'allocation.wForce', label: '淨力懲罰權重 wF', scalar: true, step: 0.01 },
      { note: '分配器求解：min ‖Tf−τ‖² + wF‖Ff‖²（0 ≤ f ≤ Fmax），見 docs/equations.md §4' },
    ],
  },
  {
    id: 'disturbances', title: '擾動',
    fields: [
      { path: 'disturbances.constantTorque', label: '常值力矩 [N·m]', vec: 3, step: 0.01 },
      { path: 'disturbances.randomTorque.sigma', label: '隨機力矩 σ [N·m/√Hz]', vec: 3, step: 0.005 },
      { path: 'disturbances.randomTorque.interval', label: '隨機取樣間隔 [s]', scalar: true, step: 0.1 },
      { path: 'disturbances.randomTorque.seed', label: '隨機種子', scalar: true, step: 1 },
      { path: 'disturbances.gravityGradient.enabled', label: '重力梯度', bool: true },
      { path: 'disturbances.gravityGradient.altitude', label: '軌道高度 [m]', scalar: true, step: 10000 },
      { path: 'disturbances.gravityGradient.phase0', label: '初始軌道相位 [rad]', scalar: true, step: 0.1 },
      { path: 'disturbances.solarPressure.enabled', label: '太陽輻射壓', bool: true },
      { path: 'disturbances.solarPressure.sunDirection', label: '太陽方向（慣性系）', vec: 3, step: 0.1 },
      { path: 'disturbances.solarPressure.Cr', label: '反射係數 Cr', scalar: true, step: 0.05 },
      { path: 'disturbances.solarPressure.area', label: '有效面積 [m²]', scalar: true, step: 0.1 },
      { path: 'disturbances.solarPressure.cpOffset', label: '壓力中心偏移 [m]', vec: 3, step: 0.01 },
    ],
  },
  {
    id: 'sensors', title: '感測器',
    fields: [
      { path: 'sensors.enabled', label: '啟用感測誤差', bool: true },
      { path: 'sensors.gyroNoise', label: '陀螺儀雜訊 [rad/s/√Hz]', vec: 3, step: 0.0002 },
      { path: 'sensors.gyroBias', label: '陀螺儀偏置 [rad/s]', vec: 3, step: 0.0001 },
      { path: 'sensors.attNoise', label: '姿態量測雜訊 [rad]', vec: 3, step: 0.0001 },
      { path: 'sensors.interval', label: '取樣間隔 [s]', scalar: true, step: 0.01 },
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
      wrap.innerHTML = `<label>${f.label}</label>`;
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
      lab.textContent = f.label;
      wrap.appendChild(lab);
      body.appendChild(wrap);
      return;
    }
    const val = get(this.cfg, f.path);
    const n = f.vec ? f.vec : 1;
    const wrap = document.createElement('div');
    wrap.className = 'field';
    wrap.innerHTML = `<label>${f.label}</label>`;
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

  // 推力器表格
  buildThrusterTable() {
    const panel = document.createElement('div');
    panel.className = 'panel';
    panel.innerHTML = `
      <div class="panel-head" role="button" tabindex="0"><span>RCS 推力器（${this.cfg.rcs.thrusters.length} 具）</span><span class="chev">▾</span></div>
      <div class="panel-body"><div class="table-scroll"><table class="thruster-table"></table></div></div>`;
    panel.querySelector('.panel-head').addEventListener('click', () => panel.classList.toggle('collapsed'));
    const table = panel.querySelector('table');
    const cols = [
      ['名稱', (t) => `<input data-f="name" value="${t.name}" style="width:64px;text-align:left">`],
      ['px', (t) => num(t.pos[0], 'pos', 0)], ['py', (t) => num(t.pos[1], 'pos', 1)], ['pz', (t) => num(t.pos[2], 'pos', 2)],
      ['dx', (t) => num(t.dir[0], 'dir', 0)], ['dy', (t) => num(t.dir[1], 'dir', 1)], ['dz', (t) => num(t.dir[2], 'dir', 2)],
      ['F[N]', (t) => num(t.maxThrust, 'maxThrust', -1)],
      ['Imin', (t) => num(t.minImpulse, 'minImpulse', -1)],
      ['t_on', (t) => num(t.delayOpen, 'delayOpen', -1)],
      ['t_off', (t) => num(t.delayClose, 'delayClose', -1)],
      ['duty', (t) => num(t.maxDuty, 'maxDuty', -1)],
      ['Isp', (t) => num(t.isp, 'isp', -1)],
      ['on', (t) => `<input type="checkbox" data-f="enabled" ${t.enabled ? 'checked' : ''}>`],
    ];
    function num(v, f, i) {
      return `<input type="number" step="0.01" data-f="${f}" data-i="${i}" value="${+v.toFixed(4)}">`;
    }
    const thead = `<tr>${cols.map((c) => `<th>${c[0]}</th>`).join('')}</tr>`;
    const rows = this.cfg.rcs.thrusters.map((t, ti) =>
      `<tr>${cols.map((c) => `<td class="${c[0] === '名稱' ? 'tname' : ''}">${c[1](t)}</td>`).join('')}</tr>`
    ).join('');
    table.innerHTML = thead + rows;
    table.querySelectorAll('input').forEach((inp) => {
      inp.addEventListener('change', () => {
        const tr = inp.closest('tr');
        const ti = [...table.querySelectorAll('tr')].indexOf(tr) - 1;
        const t = this.cfg.rcs.thrusters[ti];
        const f = inp.dataset.f;
        const i = inp.dataset.i !== undefined ? parseInt(inp.dataset.i) : -1;
        if (inp.type === 'checkbox') t[f] = inp.checked;
        else if (inp.dataset.f === 'name') t.name = inp.value;
        else if (i >= 0) t[f][i] = parseFloat(inp.value);
        else t[f] = parseFloat(inp.value);
        this.rebuild();
      });
    });
    this.container.appendChild(panel);
  }

  onChange(f, value, idx) {
    if (f.path === 'controller.type') {
      set(this.cfg, f.path, value); // 先更新型別，再重建面板
      // 切換控制器時補上該型別預設參數
      const defaults = {
        pid: { gains: [[76, 915, 0.1], [76, 915, 0.1], [76, 915, 0.1]], intLimit: [5, 5, 5], intWindow: 0.1 },
        lqr: { Qdiag: [6000, 6000, 6000, 60000, 60000, 60000], Rdiag: [1, 1, 1] },
        qpd: { Kp: [76, 76, 76], Kd: [915, 915, 915] },
      };
      this.cfg.controller.params = defaults[value];
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
