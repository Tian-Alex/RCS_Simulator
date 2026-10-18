// =====================================================================
// main.js — 應用程式進入點：組裝 UI / 3D / 圖表 / 模擬迴圈
// =====================================================================
import { Q } from './mathUtils.js';
import { presetByName, PRESETS } from './parameters.js';
import { Simulation } from './simulation.js';
import { Scene3D } from './scene3d.js';
import { PlotManager } from './plot.js';
import { UI } from './ui.js';

const DEG = 180 / Math.PI;

// ------------------- 全域狀態 -------------------
let cfg = presetByName('medium');
let sim = new Simulation(cfg);
let scene3d = null;
let plots = null;
let ui = null;
let running = false;
let speed = 1;
let lastFrameT = performance.now();
let plotRefreshT = 0;

// ------------------- DOM -------------------
const $ = (id) => document.getElementById(id);
const btnRun = $('btn-run'), btnReset = $('btn-reset'), btnFF = $('btn-ff');
const clock = $('sim-clock');
const chartTabs = $('chart-tabs');
const metricsBox = $('metrics-box'), lqrBox = $('lqr-box'), chartBox = $('chart-box');
const helpBox = $('help-box'), phaseTools = $('phase-tools'), chartHost = document.querySelector('.chart-host');

// ------------------- 初始化 -------------------
function init() {
  scene3d = new Scene3D($('view3d'), cfg);
  plots = new PlotManager(chartBox);
  ui = new UI(cfg, {
    onRebuild: rebuildSim,
    onPreset: (name) => {
      cfg = presetByName(name);
      ui.cfg = cfg;
      ui.build();
      rebuildSim();
    },
    onSelectThruster: (ti) => scene3d.setThrusterSelection(ti), // 卡片點選 → 3D 高亮
    onGroupHighlight: (indices) => scene3d.setGroupHighlight(indices ?? []), // 軸向分組 → 3D 多具高亮
  });
  scene3d.onSelectThruster = (ti) => ui.setThrusterSelection(ti); // 3D 點選 → 卡片高亮
  plots.buildAll(sim, cfg.rcs.thrusters.map((t) => t.name));
  rebuildSim();
  wireControls();
  updateLqrBox();
  requestAnimationFrame(loop);
}

// 參數變更 / 重置：重建模擬與圖表
function rebuildSim() {
  sim = new Simulation(cfg);
  scene3d.setVehicle(cfg);
  scene3d.resetTrajectory();
  plots.buildAll(sim, cfg.rcs.thrusters.map((t) => t.name));
  plots.refresh();
  updateLqrBox();
  updateMetrics();
  updateHUD();
  running = false;
  btnRun.textContent = '開始';
  ui.setLocked(false);
}

function wireControls() {
  btnRun.addEventListener('click', () => {
    if (sim.finished) return;
    running = !running;
    btnRun.textContent = running ? '暫停' : '繼續';
    ui.setLocked(running);
  });
  btnReset.addEventListener('click', () => {
    rebuildSim();
  });
  btnFF.addEventListener('click', () => {
    ui.setLocked(true);
    const t0 = performance.now();
    while (!sim.finished) sim.step(cfg.sim.dt);
    console.log(`快轉完成：${((performance.now() - t0) / 1000).toFixed(2)} s 計算時間`);
    running = false;
    btnRun.textContent = '開始';
    ui.setLocked(false);
    plots.refresh();
    updateMetrics();
    updateHUD();
  });
  $('speed').addEventListener('change', (e) => (speed = parseFloat(e.target.value)));
  chartTabs.addEventListener('click', (e) => {
    const tab = e.target.closest('.tab');
    if (!tab) return;
    chartTabs.querySelectorAll('.tab').forEach((t) => t.classList.remove('active'));
    tab.classList.add('active');
    const id = tab.dataset.tab;
    const showChart = id !== 'metrics' && id !== 'help';
    plots.show(id);
    metricsBox.hidden = id !== 'metrics';
    lqrBox.hidden = id !== 'metrics' || cfg.controller.type !== 'lqr';
    helpBox.hidden = id !== 'help';
    phaseTools.hidden = id !== 'phase';
    chartBox.style.display = showChart ? '' : 'none';
    if (id !== 'metrics' && id !== 'help') plots.refresh();
    else if (id === 'metrics') updateMetrics();
  });
  // 相圖模式切換
  phaseTools.querySelectorAll('.phase-mode').forEach((b) =>
    b.addEventListener('click', () => {
      phaseTools.querySelectorAll('.phase-mode').forEach((x) => x.classList.remove('active'));
      b.classList.add('active');
      plots.setPhaseMode(b.dataset.phaseMode);
    }));
  // 全螢幕圖表
  $('btn-fullscreen').addEventListener('click', () => chartHost.classList.toggle('fullscreen'));
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') chartHost.classList.remove('fullscreen');
  });
  document.querySelectorAll('[data-preset]').forEach((b) =>
    b.addEventListener('click', () => ui.applyPreset(b.dataset.preset)));
  $('btn-csv').addEventListener('click', exportCsv);
  $('btn-export-case').addEventListener('click', exportCase);
  $('btn-import-case').addEventListener('click', () => $('file-case').click());
  $('file-case').addEventListener('change', importCase);
}

// ------------------- 主迴圈 -------------------
function loop(now) {
  requestAnimationFrame(loop);
  const frameDt = Math.min(0.1, (now - lastFrameT) / 1000);
  lastFrameT = now;
  if (running && !sim.finished) {
    let steps = Math.round((frameDt * speed) / cfg.sim.dt);
    steps = Math.min(steps, 600); // 保持 UI 順暢
    for (let i = 0; i < steps && !sim.finished; i++) sim.step(cfg.sim.dt);
    if (sim.finished) { running = false; btnRun.textContent = '開始'; ui.setLocked(false); }
  }
  if (sim.t > 0 || running) {
    scene3d.update(sim);
    updateHUD();
    if (now - plotRefreshT > 350) {
      plotRefreshT = now;
      plots.refresh();
    }
  } else {
    scene3d.update(sim);
  }
}

function updateHUD() {
  const s = sim.state;
  const eu = Q.toEuler(s.q);
  clock.textContent = `t = ${sim.t.toFixed(2)} s`;
  $('hud-att').innerHTML = `<b>姿態</b> φ ${(eu.roll * DEG).toFixed(1)}° θ ${(eu.pitch * DEG).toFixed(1)}° ψ ${(eu.yaw * DEG).toFixed(1)}°`;
  $('hud-rate').innerHTML = `<b>ω</b> [${s.w.map((w) => (w * DEG).toFixed(2)).join(', ')}] °/s`;
  $('hud-err').innerHTML = `<b>誤差</b> ${(sim.record.attErr.length ? sim.record.attErr[sim.record.attErr.length - 1] * DEG : 0).toFixed(2)}°`;
  $('hud-prop').innerHTML = `<b>推進劑</b> ${Math.max(0, s.mProp).toFixed(3)} kg · <b>Δv</b> ${sim.dvAccum.toFixed(4)} m/s`;
}

// ------------------- LQR 狀態 -------------------
function updateLqrBox() {
  lqrBox.hidden = cfg.controller.type !== 'lqr';
  if (cfg.controller.type !== 'lqr' || !sim.lqrStatus) return;
  const r = sim.lqrStatus;
  if (!r.ok) {
    lqrBox.textContent = `LQR 求解失敗：${r.reason}`;
    return;
  }
  const K = r.K.map((row) => row.map((x) => x.toFixed(2)).join('  ')).join('\n');
  lqrBox.textContent =
    `Newton–Kleinman CARE 求解成功\n` +
    `疊代次數: ${r.iterations}   殘差: ${r.residual.toExponential(2)}\n` +
    `狀態 x = [δθ; ω]（6 維），控制 τ = −K·x\n\nK (3×6) =\n${K}\n\n` +
    `軸向可達力矩 [N·m]: [${sim.capacity.map((c) => c.toFixed(1)).join(', ')}]\n` +
    `雙向可控: ${sim.attainability.bidirectional.map((b) => (b ? '✓' : '✗')).join(' ')}`;
}

// ------------------- 效能指標 -------------------
function updateMetrics() {
  const m = sim.metrics();
  const rows = [
    ['穩定時間（進入帶寬後不再離開）', isFinite(m.settleTime) ? `${m.settleTime.toFixed(1)} s` : '未穩定'],
    ['最終姿態誤差角', `${m.finalAttErrDeg.toFixed(3)} °`],
    ['RMS 姿態誤差', `${m.rmsAttErrDeg.toFixed(3)} °`],
    ['最大姿態誤差', `${m.maxAttErrDeg.toFixed(1)} °`],
    ['推進劑消耗', `${m.propellantUsed.toFixed(4)} kg`],
    ['推進劑剩餘', `${m.propellantRemaining.toFixed(4)} kg`],
    ['累計 Δv（RCS 淨力）', `${m.dvAccum.toFixed(4)} m/s`],
    ['Δv 向量 |v(t)−v(0)|', `${Math.hypot(...m.dvVector).toFixed(4)} m/s`],
    ['控制能量 ∫‖τ‖²dt', `${m.controlEffort.toFixed(1)} N²·m²·s`],
  ];
  // 推力器統計：每具一列（表格排列，避免長字串換行重疊）
  const G0 = 9.80665;
  const tRows = sim.rcs.thrusters.map((t, i) => {
    const mdot = (t.maxThrust * m.onTimes[i]) / (t.isp * G0);
    return `<tr><td>${t.name}${t.enabled ? '' : '（停用）'}</td><td>${m.cycles[i]}</td><td>${m.onTimes[i].toFixed(1)}</td><td>${mdot.toFixed(4)}</td></tr>`;
  }).join('');
  metricsBox.innerHTML = '<h3>效能指標（目前參數設定）</h3>' +
    `<table>${rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join('')}</table>` +
    `<h4 style="font-size:12px;margin:10px 0 2px;color:#8b96ad">各推力器統計</h4>` +
    `<table class="mtable"><tr><th>推力器</th><th>循環次數</th><th>累計點火 [s]</th><th>推進劑 [kg]</th></tr>${tRows}</table>` +
    `<p style="font-size:11px;color:#5a6478;margin-top:8px">Δv 定義見 docs/equations.md §9：累計 RCS 淨加速度積分 ≠ |v(t)−v(0)|（純力偶消耗推進劑但不產生淨 Δv）</p>`;
}

// ------------------- CSV 匯出 -------------------
function exportCsv() {
  const R = sim.record;
  const n = R.t.length;
  const head = ['t', 'qw', 'qx', 'qy', 'qz', 'roll_deg', 'pitch_deg', 'yaw_deg',
    'errAngle_deg', 'errX_deg', 'errY_deg', 'errZ_deg',
    'wx_degs', 'wy_degs', 'wz_degs',
    'tauCmdX', 'tauCmdY', 'tauCmdZ', 'tauActX', 'tauActY', 'tauActZ',
    'mProp', 'dvAccum', 'vx', 'vy', 'vz', 'rx', 'ry', 'rz', 'Fx_I', 'Fy_I', 'Fz_I'];
  const lines = [head.join(',')];
  for (let i = 0; i < n; i++) {
    const e = R.euler[i].map((x) => x * DEG);
    const ev = R.attErrVec[i].map((x) => x * DEG);
    const w = R.w[i].map((x) => x * DEG);
    lines.push([
      R.t[i].toFixed(4), ...R.q[i].map((x) => x.toFixed(6)),
      ...e.map((x) => x.toFixed(4)), (R.attErr[i] * DEG).toFixed(4), ...ev.map((x) => x.toFixed(4)),
      ...w.map((x) => x.toFixed(4)),
      ...R.tauCmd[i].map((x) => x.toFixed(4)), ...R.tauActual[i].map((x) => x.toFixed(4)),
      R.mProp[i].toFixed(5), R.dv[i].toFixed(5),
      ...R.v[i].map((x) => x.toFixed(5)), ...R.r[i].map((x) => x.toFixed(4)),
      ...R.F_I[i].map((x) => x.toFixed(4)),
    ].join(','));
  }
  download(`${stamp()}_rcs_sim.csv`, lines.join('\n'), 'text/csv');
}

// ------------------- 案例匯入匯出 -------------------
function exportCase() {
  download(`${stamp()}_case.json`, JSON.stringify(cfg, null, 2), 'application/json');
}
function importCase(e) {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = () => {
    try {
      const data = JSON.parse(reader.result);
      if (!data.vehicle || !data.rcs) throw new Error('格式不符');
      cfg = data;
      ui.cfg = cfg;
      ui.build();
      rebuildSim();
      ui.msg.className = 'validate-msg ok';
      ui.msg.textContent = '案例已載入';
    } catch (err) {
      ui.msg.className = 'validate-msg err';
      ui.msg.textContent = '⚠ 匯入失敗：' + err.message;
    }
  };
  reader.readAsText(file);
  e.target.value = '';
}

function stamp() {
  const d = new Date();
  return `${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}_${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}`;
}
const pad = (x) => String(x).padStart(2, '0');

function download(name, content, type) {
  const blob = new Blob([content], { type });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  URL.revokeObjectURL(a.href);
}

init();
