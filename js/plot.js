// =====================================================================
// plot.js — Chart.js 時間序列圖表管理
// 圖表清單見 index.html 分頁；資料來源 = Simulation.record（記錄間隔取樣）
// =====================================================================
import { Q } from './mathUtils.js';

const AXIS_COLORS = ['#ff6b6b', '#4fd8e0', '#b28cff']; // X/Y/Z
const DEG = 180 / Math.PI;

function baseOpts(legend = true) {
  return {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    pointRadius: 0,
    borderWidth: 1.4,
    legend: legend ? { display: true, labels: { color: '#8b96ad', font: { size: 10 } } } : { display: false },
    scales: {
      x: { type: 'linear', ticks: { color: '#5a6478', font: { size: 10 }, maxTicksLimit: 10 }, grid: { color: '#1d2438' }, title: { display: true, text: 't [s]', color: '#5a6478', font: { size: 10 } } },
      y: { ticks: { color: '#8b96ad', font: { size: 10 } }, grid: { color: '#1d2438' } },
    },
    plugins: { tooltip: { enabled: false } },
  };
}

function mkDataset(label, color, data, extra = {}) {
  return { label, data, borderColor: color, backgroundColor: color, showLine: true, ...extra };
}

export class PlotManager {
  constructor(container) {
    this.container = container;
    this.charts = {};
    this.currentTab = 'attitude';
    this.lastIdx = {};
  }

  buildAll(sim, thrusterNames) {
    this.sim = sim;
    this.thrusterNames = thrusterNames;
    this.container.innerHTML = '';
    this.charts = {};
    this.lastIdx = {};
    const R = sim.record;

    // ---- 姿態（Euler 角）----
    this.charts.attitude = this.mkChart('attitude', {
      datasets: ['Roll φ', 'Pitch θ', 'Yaw ψ'].map((n, a) =>
        mkDataset(n, AXIS_COLORS[a], this.mapT(R.t, (i) => R.euler[i][a] * DEG)))
    }, { y: { title: { display: true, text: 'deg', color: '#5a6478' } } });

    // ---- 姿態誤差角 ----
    this.charts.error = this.mkChart('error', {
      datasets: [
        mkDataset('總誤差角', '#f0b25f', this.mapT(R.t, (i) => R.attErr[i] * DEG), { borderWidth: 2 }),
        ...['δx', 'δy', 'δz'].map((n, a) =>
          mkDataset(n, AXIS_COLORS[a], this.mapT(R.t, (i) => R.attErrVec[i][a] * DEG), { borderDash: [4, 3] })),
      ],
    }, { y: { title: { display: true, text: 'deg', color: '#5a6478' } } });

    // ---- 角速度 ----
    this.charts.rates = this.mkChart('rates', {
      datasets: ['ωx (roll)', 'ωy (pitch)', 'ωz (yaw)'].map((n, a) =>
        mkDataset(n, AXIS_COLORS[a], this.mapT(R.t, (i) => R.w[i][a] * DEG)))
    }, { y: { title: { display: true, text: 'deg/s', color: '#5a6478' } } });

    // ---- 力矩 ----
    this.charts.torque = this.mkChart('torque', {
      datasets: [
        ...['τcmd x', 'τcmd y', 'τcmd z'].map((n, a) =>
          mkDataset(n, AXIS_COLORS[a], this.mapT(R.t, (i) => R.tauCmd[i][a]), { borderDash: [5, 3] })),
        ...['τact x', 'τact y', 'τact z'].map((n, a) =>
          mkDataset(n, AXIS_COLORS[a], this.mapT(R.t, (i) => R.tauActual[i][a]))),
      ],
    }, { y: { title: { display: true, text: 'N·m', color: '#5a6478' } } });

    // ---- 閥門開關（Gantt 式階梯圖）----
    const vDatasets = [];
    const nV = sim.rcs.valves.length;
    sim.rcs.valves.forEach((v, i) => {
      const base = (nV - 1 - i);
      const data = [];
      for (let k = 0; k < R.t.length; k++) data.push({ x: R.t[k], y: base + R.thrusterOn[k][i] * 0.85 });
      vDatasets.push(mkDataset(this.thrusterNames[i] ?? `T${i + 1}`, '#f0b25f', data, { stepped: true, borderWidth: 1 }));
    });
    this.charts.valves = this.mkChart('valves', { datasets: vDatasets }, {
      y: { min: 0, max: nV, ticks: { stepSize: 1, color: '#8b96ad', font: { size: 9 }, callback: (v) => this.thrusterNames[nV - 1 - v] ?? '', autoSkip: false }, grid: { color: '#1d2438' } },
    });

    // ---- 推進劑與 Δv ----
    this.charts.propellant = this.mkChart('propellant', {
      datasets: [
        mkDataset('推進劑剩餘 [kg]', '#62c96b', this.mapT(R.t, (i) => R.mProp[i])),
        mkDataset('累計 Δv [m/s]', '#4fd8e0', this.mapT(R.t, (i) => R.dv[i]), { yAxisID: 'y1' }),
      ],
    }, {
      y: { position: 'left', title: { display: true, text: 'kg', color: '#5a6478' } },
      y1: { position: 'right', grid: { drawOnChartArea: false }, title: { display: true, text: 'm/s', color: '#5a6478' } },
    });

    // ---- 平移狀態 ----
    this.charts.trans = this.mkChart('trans', {
      datasets: [
        mkDataset('|v| [m/s]', '#4fd8e0', this.mapT(R.t, (i) => Math.hypot(...R.v[i]))),
        mkDataset('|r| [m]', '#f0b25f', this.mapT(R.t, (i) => Math.hypot(...R.r[i]))),
        mkDataset('|Δv 向量| [m/s]', '#62c96b', this.mapT(R.t, (i) => Math.hypot(R.v[i][0] - R.v[0][0], R.v[i][1] - R.v[0][1], R.v[i][2] - R.v[0][2]))),
      ],
    }, { y: { title: { display: true, text: 'm/s, m', color: '#5a6478' } } });

    // ---- 相圖（誤差角 vs 角速度，每軸）----
    this.charts.phase = this.mkChart('phase', {
      datasets: ['x (roll)', 'y (pitch)', 'z (yaw)'].map((n, a) => ({
        label: n, data: [], borderColor: AXIS_COLORS[a], backgroundColor: AXIS_COLORS[a],
        showLine: false, pointRadius: 1.1,
      })),
    }, {
      x: { title: { display: true, text: '誤差角 δ [deg]', color: '#5a6478' }, ticks: { color: '#5a6478', font: { size: 10 } }, grid: { color: '#1d2438' } },
      y: { title: { display: true, text: '角速度 ω [deg/s]', color: '#5a6478' } },
    });

    this.show(this.currentTab);
  }

  mapT(tArr, fn) {
    const out = [];
    for (let i = 0; i < tArr.length; i++) out.push({ x: tArr[i], y: fn(i) });
    return out;
  }

  mkChart(id, cfg, extraScales = {}) {
    const cvs = document.createElement('canvas');
    cvs.dataset.testid = `chart-${id}`;
    this.container.appendChild(cvs);
    const opts = baseOpts();
    Object.assign(opts.scales, extraScales);
    return new Chart(cvs, { type: 'line', data: cfg, options: opts });
  }

  show(tab) {
    this.currentTab = tab;
    Object.entries(this.charts).forEach(([k, c]) => {
      c.canvas.style.display = k === tab ? '' : 'none';
      if (k === tab) c.resize();
    });
  }

  // 即時更新（自上次以後的新記錄點附加）
  refresh() {
    if (!this.sim) return;
    const R = this.sim.record;
    const n = R.t.length;
    for (const [k, chart] of Object.entries(this.charts)) {
      const start = this.lastIdx[k] ?? 0;
      if (start >= n) continue;
      const t0 = start > 0 ? R.t[start - 1] : R.t[0];
      const idxs = [];
      for (let i = Math.max(0, start - 1); i < n; i++) idxs.push(i);
      switch (k) {
        case 'attitude': this.appendN(chart, idxs, (i) => R.euler[i].map((e) => e * DEG)); break;
        case 'error': this.appendN(chart, idxs, (i) => [R.attErr[i] * DEG, ...R.attErrVec[i].map((d) => d * DEG)]); break;
        case 'rates': this.appendN(chart, idxs, (i) => R.w[i].map((w) => w * DEG)); break;
        case 'torque': this.appendN(chart, idxs, (i) => [...R.tauCmd[i], ...R.tauActual[i]]); break;
        case 'valves': {
          const nV = this.sim.rcs.valves.length;
          chart.data.datasets.forEach((ds, vi) => {
            const base = nV - 1 - vi;
            for (const i of idxs) ds.data.push({ x: R.t[i], y: base + R.thrusterOn[i][vi] * 0.85 });
          });
          break;
        }
        case 'propellant': this.appendN(chart, idxs, (i) => [R.mProp[i], R.dv[i]]); break;
        case 'trans': this.appendN(chart, idxs, (i) => [
          Math.hypot(...R.v[i]), Math.hypot(...R.r[i]),
          Math.hypot(R.v[i][0] - R.v[0][0], R.v[i][1] - R.v[0][1], R.v[i][2] - R.v[0][2]),
        ]); break;
        case 'phase': {
          for (const i of idxs)
            for (let a = 0; a < 3; a++)
              chart.data.datasets[a].data.push({ x: R.attErrVec[i][a] * DEG, y: R.w[i][a] * DEG });
          break;
        }
      }
      this.lastIdx[k] = n;
      chart.update('none');
    }
  }

  appendN(chart, idxs, fn) {
    const R = this.sim.record;
    chart.data.datasets.forEach((ds, di) => {
      for (const i of idxs) {
        const vals = fn(i);
        ds.data.push({ x: R.t[i], y: vals[di] });
      }
    });
  }
}
