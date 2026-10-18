import { Simulation } from './js/simulation.js';
import { presetByName } from './js/parameters.js';
const Tms = [0.5, 0.8, 1.2, 1.6, 2.0];
const Uons = [0.04, 0.06, 0.08, 0.12];
const ctrls = ['pid', 'lqr', 'qpd'];
for (const Tm of Tms) for (const Uon of Uons) {
  const line = [`${Tm}/${Uon}`];
  for (const ctrl of ctrls) {
    const cfg = presetByName('medium');
    cfg.controller.type = ctrl;
    if (ctrl === 'lqr') cfg.controller.params = { Qdiag: [6000,6000,6000,60000,60000,60000], Rdiag: [1,1,1] };
    if (ctrl === 'qpd') cfg.controller.params = { Kp: [76,76,76], Kd: [915,915,915] };
    cfg.sim.settleBandDeg = 1.0;
    cfg.modulator.pwpf = { Km: 1.0, Tm, Uon, Uoff: Uon / 2 };
    const sim = new Simulation(cfg);
    sim.build();
    while (!sim.finished) sim.step(cfg.sim.dt);
    const m = sim.metrics();
    line.push(`${ctrl}: ${(m.settleTime === Infinity ? 'X' : m.settleTime.toFixed(0))}/${m.finalAttErrDeg.toFixed(2)}`);
  }
  console.log(line.join('  '));
}
