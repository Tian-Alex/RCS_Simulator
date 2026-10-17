// =====================================================================
// scene3d.js — Three.js 3D 場景
//   ・火箭剛體（圓柱 + 鼻錐）、質心標記、本體軸（XYZ）
//   ・目標姿態框架（半透明白框）
//   ・各 RCS 推力器位置、噴嘴方向與點火排焰效果
//   ・角速度向量、速度向量、慣性系軌跡線、星空背景
// =====================================================================
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { V, Q } from './mathUtils.js';

const AXIS_COLORS = [0xff6b6b, 0x4fd8e0, 0xb28fff];

export class Scene3D {
  constructor(container, cfg) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 5000);
    this.camera.position.set(10, 6, 14);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;

    this.buildLights();
    this.buildStars();
    this.buildVehicle(cfg);
    this.buildTargetFrame();
    this.buildVectors();
    this.trajectory = [];

    this.rocketGroup = new THREE.Group();
    this.scene.add(this.rocketGroup);
    this.rocketGroup.add(this.bodyGroup, this.targetFrame, this.cgMarker, this.thrusterGroup);
    window.addEventListener('resize', () => this.resize());
  }

  buildLights() {
    this.scene.add(new THREE.AmbientLight(0x8899bb, 0.7));
    const sun = new THREE.DirectionalLight(0xfff2d0, 2.2);
    sun.position.set(30, 20, 15);
    this.scene.add(sun);
    const rim = new THREE.DirectionalLight(0x4fd8e0, 0.5);
    rim.position.set(-15, -5, -20);
    this.scene.add(rim);
  }

  buildStars() {
    const n = 1600;
    const pos = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      const r = 400 + Math.random() * 800;
      const th = Math.random() * Math.PI * 2, ph = Math.acos(2 * Math.random() - 1);
      pos[i * 3] = r * Math.sin(ph) * Math.cos(th);
      pos[i * 3 + 1] = r * Math.cos(ph);
      pos[i * 3 + 2] = r * Math.sin(ph) * Math.sin(th);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    this.scene.add(new THREE.Points(g, new THREE.PointsMaterial({ color: 0xaabbee, size: 1.6, sizeAttenuation: true })));
  }

  // 箭體幾何（依 cfg：長度、半徑、質心）
  buildVehicle(cfg) {
    const L = cfg.vehicle.length, R = cfg.vehicle.radius;
    this.bodyGroup = new THREE.Group();

    // 箭身（X 軸 = 縱軸）
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0xd8dde8, roughness: 0.55, metalness: 0.35 });
    const cyl = new THREE.Mesh(new THREE.CylinderGeometry(R, R, L * 0.8, 48), bodyMat);
    cyl.rotation.z = -Math.PI / 2;
    cyl.position.x = -L * 0.1;
    this.bodyGroup.add(cyl);

    // 鼻錐
    const nose = new THREE.Mesh(new THREE.ConeGeometry(R, L * 0.2, 48), new THREE.MeshStandardMaterial({ color: 0xb9c2d4, roughness: 0.4, metalness: 0.5 }));
    nose.rotation.z = -Math.PI / 2;
    nose.position.x = L * 0.4;
    this.bodyGroup.add(nose);

    // 環帶（視覺參考）
    for (const fx of [-L * 0.3, 0, L * 0.3]) {
      const ring = new THREE.Mesh(new THREE.TorusGeometry(R * 1.005, R * 0.02, 8, 64),
        new THREE.MeshStandardMaterial({ color: 0x39445c, roughness: 0.8 }));
      ring.rotation.y = Math.PI / 2;
      ring.position.x = fx;
      this.bodyGroup.add(ring);
    }

    // 本體軸
    const axisLen = L * 0.75;
    const labels = ['X', 'Y', 'Z'];
    for (let a = 0; a < 3; a++) {
      const dir = [0, 0, 0]; dir[a] = 1;
      const g = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(...V.scale(dir, -axisLen * 0.4)),
        new THREE.Vector3(...V.scale(dir, axisLen)),
      ]);
      const line = new THREE.Line(g, new THREE.LineBasicMaterial({ color: AXIS_COLORS[a] }));
      this.bodyGroup.add(line);
    }

    // 質心標記
    const cg = cfg.vehicle.cg;
    this.cgMarker = new THREE.Mesh(new THREE.OctahedronGeometry(R * 0.12),
      new THREE.MeshBasicMaterial({ color: 0xffd166 }));
    this.cgMarker.position.set(...cg);

    // 推力器 + 排焰
    this.thrusterGroup = new THREE.Group();
    this.flames = [];
    this.nozzleDirs = [];
    cfg.rcs.thrusters.forEach((t) => {
      const pos = t.pos, dir = t.dir;
      // 噴嘴（小圓柱，指向 dir）
      const noz = new THREE.Mesh(new THREE.CylinderGeometry(R * 0.045, R * 0.03, R * 0.09, 12),
        new THREE.MeshStandardMaterial({ color: 0x8b96ad, metalness: 0.7, roughness: 0.35 }));
      const q = quaternionFromX(dir);
      noz.quaternion.copy(q);
      noz.position.set(pos[0] - dir[0] * R * 0.045, pos[1] - dir[1] * R * 0.045, pos[2] - dir[2] * R * 0.045);
      this.thrusterGroup.add(noz);
      // 排焰（圓錐，沿 −dir）
      const flame = new THREE.Mesh(new THREE.ConeGeometry(R * 0.07, R * 0.5, 12),
        new THREE.MeshBasicMaterial({ color: 0xffc46b, transparent: true, opacity: 0.95 }));
      const fq = quaternionFromX(V.scale(dir, -1));
      flame.quaternion.copy(fq);
      flame.position.set(pos[0] - dir[0] * R * 0.28, pos[1] - dir[1] * R * 0.28, pos[2] - dir[2] * R * 0.28);
      flame.visible = false;
      this.thrusterGroup.add(flame);
      this.flames.push(flame);
    });
  }

  buildTargetFrame() {
    // 目標姿態框架（三條半透明白線）
    this.targetFrame = new THREE.Group();
    const len = 2.6;
    for (let a = 0; a < 3; a++) {
      const dir = [0, 0, 0]; dir[a] = 1;
      const g = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(...V.scale(dir, -len * 0.45)),
        new THREE.Vector3(...V.scale(dir, len)),
      ]);
      const line = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.35 }));
      this.targetFrame.add(line);
    }
  }

  buildVectors() {
    // ω 與 v 向量（慣性系放置於場景原點旁，長度誇張化以利觀察）
    const mk = (color) => {
      const g = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
      return new THREE.Line(g, new THREE.LineBasicMaterial({ color }));
    };
    this.omegaLine = mk(0xffd166);
    this.velLine = mk(0x62c96b);
    this.scene.add(this.omegaLine, this.velLine);
    // 軌跡
    const trajGeo = new THREE.BufferGeometry();
    this.trajPositions = new Float32Array(6000 * 3);
    trajGeo.setAttribute('position', new THREE.BufferAttribute(this.trajPositions, 3));
    this.trajLine = new THREE.Line(trajGeo, new THREE.LineBasicMaterial({ color: 0x4fd8e0, transparent: true, opacity: 0.5 }));
    this.trajCount = 0;
    this.scene.add(this.trajLine);
  }

  setVehicle(cfg) {
    // 重建箭體（參數變更時）
    this.rocketGroup.remove(this.bodyGroup, this.cgMarker, this.thrusterGroup);
    this.buildVehicle(cfg);
    this.rocketGroup.add(this.bodyGroup, this.cgMarker, this.thrusterGroup);
  }

  // 更新渲染（每幀）
  // sim：Simulation 實例
  update(sim) {
    const s = sim.state;
    const q = s.q;
    // 火箭姿態：body→inertial；Three.js 中場景為慣性系
    const Rm = Q.toMatrix(q);
    const m4 = new THREE.Matrix4();
    m4.set(
      Rm[0], Rm[1], Rm[2], s.r[0],
      Rm[3], Rm[4], Rm[5], s.r[1],
      Rm[6], Rm[7], Rm[8], s.r[2],
      0, 0, 0, 1
    );
    this.rocketGroup.matrix.identity();
    this.bodyGroup.matrix.copy(m4);
    this.bodyGroup.matrixAutoUpdate = false;
    this.cgMarker.matrix.copy(m4);
    this.cgMarker.matrixAutoUpdate = false;
    this.thrusterGroup.matrix.copy(m4);
    this.thrusterGroup.matrixAutoUpdate = false;

    // 目標姿態框架
    const RmT = Q.toMatrix(sim.qTarget);
    const m4t = new THREE.Matrix4();
    m4t.set(
      RmT[0], RmT[1], RmT[2], s.r[0],
      RmT[3], RmT[4], RmT[5], s.r[1],
      RmT[6], RmT[7], RmT[8], s.r[2],
      0, 0, 0, 1
    );
    this.targetFrame.matrix.copy(m4t);
    this.targetFrame.matrixAutoUpdate = false;

    // 排焰可見性
    const on = sim.rcs.valves.map((v) => v.state === 'OPEN');
    this.flames.forEach((f, i) => {
      f.visible = on[i] && sim.rcs.thrusters[i].enabled;
      if (f.visible) f.scale.setScalar(0.85 + Math.random() * 0.3);
    });

    // ω、v 向量（原點在質心，長度適度誇張化以利觀察）
    const cgI = V.add(s.r, Q.rotateToInertial(q, sim.cfg.vehicle.cg));
    const wI = Q.rotateToInertial(q, s.w);
    const wEnd = V.add(cgI, V.scale(wI, 2.2));
    this.omegaLine.geometry.setFromPoints([new THREE.Vector3(...cgI), new THREE.Vector3(...wEnd)]);
    const vEnd = V.add(cgI, V.scale(s.v, 1.0));
    this.velLine.geometry.setFromPoints([new THREE.Vector3(...cgI), new THREE.Vector3(...vEnd)]);

    // 軌跡（每 0.1s 加一點）
    if (!this._lastTrajT || sim.t - this._lastTrajT > 0.1) {
      this._lastTrajT = sim.t;
      if (this.trajCount < 6000) {
        this.trajPositions[this.trajCount * 3] = s.r[0];
        this.trajPositions[this.trajCount * 3 + 1] = s.r[1];
        this.trajPositions[this.trajCount * 3 + 2] = s.r[2];
        this.trajCount++;
        this.trajLine.geometry.setDrawRange(0, this.trajCount);
        this.trajLine.geometry.attributes.position.needsUpdate = true;
      }
    }

    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  resetTrajectory() {
    this.trajCount = 0;
    this.trajLine.geometry.setDrawRange(0, 0);
    this._lastTrajT = 0;
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }
}

// 由 +X 方向向 dir 的旋轉四元數
function quaternionFromX(dir) {
  const d = V.norm(dir) || 1;
  const n = [dir[0] / d, dir[1] / d, dir[2] / d];
  const xAxis = new THREE.Vector3(1, 0, 0);
  const target = new THREE.Vector3(...n);
  if (Math.abs(target.x - 1) < 1e-9 && Math.abs(target.y) < 1e-9 && Math.abs(target.z) < 1e-9)
    return new THREE.Quaternion();
  const q = new THREE.Quaternion();
  q.setFromUnitVectors(xAxis, target);
  return q;
}
