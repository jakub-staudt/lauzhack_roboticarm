'use strict';
// Read-only 3D-ish diagram of the arm drawn from the measured joints (no extra bandwidth: it reuses /api/state).
// Link transforms are the SO-101 body offsets from soarm/robot/so101.xml; every hinge turns about its local Z axis.
const PosePreview = (() => {
  const h = Math.SQRT1_2;
  const JOINTS = [
    ['shoulder_pan', [0.0388353, 0, 0.0624], [0, 0, -1, 0]],
    ['shoulder_lift', [-0.0303992, -0.0182778, -0.0542], [0.5, -0.5, -0.5, -0.5]],
    ['elbow_flex', [-0.11257, -0.028, 0], [h, 0, 0, h]],
    ['wrist_flex', [-0.1349, 0.0052, 0], [h, 0, 0, -h]],
    ['wrist_roll', [0, -0.0611, 0.0181], [0.0172091082, -0.0172091082, 0.7068973381, 0.7068973381]],
    ['gripper', [0.0202, 0.0188, -0.0234], [h, h, 0, 0]],
  ];
  const [CLOSED, OPEN] = [-0.0951068, 0.9541361];  // gripper 0..100 -> jaw angle [rad], from robot.yaml

  const add = ([a, b, c], [x, y, z]) => [a + x, b + y, c + z];
  const mul = ([a, b, c, d], [w, x, y, z]) =>
    [a * w - b * x - c * y - d * z, a * x + b * w + c * z - d * y, a * y - b * z + c * w + d * x, a * z + b * y - c * x + d * w];
  const at = ({origin, rotation}, p) => {
    const [w, a, b, c] = rotation;
    return add(origin, mul(mul(rotation, [0, ...p]), [w, -a, -b, -c]).slice(1));
  };

  function geometry(m) {
    if (!m || !JOINTS.every(([n]) => Number.isFinite(m[n]))) return null;
    let parent = {origin: [0, 0, 0], rotation: [1, 0, 0, 0]};
    const chain = JOINTS.map(([name, position, rotation]) => {
      const angle = name === 'gripper' ? CLOSED + (OPEN - CLOSED) * Math.max(0, Math.min(100, m.gripper)) / 100
                                       : m[name] * Math.PI / 180;
      parent = {origin: at(parent, position), rotation: mul(mul(parent.rotation, rotation), [Math.cos(angle / 2), 0, 0, Math.sin(angle / 2)])};
      return parent;
    });
    const [, shoulder, elbow, wrist, roll, jaw] = chain;
    return {anchors: [shoulder, elbow, wrist, roll].map(f => f.origin), fixedJaw: at(roll, [-0.0081, 0, -0.101]),
            jawPivot: jaw.origin, movingJaw: at(jaw, [-0.012, -0.078, 0.0192]), camera: at(roll, [-0.001, 0.06, -0.04])};
  }

  function create(canvas) {
    const ctx = canvas.getContext('2d'); let last;
    function draw(measured, tableZ = 0) {
      const w = canvas.clientWidth, hgt = canvas.clientHeight, pose = geometry(measured);
      if (!ctx || !w || !hgt || !pose) return false;
      const key = JSON.stringify([w, hgt, tableZ, JOINTS.map(([n]) => measured[n])]);
      if (key === last) return true;
      last = key;
      const r = Math.min(window.devicePixelRatio || 1, 2);
      canvas.width = Math.round(w * r); canvas.height = Math.round(hgt * r);
      ctx.setTransform(r, 0, 0, r, 0, 0);
      ctx.fillStyle = '#13181b'; ctx.fillRect(0, 0, w, hgt);
      ctx.lineCap = ctx.lineJoin = 'round';
      const scale = Math.min(w / 0.65, hgt / 0.49);
      const P = ([x, y, z]) => [w * 0.52 + scale * (-0.71 * x + 0.71 * y), hgt * 0.66 + scale * (0.29 * x + 0.29 * y - 0.91 * (z - tableZ))];
      const link = (a, b, color, weight) => { ctx.strokeStyle = color; ctx.lineWidth = weight; ctx.beginPath(); ctx.moveTo(...P(a)); ctx.lineTo(...P(b)); ctx.stroke(); };
      const dot = (p, color, rad) => { ctx.fillStyle = color; ctx.beginPath(); ctx.arc(...P(p), rad, 0, 2 * Math.PI); ctx.fill(); };
      for (let i = -3; i <= 3; ++i) {  // table grid, 10 cm squares
        const g = i / 10, c = i === 0 ? '#485359' : '#2e393d';
        link([-0.3, g, tableZ], [0.3, g, tableZ], c, 1); link([g, -0.3, tableZ], [g, 0.3, tableZ], c, 1);
      }
      const size = Math.max(0.6, scale / 400), [shoulder, elbow, wrist, roll] = pose.anchors, base = [0.0388353, 0, tableZ + 0.014];
      dot(base, '#58686d', 14 * size);
      for (const [a, b, wd] of [[base, shoulder, 17], [shoulder, elbow, 14], [elbow, wrist, 12], [wrist, roll, 10],
                                 [roll, pose.fixedJaw, 7], [roll, pose.jawPivot, 7], [pose.jawPivot, pose.movingJaw, 6]]) {
        link(a, b, '#766529', (wd + 3) * size); link(a, b, '#ffd84a', wd * size);
      }
      for (const a of pose.anchors) dot(a, '#a78c34', 4 * size);
      link(roll, pose.camera, '#75878c', 4 * size); dot(pose.camera, '#e0eef0', 4 * size);
      return true;
    }
    return {draw};
  }
  return {create, geometry};
})();
