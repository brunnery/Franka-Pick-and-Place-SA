// ---- FR3 kinematics (URDF parameters from franka_description, robots/fr3/kinematics.yaml) ----
const H = Math.PI / 2;
const JOINTS = [
  { p: [0, 0, 0.333], rpy: [0, 0, 0] },
  { p: [0, 0, 0], rpy: [-H, 0, 0] },
  { p: [0, -0.316, 0], rpy: [H, 0, 0] },
  { p: [0.0825, 0, 0], rpy: [H, 0, 0] },
  { p: [-0.0825, 0.384, 0], rpy: [-H, 0, 0] },
  { p: [0, 0, 0], rpy: [H, 0, 0] },
  { p: [0.088, 0, 0], rpy: [H, 0, 0] },
];
const Q_LO = [-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508];
const Q_HI = [2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508];
const Q_HOME = [0, -0.785, 0, -2.356, 0, 1.571, 0.785];
const FLANGE_Z = 0.107;            // link7 -> flange
const FLANGE_YAW = -Math.PI / 4;   // same mounting yaw as the Franka Hand
const GRIP_LEN = 0.1563;           // gripper flange face -> iris face (CAD)
const TCP_IN_GRIPPER = 0.0027;     // nut centre above the iris face (nut bottom 0.2 mm above the face)
const TCP_Z = GRIP_LEN - TCP_IN_GRIPPER;

function mmul(a, b) { const r = new Float64Array(16); for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) { let s = 0; for (let k = 0; k < 4; k++) s += a[i * 4 + k] * b[k * 4 + j]; r[i * 4 + j] = s; } return r; }
function mrpy(r, p, y, t) {
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  return Float64Array.from([cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, t[0], sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, t[1], -sp, cp * sr, cp * cr, t[2], 0, 0, 0, 1]);
}
const FIXED = JOINTS.map(j => mrpy(j.rpy[0], j.rpy[1], j.rpy[2], j.p));
const TOOL = mrpy(0, 0, FLANGE_YAW, [0, 0, FLANGE_Z]);
const TOOL_TCP = mmul(TOOL, mrpy(0, 0, 0, [0, 0, TCP_Z]));
function rotz(q) { const c = Math.cos(q), s = Math.sin(q); return Float64Array.from([c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]); }
// frames[i] = joint i+1 frame (after rotation), frames[7] = TCP
function fkAll(q, base) {
  let M = base; const fr = [];
  for (let i = 0; i < 7; i++) { M = mmul(mmul(M, FIXED[i]), rotz(q[i])); fr.push(M); }
  fr.push(mmul(M, TOOL_TCP));
  return fr;
}
function solve6(A, b) { // gaussian elimination, A 6x6 row-major
  const n = 6, M = A.slice(), x = b.slice();
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r * n + c]) > Math.abs(M[p * n + c])) p = r;
    if (p !== c) { for (let k = 0; k < n; k++) { const t = M[c * n + k]; M[c * n + k] = M[p * n + k]; M[p * n + k] = t; } const t = x[c]; x[c] = x[p]; x[p] = t; }
    const d = M[c * n + c];
    for (let r = c + 1; r < n; r++) { const f = M[r * n + c] / d; if (!f) continue; for (let k = c; k < n; k++) M[r * n + k] -= f * M[c * n + k]; x[r] -= f * x[c]; }
  }
  for (let r = n - 1; r >= 0; r--) { let s = x[r]; for (let k = r + 1; k < n; k++) s -= M[r * n + k] * x[k]; x[r] = s / M[r * n + r]; }
  return x;
}
// target: position p, tool z pointing down, yaw about world z
function ik(qInit, base, p, yaw, iters = 60, restGain = 0.05) {
  const q = qInit.slice();
  const Rt = mrpy(Math.PI, 0, 0, [0, 0, 0]); const Rd = mmul(mrpy(0, 0, yaw, [0, 0, 0]), Rt);
  let err = 1;
  for (let it = 0; it < iters; it++) {
    const fr = fkAll(q, base), E = fr[7];
    const ep = [p[0] - E[3], p[1] - E[7], p[2] - E[11]];
    // orientation error: 0.5 * sum(cross(R_e[:,i], R_d[:,i]))
    const eo = [0, 0, 0];
    for (let c = 0; c < 3; c++) {
      const a = [E[c], E[4 + c], E[8 + c]], d = [Rd[c], Rd[4 + c], Rd[8 + c]];
      eo[0] += 0.5 * (a[1] * d[2] - a[2] * d[1]); eo[1] += 0.5 * (a[2] * d[0] - a[0] * d[2]); eo[2] += 0.5 * (a[0] * d[1] - a[1] * d[0]);
    }
    const e = [...ep, ...eo];
    err = Math.hypot(...ep);
    const Jm = []; // 6x7
    for (let i = 0; i < 7; i++) {
      const F = fr[i], z = [F[2], F[6], F[10]], o = [F[3], F[7], F[11]], d = [E[3] - o[0], E[7] - o[1], E[11] - o[2]];
      Jm.push([z[1] * d[2] - z[2] * d[1], z[2] * d[0] - z[0] * d[2], z[0] * d[1] - z[1] * d[0], z[0], z[1], z[2]]);
    }
    const lam = 1e-4, A = new Float64Array(36);
    for (let r = 0; r < 6; r++) for (let c = 0; c < 6; c++) { let s = 0; for (let k = 0; k < 7; k++) s += Jm[k][r] * Jm[k][c]; A[r * 6 + c] = s + (r === c ? lam : 0); }
    const y = solve6(A, e);
    const dq = new Array(7).fill(0);
    for (let k = 0; k < 7; k++) for (let r = 0; r < 6; r++) dq[k] += Jm[k][r] * y[r];
    // nullspace pull toward home posture: (I - J+J) z
    const z0 = q.map((v, k) => restGain * (Q_HOME[k] - v));
    const Jz = [0, 0, 0, 0, 0, 0]; for (let r = 0; r < 6; r++) for (let k = 0; k < 7; k++) Jz[r] += Jm[k][r] * z0[k];
    const yz = solve6(A, Jz);
    for (let k = 0; k < 7; k++) { let s = 0; for (let r = 0; r < 6; r++) s += Jm[k][r] * yz[r]; dq[k] += z0[k] - s; }
    for (let k = 0; k < 7; k++) q[k] = Math.min(Q_HI[k] - 0.01, Math.max(Q_LO[k] + 0.01, q[k] + dq[k]));
  }
  return { q, err };
}
if (typeof module !== 'undefined') module.exports = { fkAll, ik, Q_HOME, Q_LO, Q_HI, mrpy };
