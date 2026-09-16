// Run from the repository root: node tests/aviator_hands.mjs
// Uses the checked-in MuJoCo WASM build; no npm installation is required.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { pathToFileURL } from 'node:url';

const root = path.resolve(import.meta.dirname, '..');
const wasmDir = path.join(root, 'reference/rocos-mujoco/third_party/mujoco-3.4.0/wasm');
const { default: loadMujoco } = await import(pathToFileURL(path.join(wasmDir, 'mujoco_wasm.js')));
const mj = await loadMujoco({ wasmBinary: fs.readFileSync(path.join(wasmDir, 'mujoco_wasm.wasm')) });
const xml = fs.readFileSync(path.join(root, 'mjcf/aviator.xml'), 'utf8');
mj.FS.mkdirTree('/model/mjcf');
mj.FS.writeFile('/model/mjcf/aviator.xml', xml);
for (const match of xml.matchAll(/<mesh\b[^>]*\bfile="([^"]+)"/g)) {
  const relative = path.posix.join('meshes', match[1]);
  mj.FS.mkdirTree(path.posix.dirname('/model/' + relative));
  mj.FS.writeFile('/model/' + relative, fs.readFileSync(path.join(root, relative)));
}
const model = mj.MjModel.mj_loadXML('/model/mjcf/aviator.xml');
const data = new mj.MjData(model);
const id = (kind, name) => {
  const result = mj.mj_name2id(model, kind, name);
  assert.ok(result >= 0, `Missing ${name}`);
  return result;
};
const near = (actual, expected, tolerance = 1e-10) => {
  assert.equal(actual.length, expected.length);
  actual.forEach((value, i) => assert.ok(Math.abs(value - expected[i]) < tolerance,
    `${value} differs from ${expected[i]} at ${i}`));
};
// mjtObj: body=1, joint=3, site=6, equality=17, key=24.
for (const [side, letter, base] of [['left', 'L', 'l_base_link'], ['right', 'R', 'r_base_link']]) {
  const hand = id(1, base);
  const flange = id(1, `AR5-5_07${letter}-W4C4A2_flan_link`);
  assert.equal(model.body_parentid[hand], flange, `${side} hand on wrong flange`);
  near(Array.from(model.body_pos.slice(3 * hand, 3 * hand + 3)), [0, 0, -0.018]);
  near(Array.from(model.body_quat.slice(4 * hand, 4 * hand + 4)), [1, 0, 0, 0]);
  id(6, `${side}_tcp`);
  assert.equal(model.eq_active0[id(17, `${side}_grasp`)], 0, 'Grasp weld must stay inactive');
  for (const finger of ['thumb', 'index', 'middle', 'ring', 'little']) {
    for (let segment = 1; segment <= (finger === 'thumb' ? 4 : 2); segment++) {
      id(3, `${side}_${finger}_${segment}_joint`);
    }
  }
}
assert.equal(model.nq, 40, 'Expected 2 wheel + 14 arm + 24 hand coordinates');
assert.equal(model.neq, 14, 'Expected 12 mimic equalities + 2 grasp welds');
mj.mj_resetDataKeyframe(model, data, id(24, 'aviator_home'));
mj.mj_forward(model, data);
for (const [side, letter] of [['left', 'L'], ['right', 'R']]) {
  const flange = id(1, `AR5-5_07${letter}-W4C4A2_flan_link`);
  const tcp = id(6, `${side}_tcp`);
  const expected = [0, 1, 2].map(i => data.xpos[3 * flange + i]
    + 0.1 * data.xmat[9 * flange + 3 * i + 2]);
  near(Array.from(data.site_xpos.slice(3 * tcp, 3 * tcp + 3)), expected);
  // The legacy TCP rotates +90 degrees about the flange's Y axis.
  const expectedRotation = [];
  for (let row = 0; row < 3; row++) {
    const offset = 9 * flange + 3 * row;
    expectedRotation.push(-data.xmat[offset + 2], data.xmat[offset + 1], data.xmat[offset]);
  }
  near(Array.from(data.site_xmat.slice(9 * tcp, 9 * tcp + 9)), expectedRotation);
}
const armHome = {
  L: [1.94771630354, 1.57079632679, -1.37618323285, 2.04981289676, 0.156860282066, 0.0729290021534, 0.156918313879],
  R: [-1.84860741719, 1.57079632679, 1.11467509058, 1.93698015773, -0.180011079028, 0.0687414348287, -0.693679354889],
};
for (const [letter, positions] of Object.entries(armHome)) {
  positions.forEach((value, index) => {
    const joint = id(3, `AR5-5_07${letter}-W4C4A2_joint_${index + 1}`);
    near([data.qpos[model.jnt_qposadr[joint]]], [value]);
  });
}
for (const side of ['left', 'right']) {
  const setJoint = (finger, segment, value) => {
    const joint = id(3, `${side}_${finger}_${segment}_joint`);
    assert.equal(data.qpos[model.jnt_qposadr[joint]], 0, 'Hand home must be zero');
    data.qpos[model.jnt_qposadr[joint]] = value;
  };
  setJoint('thumb', 1, 0.3);
  setJoint('thumb', 2, 0.2);
  setJoint('thumb', 3, 0.2 * 0.8392);
  setJoint('thumb', 4, 0.2 * 0.8392 * 0.891);
  // Distinct finger angles also catch mimic constraints wired to the wrong finger.
  for (const [finger, angle] of [['index', 0.25], ['middle', 0.35], ['ring', 0.45], ['little', 0.55]]) {
    setJoint(finger, 1, angle);
    setJoint(finger, 2, angle * 1.0843);
  }
}
mj.mj_forward(model, data);
// At an independently chosen articulated pose, every active equality
// should be satisfied. This catches wrong mimic direction or multiplier.
let equalityRows = 0;
for (let i = 0; i < data.nefc; i++) {
  if (data.efc_type[i] === 0) {
    equalityRows++;
    assert.ok(Math.abs(data.efc_pos[i]) < 1e-10,
      `Unsatisfied mimic equality: ${data.efc_pos[i]}`);
  }
}
assert.equal(equalityRows, 12, 'All hand mimic constraints must be active');
for (let i = 0; i < 100; i++) mj.mj_step(model, data);
assert.ok(Array.from(data.qpos).every(Number.isFinite), 'Non-finite position');
assert.ok(Array.from(data.qvel).every(Number.isFinite), 'Non-finite velocity');
for (let i = 0; i < data.warning.size(); i++) {
  assert.equal(data.warning.get(i).number, 0, `MuJoCo warning ${i}`);
}
// Verify actual following: set only independent finger joints, leave every
// follower at zero, then let dynamics enforce the equality constraints.
// Remove gravity and contacts to isolate coupling from external forces.
model.opt.gravity.fill(0);
model.geom_contype.fill(0);
model.geom_conaffinity.fill(0);
mj.mj_resetDataKeyframe(model, data, id(24, 'aviator_home'));
const jointAddress = name => model.jnt_qposadr[id(3, name)];
const couplings = [];
for (const side of ['left', 'right']) {
  data.qpos[jointAddress(`${side}_thumb_2_joint`)] = 0.2;
  couplings.push([`${side}_thumb_3_joint`, `${side}_thumb_2_joint`, 0.8392]);
  couplings.push([`${side}_thumb_4_joint`, `${side}_thumb_3_joint`, 0.891]);
  for (const [finger, angle] of [['index', 0.25], ['middle', 0.35], ['ring', 0.45], ['little', 0.55]]) {
    data.qpos[jointAddress(`${side}_${finger}_1_joint`)] = angle;
    couplings.push([`${side}_${finger}_2_joint`, `${side}_${finger}_1_joint`, 1.0843]);
  }
}
mj.mj_forward(model, data);
// Paused viewer sliders and mj_forward do not project qpos onto constraints.
for (const [follower] of couplings) assert.equal(data.qpos[jointAddress(follower)], 0);
for (let i = 0; i < 500; i++) mj.mj_step(model, data);
let maxCouplingError = 0;
for (const [follower, driver, multiplier] of couplings) {
  const followerAngle = data.qpos[jointAddress(follower)];
  assert.ok(followerAngle > 0.01, `${follower} did not follow its driver`);
  const error = Math.abs(followerAngle - multiplier * data.qpos[jointAddress(driver)]);
  maxCouplingError = Math.max(maxCouplingError, error);
  assert.ok(error < 1e-6, `${follower} coupling error ${error}`);
}
console.log(`PASS: MuJoCo loaded ${model.nbody} bodies, ${model.nq} coordinates, ${model.neq} equalities; connections, home, mimic pose and 100 steps verified. All 12 followers moved after changing drivers only; maximum error after 500 isolated steps: ${maxCouplingError.toExponential(3)} rad.`);
data.delete();
model.delete();
