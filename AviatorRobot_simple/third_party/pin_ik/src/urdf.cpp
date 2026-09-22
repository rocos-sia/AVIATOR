#include <pin_ik/urdf.hpp>
#include <algorithm>
#include <limits>

namespace PIN_IK {

void loadURDFModel(const std::string& urdf_xml, const std::string& base,
                   const std::string& tip, Model& model,
                   Eigen::VectorXd& lower, Eigen::VectorXd& upper,
                   FrameIndex& tip_frame_id) {
  Model full;
  pinocchio::urdf::buildModelFromXML(urdf_xml, full);
  if (!full.existFrame(base, pinocchio::BODY) || !full.existFrame(tip, pinocchio::BODY))
    throw std::invalid_argument("Base or tip link not found in URDF");
  const FrameIndex base_id = full.getFrameId(base, pinocchio::BODY);
  const FrameIndex tip_id = full.getFrameId(tip, pinocchio::BODY);

  // Follow frames, not just joints: fixed links can share a parent joint.
  std::vector<FrameIndex> frames;
  for (FrameIndex f = tip_id; f != base_id; f = full.frames[f].parentFrame) {
    if (f == 0) throw std::invalid_argument("Base is not an ancestor of tip");
    frames.push_back(f);
  }
  std::reverse(frames.begin(), frames.end());
  const JointIndex base_joint = full.frames[base_id].parentJoint;
  std::vector<JointIndex> joints;
  for (JointIndex j = full.frames[tip_id].parentJoint; j != base_joint; j = full.parents[j]) {
    if (j == 0) throw std::invalid_argument("Base is not an ancestor of tip");
    joints.push_back(j);
  }
  std::reverse(joints.begin(), joints.end());
  if (joints.empty()) throw std::invalid_argument("Chain has no movable joints");

  model = Model();
  model.name = full.name;
  model.addBodyFrame(base, 0, SE3::Identity());
  std::vector<JointIndex> mapping(full.joints.size(), 0);
  JointIndex parent = 0;
  for (JointIndex j : joints) {
    SE3 placement = full.jointPlacements[j];
    if (parent == 0) placement = full.frames[base_id].placement.actInv(placement);
    parent = model.addJoint(parent, full.joints[j], placement, full.names[j],
                           full.effortLimit.segment(full.joints[j].idx_v(), full.joints[j].nv()),
                           full.velocityLimit.segment(full.joints[j].idx_v(), full.joints[j].nv()),
                           full.lowerPositionLimit.segment(full.joints[j].idx_q(), full.joints[j].nq()),
                           full.upperPositionLimit.segment(full.joints[j].idx_q(), full.joints[j].nq()));
    model.addJointFrame(parent);
    mapping[j] = parent;
  }
  for (FrameIndex f : frames) {
    const auto& frame = full.frames[f];
    if (frame.type != pinocchio::BODY) continue;
    SE3 placement = frame.placement;
    if (frame.parentJoint == base_joint)
      placement = full.frames[base_id].placement.actInv(placement);
    model.addBodyFrame(frame.name, mapping[frame.parentJoint], placement);
  }
  tip_frame_id = model.getFrameId(tip, pinocchio::BODY);
  const auto types = jointTypes(model);
  lower.resize(model.nv);
  upper.resize(model.nv);
  for (JointIndex j = 1; j < model.joints.size(); ++j) {
    const auto& joint = model.joints[j];
    const int v = joint.idx_v();
    lower[v] = types[v] == Continuous ? -std::numeric_limits<double>::infinity()
                                    : model.lowerPositionLimit[joint.idx_q()];
    upper[v] = types[v] == Continuous ? std::numeric_limits<double>::infinity()
                                    : model.upperPositionLimit[joint.idx_q()];
  }
}

} // namespace PIN_IK
