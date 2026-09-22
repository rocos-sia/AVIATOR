#include <pin_ik/pin_ik.hpp>
#include <pin_ik/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <random>

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

template<class F> void rejects(F fn) {
  try { fn(); } catch (const std::invalid_argument&) { return; }
  throw std::runtime_error("Expected invalid_argument");
}

int main(int argc, char** argv) {
  try {
    require(argc == 2, "Missing URDF path");
    std::ifstream file(argv[1]);
    std::string xml((std::istreambuf_iterator<char>(file)), {});

    PIN_IK::PIN_IK parsed("base", "tip", xml, 0.03);
    pinocchio::Model model;
    Eigen::VectorXd lo, hi;
    require(parsed.getModel(model) && parsed.getLimits(lo, hi), "Initialization failed");
    require(model.nq == 7 && model.nv == 6, "Wrong configuration/tangent dimensions");
    require(lo.size() == 6 && hi.size() == 6, "Limits must use scalar joint coordinates");
    require(lo(0) == -1.0 && hi(0) == 1.0, "Joint limits missing");
    require(std::isinf(lo(5)) && lo(5) < 0 && std::isinf(hi(5)) && hi(5) > 0,
            "Continuous limits missing");

    pinocchio::Data data(model);
    pinocchio::FrameIndex tip_frame_id = model.getFrameId("tip", pinocchio::BODY);

    Eigen::VectorXd q(6), seed(6), out;
    q << 0.3, -0.2, 0.4, 0.2, -0.3, 0.7;
    seed.setZero();
    seed(5) = 8 * M_PI; // Preserve the seed's winding, not just [-pi, pi].

    pinocchio::forwardKinematics(model, data, PIN_IK::toPinocchioConfiguration(model, q));
    pinocchio::updateFramePlacements(model, data);
    pinocchio::SE3 goal = data.oMf[tip_frame_id];

    // Build expected transform analytically
    Eigen::Matrix3d R_x, R_y, R_z;
    R_x = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX());
    R_y = Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY());
    R_z = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotation = R_x * R_y * R_z;

    Eigen::Matrix3d R_rpy;
    R_rpy = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());

    Eigen::Vector3d translation = q.head<3>() + rotation * Eigen::Vector3d(0.1, 0.2, 0.3);
    pinocchio::SE3 expected(rotation * R_rpy, translation);

    require(goal.isApprox(expected, 1e-10), "URDF FK disagrees with analytic transform");

    for (auto mode : {PIN_IK::Speed, PIN_IK::Distance, PIN_IK::Manip1, PIN_IK::Manip2, PIN_IK::Manip3}) {
      PIN_IK::PIN_IK solver(model, lo, hi, tip_frame_id, 0.03, 1e-6, mode);
      require(solver.CartToJnt(seed, goal, out) >= 0, "IK failed");

      pinocchio::forwardKinematics(model, data, PIN_IK::toPinocchioConfiguration(model, out));
      pinocchio::updateFramePlacements(model, data);
      pinocchio::SE3 actual = data.oMf[tip_frame_id];

      require(actual.isApprox(goal, 1e-5), "IK residual too large");
      for (int i = 0; i < 6; ++i)
        require(out(i) >= lo(i)-1e-8 && out(i) <= hi(i)+1e-8, "Joint limit violation");

      require(std::abs(out(5) - seed(5)) <= M_PI, "Continuous solution not near seed");
      std::vector<Eigen::VectorXd> solutions;
      require(solver.getSolutions(solutions), "Solutions missing");
    }

    // Exercise both algorithms independently, including non-axis-aligned continuous joints.
    for (const auto& axis : {std::string("1 0 0"), std::string("0 1 0"),
                             std::string("0 0 1"), std::string("0.6 0.8 0")}) {
      const std::string continuous_xml =
        "<robot name='continuous'><link name='base'/><link name='tip'/>"
        "<joint name='spin' type='continuous'><parent link='base'/><child link='tip'/>"
        "<axis xyz='" + axis + "'/></joint></robot>";
      pinocchio::Model cm;
      Eigen::VectorXd cl, cu;
      pinocchio::FrameIndex cf;
      PIN_IK::loadURDFModel(continuous_xml, "base", "tip", cm, cl, cu, cf);
      require(cm.nq == 2 && cm.nv == 1, "Continuous joint dimensions");
      Eigen::VectorXd angle(1), initial(1), result;
      angle[0] = -M_PI + 0.04;
      initial[0] = 10 * M_PI + M_PI - 0.04;
      pinocchio::Data cd(cm);
      const auto cq = PIN_IK::toPinocchioConfiguration(cm, angle);
      require(std::abs(cq.norm() - 1) < 1e-12, "Unit circle conversion");
      pinocchio::forwardKinematics(cm, cd, cq);
      pinocchio::updateFramePlacements(cm, cd);
      const auto target = cd.oMf[cf];
      PIN_IK::ChainIkSolverPos_TL newton(cm, cl, cu, cf, 0.1, 1e-6);
      NLOPT_IK::NLOPT_IK nlopt(cm, cl, cu, cf, 0.1, 1e-6);
      require(newton.CartToJnt(initial, target, result) >= 0, "Continuous Newton IK");
      require(std::abs(result[0] - initial[0] - 0.08) < 1e-5, "Newton winding");
      require(nlopt.CartToJnt(initial, target, result) >= 0, "Continuous NLopt IK");
      pinocchio::forwardKinematics(cm, cd, PIN_IK::toPinocchioConfiguration(cm, result));
      pinocchio::updateFramePlacements(cm, cd);
      require(cd.oMf[cf].isApprox(target, 1e-5), "Continuous NLopt residual");
      PIN_IK::PIN_IK combined(cm, cl, cu, cf, 0.1, 1e-6);
      require(combined.CartToJnt(initial, target, result) >= 0, "Continuous combined IK");
      require(std::abs(result[0] - initial[0] - 0.08) < 1e-5, "Combined winding");
      Eigen::VectorXd invalid = cl;
      invalid[0] = 0;
      rejects([&]{ combined.setLimits(invalid, cu); });
      require(combined.setLimits(cl, cu), "Continuous setLimits");
    }

    // Non-root base: remove upstream joints and express the goal in the base frame.
    PIN_IK::PIN_IK subchain("pitch", "tip", xml, 0.1);
    pinocchio::Model submodel;
    subchain.getModel(submodel);
    require(submodel.nv == 1 && submodel.nq == 2, "Subchain includes upstream joints");
    pinocchio::Data subdata(submodel);
    Eigen::VectorXd subq(1);
    subq[0] = q[5];
    pinocchio::forwardKinematics(submodel, subdata, PIN_IK::toPinocchioConfiguration(submodel, subq));
    pinocchio::updateFramePlacements(submodel, subdata);
    pinocchio::forwardKinematics(model, data, PIN_IK::toPinocchioConfiguration(model, q));
    pinocchio::updateFramePlacements(model, data);
    require(subdata.oMf[submodel.getFrameId("tip", pinocchio::BODY)].isApprox(
      data.oMf[model.getFrameId("pitch", pinocchio::BODY)].actInv(data.oMf[tip_frame_id]), 1e-10),
      "Subchain reference frame");

    // A fixed, offset base and an unrelated branch must not leak into the IK chain.
    const std::string branched = R"(<robot name="branched">
      <link name="root"/><link name="mount"/><link name="base"/><link name="a"/>
      <link name="b"/><link name="tip"/><link name="other"/>
      <joint name="upstream" type="continuous"><parent link="root"/><child link="mount"/></joint>
      <joint name="offset" type="fixed"><parent link="mount"/><child link="base"/>
        <origin xyz="0.2 0.3 0.4" rpy="0.1 0.2 0.3"/></joint>
      <joint name="first" type="continuous"><parent link="base"/><child link="a"/><axis xyz="1 0 0"/></joint>
      <joint name="slide" type="prismatic"><parent link="a"/><child link="b"/>
        <axis xyz="0.6 0.8 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
      <joint name="last" type="continuous"><parent link="b"/><child link="tip"/><axis xyz="0 0 1"/></joint>
      <joint name="branch" type="continuous"><parent link="root"/><child link="other"/></joint>
      </robot>)";
    pinocchio::Model chain, full;
    Eigen::VectorXd bl, bu;
    pinocchio::FrameIndex bf;
    PIN_IK::loadURDFModel(branched, "base", "tip", chain, bl, bu, bf);
    require(chain.nv == 3 && chain.nq == 5, "Mixed continuous dimensions");
    require(bl.size() == 3 && bl[1] == -1 && bu[1] == 1, "Mixed limit indexing");
    Eigen::VectorXd bq(3);
    bq << 0.3, 0.4, -0.7;
    const auto native = PIN_IK::toPinocchioConfiguration(chain, bq);
    require(std::abs(native[2] - 0.4) < 1e-12 &&
            std::abs(native[3] - std::cos(-0.7)) < 1e-12, "Mixed configuration indexing");
    pinocchio::urdf::buildModelFromXML(branched, full);
    Eigen::VectorXd fq = Eigen::VectorXd::Zero(full.nv);
    fq[full.joints[full.getJointId("first")].idx_v()] = bq[0];
    fq[full.joints[full.getJointId("slide")].idx_v()] = bq[1];
    fq[full.joints[full.getJointId("last")].idx_v()] = bq[2];
    fq[full.joints[full.getJointId("upstream")].idx_v()] = 0.9;
    pinocchio::Data fd(full), bd(chain);
    pinocchio::forwardKinematics(full, fd, PIN_IK::toPinocchioConfiguration(full, fq));
    pinocchio::updateFramePlacements(full, fd);
    pinocchio::forwardKinematics(chain, bd, native);
    pinocchio::updateFramePlacements(chain, bd);
    const auto bgoal = bd.oMf[bf];
    require(bgoal.isApprox(fd.oMf[full.getFrameId("base", pinocchio::BODY)].actInv(
      fd.oMf[full.getFrameId("tip", pinocchio::BODY)]), 1e-10), "Fixed-base extraction");
    PIN_IK::PIN_IK bsolver(chain, bl, bu, bf, 0.1, 1e-6);
    Eigen::VectorXd bseed = Eigen::VectorXd::Zero(3), bout;
    bseed[0] = 4 * M_PI;
    bseed[2] = -6 * M_PI;
    require(bsolver.CartToJnt(bseed, bgoal, bout) >= 0, "Mixed continuous IK");
    pinocchio::forwardKinematics(chain, bd, PIN_IK::toPinocchioConfiguration(chain, bout));
    pinocchio::updateFramePlacements(chain, bd);
    require(bd.oMf[bf].isApprox(bgoal, 1e-5), "Mixed continuous FK residual");
    rejects([&]{ PIN_IK::PIN_IK bad("other", "tip", branched); });

    // Test a 6R arm
    pinocchio::Model arm_model;
    arm_model.name = "6R_arm";

    pinocchio::JointIndex parent_id = 0;
    for (int i = 0; i < 6; ++i) {
      std::string joint_name = "joint_" + std::to_string(i);
      std::string body_name = "body_" + std::to_string(i);

      pinocchio::SE3 placement = pinocchio::SE3::Identity();
      placement.translation() = Eigen::Vector3d(0.15, 0, 0.1);

      pinocchio::Inertia inertia = pinocchio::Inertia::Random();

      if (i % 3 == 0)
        parent_id = arm_model.addJoint(parent_id, pinocchio::JointModelRZ(), placement, joint_name);
      else if (i % 3 == 1)
        parent_id = arm_model.addJoint(parent_id, pinocchio::JointModelRY(), placement, joint_name);
      else
        parent_id = arm_model.addJoint(parent_id, pinocchio::JointModelRX(), placement, joint_name);

      arm_model.addJointFrame(parent_id);
      arm_model.appendBodyToJoint(parent_id, inertia, pinocchio::SE3::Identity());
      arm_model.addBodyFrame(body_name, parent_id, pinocchio::SE3::Identity());
    }

    arm_model.addFrame(pinocchio::Frame("tip", parent_id, 0, pinocchio::SE3::Identity(), pinocchio::BODY));

    Eigen::VectorXd arm_lo(6), arm_hi(6), arm_q(6), arm_seed(6), arm_out;
    arm_lo.setConstant(-2.5);
    arm_hi.setConstant(2.5);

    pinocchio::FrameIndex arm_tip_id = arm_model.getFrameId("tip", pinocchio::BODY);
    PIN_IK::PIN_IK arm_solver(arm_model, arm_lo, arm_hi, arm_tip_id, 0.05, 1e-6);
    pinocchio::Data arm_data(arm_model);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> sample(-1.5, 1.5);

    for (int trial = 0; trial < 100; ++trial) {
      for (int i = 0; i < 6; ++i) {
        arm_q(i) = sample(rng);
        arm_seed(i) = arm_q(i) + 0.1;
      }

      pinocchio::forwardKinematics(arm_model, arm_data, arm_q);
      pinocchio::updateFramePlacements(arm_model, arm_data);
      pinocchio::SE3 arm_goal = arm_data.oMf[arm_tip_id];

      require(arm_solver.CartToJnt(arm_seed, arm_goal, arm_out) >= 0, "6R arm IK failed");

      pinocchio::forwardKinematics(arm_model, arm_data, arm_out);
      pinocchio::updateFramePlacements(arm_model, arm_data);
      pinocchio::SE3 arm_actual = arm_data.oMf[arm_tip_id];

      require(arm_goal.isApprox(arm_actual, 1e-5), "6R arm FK residual");
      for (int i = 0; i < 6; ++i)
        require(arm_out(i) >= -2.5 && arm_out(i) <= 2.5, "6R limits");
    }

    // Test rotated joint
    const std::string rotated = R"(<robot name="rotated"><link name="base"/><link name="tip"/>
      <joint name="joint" type="revolute"><parent link="base"/><child link="tip"/>
      <origin xyz="0.2 -0.1 0.4" rpy="0.3 -0.2 0.7"/><axis xyz="0 1 0"/>
      <limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>)";

    pinocchio::Model rotated_model;
    Eigen::VectorXd rotated_lo, rotated_hi;
    pinocchio::FrameIndex rotated_tip_id;
    PIN_IK::loadURDFModel(rotated, "base", "tip", rotated_model, rotated_lo, rotated_hi, rotated_tip_id);

    Eigen::VectorXd angle(1);
    angle(0) = 0.6;
    pinocchio::Data rotated_data(rotated_model);
    pinocchio::forwardKinematics(rotated_model, rotated_data, angle);
    pinocchio::updateFramePlacements(rotated_model, rotated_data);
    pinocchio::SE3 rotated_actual = rotated_data.oMf[rotated_tip_id];

    Eigen::Matrix3d expected_rot;
    expected_rot = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()) *
                   Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitY());
    pinocchio::SE3 rotated_expected(expected_rot, Eigen::Vector3d(0.2, -0.1, 0.4));

    require(rotated_actual.isApprox(rotated_expected, 1e-10), "Rotated URDF joint transform");

    rejects([&]{ Eigen::VectorXd wrong(1); parsed.setLimits(wrong, hi); });
    require(parsed.CartToJnt(Eigen::VectorXd(2), goal, out) < 0, "Wrong seed accepted");

    auto begin = std::chrono::steady_clock::now();
    pinocchio::SE3 unreachable(Eigen::Matrix3d::Identity(), Eigen::Vector3d(10, 10, 10));
    require(parsed.CartToJnt(seed, unreachable, out) < 0, "Unreachable target accepted");
    require(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count() < 1, "Timeout broken");

    rejects([&]{ PIN_IK::PIN_IK bad("base", "missing", xml); });
    rejects([&]{ PIN_IK::PIN_IK bad("tip", "base", xml); });
    rejects([&]{ PIN_IK::PIN_IK bad("base", "tip", "bad xml"); });
    rejects([&]{ PIN_IK::PIN_IK bad(model, Eigen::VectorXd(1), hi, tip_frame_id); });

    std::cout << "All five solve modes, 100 6R targets, analytic FK, limits, invalid inputs and timeout passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
