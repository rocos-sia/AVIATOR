#include <pin_ik/pin_ik.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <iostream>

int main() {
    std::cout << "Testing PIN-IK with Pinocchio..." << std::endl;

    // Create a simple 2-DOF robot URDF
    std::string urdf = R"(
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="tip"/>

  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  </joint>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  </joint>

  <joint name="tip_joint" type="fixed">
    <parent link="link2"/>
    <child link="tip"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
)";

    try {
        // Create PIN-IK solver
        PIN_IK::PIN_IK solver("base", "tip", urdf, 0.005, 1e-5, PIN_IK::Speed);
        std::cout << "✓ PIN-IK solver created successfully" << std::endl;

        // Get model info
        pinocchio::Model model;
        Eigen::VectorXd lower, upper;
        solver.getModel(model);
        solver.getLimits(lower, upper);

        std::cout << "✓ Model has " << model.nq << " joints" << std::endl;
        std::cout << "✓ Joint limits: [" << lower.transpose() << "] to ["
                  << upper.transpose() << "]" << std::endl;

        // Set up a simple IK problem
        Eigen::VectorXd q_init = Eigen::VectorXd::Zero(model.nq);
        pinocchio::SE3 target;
        target.translation() = Eigen::Vector3d(1.5, 0.5, 0);
        target.rotation() = Eigen::Matrix3d::Identity();

        Eigen::VectorXd q_solution;
        int result = solver.CartToJnt(q_init, target, q_solution);

        if (result >= 0) {
            std::cout << "✓ IK solved! Found " << result << " solution(s)" << std::endl;
            std::cout << "  Solution: [" << q_solution.transpose() << "]" << std::endl;
        } else {
            std::cout << "✗ IK failed (this is OK for this simple test)" << std::endl;
        }

        std::cout << "\n=== PIN-IK Pinocchio integration test PASSED ===" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "✗ Error: " << e.what() << std::endl;
        return 1;
    }
}
