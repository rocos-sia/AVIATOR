#include <pin_ik/pin_ik.hpp>
#include <iostream>

int main() {
    std::string urdf = R"(
<robot name="standalone_test">
  <link name="base"/><link name="x"/><link name="y"/><link name="z"/>
  <link name="roll"/><link name="pitch"/><link name="yaw"/><link name="tip"/>
  <joint name="x" type="prismatic"><parent link="base"/><child link="x"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="2"/><safety_controller soft_lower_limit="-0.8" soft_upper_limit="0.8" k_velocity="1"/></joint>
  <joint name="y" type="prismatic"><parent link="x"/><child link="y"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="10" velocity="2"/></joint>
  <joint name="z" type="prismatic"><parent link="y"/><child link="z"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="10" velocity="2"/></joint>
  <joint name="roll" type="revolute"><parent link="z"/><child link="roll"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="2"/></joint>
  <joint name="pitch" type="revolute"><parent link="roll"/><child link="pitch"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="10" velocity="2"/></joint>
  <joint name="yaw" type="continuous"><parent link="pitch"/><child link="yaw"/><axis xyz="0 0 1"/></joint>
  <joint name="tool" type="fixed"><parent link="yaw"/><child link="tip"/><origin xyz="0.1 0.2 0.3" rpy="0.1 0.2 0.3"/></joint>
</robot>
)";

    try {
        PIN_IK::PIN_IK solver("base", "tip", urdf, 0.03);

        pinocchio::Model model;
        Eigen::VectorXd lo, hi;
        solver.getModel(model);
        solver.getLimits(lo, hi);

        std::cout << "Model nq: " << model.nq << std::endl;
        std::cout << "Model njoints: " << model.njoints << std::endl;
        std::cout << "Limits size: " << lo.size() << std::endl;

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
