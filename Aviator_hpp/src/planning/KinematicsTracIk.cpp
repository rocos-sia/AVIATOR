#include "aviator/Kinematics.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <urdf_parser/urdf_parser.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace aviator {

namespace {

// Pinocchio SE3 <-> KDL Frame 转换
KDL::Frame se3ToFrame(const pinocchio::SE3 &se3) {
    const auto &R = se3.rotation();
    const auto &p = se3.translation();
    return KDL::Frame(KDL::Rotation(R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0),
                                     R(2, 1), R(2, 2)),
                      KDL::Vector(p.x(), p.y(), p.z()));
}

pinocchio::SE3 frameToSe3(const KDL::Frame &frame) {
    Eigen::Matrix3d R;
    R << frame.M.data[0], frame.M.data[1], frame.M.data[2], frame.M.data[3], frame.M.data[4],
        frame.M.data[5], frame.M.data[6], frame.M.data[7], frame.M.data[8];
    Eigen::Vector3d p(frame.p.x(), frame.p.y(), frame.p.z());
    return pinocchio::SE3(R, p);
}

} // namespace

// TRAC-IK 运动学求解器实现
//
// TRAC-IK (Track-IK) = KDL 数值 IK + SQP 优化 + 随机重启
//
// 与纯数值方法（如 KDL ChainIkSolverPos_LMA）相比:
// - 更高的成功率（通过 SQP 优化 + 多次随机重启）
// - 更快的求解速度（混合解析 + 数值优化）
// - 更好的冗余度处理（自动优化距离种子最近、manipulability 等）
//
// 优势:
// - 解析求解 + 数值优化混合，成功率和速度都优于纯数值方法
// - 自动处理冗余度优化 (距离种子最近、manipulability 等)
// - 久经考验的开源实现，被广泛用于工业和研究
class TracIkKinematics : public Kinematics {
  public:
    TracIkKinematics(const std::string &urdf_path, double joint2_min, double joint2_max)
        : joint2_min_(joint2_min), joint2_max_(joint2_max) {
        // 从 URDF 解析机器人模型
        urdf::Model model;
        if (!model.initFile(urdf_path))
            throw std::runtime_error("Failed to parse URDF: " + urdf_path);

        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(model, tree))
            throw std::runtime_error("Failed to construct KDL tree from URDF");

        // 为每条手臂提取运动链（aircraft -> flan_link）
        // 与 AviatorRobot 保持一致，从 aircraft 根节点开始
        // 注意：运动链会包含 roll_input_joint（飞机滚转），需要正确处理
        std::string base = "aircraft";
        std::array<std::string, 2> tips = {
            "AR5-5_07L-W4C4A2_flan_link",
            "AR5-5_07R-W4C4A2_flan_link"};

        for (int side = 0; side < 2; ++side) {
            KDL::Chain chain;
            if (!tree.getChain(base, tips[side], chain))
                throw std::runtime_error("Failed to extract chain: " + base + " -> " +
                                         tips[side]);

            // 调试：输出运动链信息
            std::cout << "[TracIK] Chain " << (side == 0 ? "LEFT" : "RIGHT") << ": "
                      << base << " -> " << tips[side] << "\n";
            std::cout << "  Segments: " << chain.getNrOfSegments() << "\n";
            std::cout << "  Total joints (all types): " << chain.getNrOfJoints() << "\n";

            int revolute_count = 0;
            for (size_t seg_idx = 0; seg_idx < chain.getNrOfSegments(); ++seg_idx) {
                const KDL::Segment &seg = chain.getSegment(seg_idx);
                const KDL::Joint &joint = seg.getJoint();
                if (joint.getType() != KDL::Joint::None) {
                    std::cout << "    Segment " << seg_idx << ": " << joint.getName()
                              << " (type=" << joint.getType() << ")\n";
                    if (joint.getType() == KDL::Joint::RotAxis ||
                        joint.getType() == KDL::Joint::RotX ||
                        joint.getType() == KDL::Joint::RotY ||
                        joint.getType() == KDL::Joint::RotZ) {
                        revolute_count++;
                    }
                }
            }
            std::cout << "  Revolute joints: " << revolute_count << "\n";

            // 提取关节限位（使用 getNrOfJoints() 而不是固定的 7）
            int num_joints = chain.getNrOfJoints();
            KDL::JntArray q_min(num_joints), q_max(num_joints);
            int joint_idx = 0;
            for (size_t seg_idx = 0; seg_idx < chain.getNrOfSegments(); ++seg_idx) {
                const KDL::Segment &seg = chain.getSegment(seg_idx);
                const KDL::Joint &joint = seg.getJoint();
                if (joint.getType() != KDL::Joint::None) {
                    // 从 URDF 获取限位
                    auto urdf_joint = model.getJoint(joint.getName());
                    if (urdf_joint && urdf_joint->limits) {
                        q_min(joint_idx) = urdf_joint->limits->lower;
                        q_max(joint_idx) = urdf_joint->limits->upper;
                        std::cout << "    Joint " << joint_idx << " (" << joint.getName()
                                  << "): [" << q_min(joint_idx) << ", " << q_max(joint_idx) << "]\n";
                    } else {
                        // 默认限位
                        q_min(joint_idx) = -M_PI;
                        q_max(joint_idx) = M_PI;
                        std::cout << "    Joint " << joint_idx << " (" << joint.getName()
                                  << "): using default limits\n";
                    }
                    joint_idx++;
                }
            }

            // 创建 TRAC-IK 求解器
            // 参数: chain, q_min, q_max, timeout(s), eps(m/rad), solve_type
            // solve_type: Speed (快速), Distance (距离种子近), Manip1/Manip2 (可操作性)
            constexpr double timeout = 0.05; // 50ms 超时（平衡精度和实时性）
            constexpr double eps = 1e-5;      // 高精度
            ik_solvers_[side].reset(
                new TRAC_IK::TRAC_IK(chain, q_min, q_max, timeout, eps, TRAC_IK::Distance));

            fk_solvers_[side].reset(new KDL::ChainFkSolverPos_recursive(chain));
            chains_[side] = chain;
        }
    }

    bool solveIk(Side side, const std::array<double, 7> &seed, const pinocchio::SE3 &target,
                 std::array<double, 7> &out) override {
        int idx = static_cast<int>(side);

        // 调试输出
        static int call_count = 0;
        bool debug = (call_count < 5); // 只输出前 5 次调用
        if (debug) {
            std::cout << "[TracIK::solveIk] Call #" << call_count << " for "
                      << (side == Side::Left ? "LEFT" : "RIGHT") << " arm\n";
            std::cout << "  Seed (7 arm joints): [";
            for (int i = 0; i < 7; ++i)
                std::cout << seed[i] << (i < 6 ? ", " : "]\n");
            std::cout << "  Target position: [" << target.translation().transpose() << "]\n";
            call_count++;
        }

        // 构建完整的种子配置（包含 roll_input_joint）
        // 运动链是 aircraft -> flan_link，包含 8 个关节：roll_input + 7个手臂关节
        int num_joints = chains_[idx].getNrOfJoints();
        KDL::JntArray q_init(num_joints);

        // 第一个关节是 roll_input_joint，设为 0（飞机不滚转）
        q_init(0) = 0.0;
        // 后面 7 个关节是手臂关节
        for (int i = 0; i < 7; ++i)
            q_init(i + 1) = seed[i];

        // 目标位姿（直接使用，因为运动链从 aircraft 开始）
        KDL::Frame target_frame = se3ToFrame(target);

        // 求解 IK
        KDL::JntArray q_out(num_joints);
        int result = ik_solvers_[idx]->CartToJnt(q_init, target_frame, q_out);

        if (debug) {
            std::cout << "  TracIK result: " << result << " (< 0 means failed)\n";
            if (result >= 0) {
                std::cout << "  Solution: [";
                for (int i = 0; i < num_joints; ++i)
                    std::cout << q_out(i) << (i < num_joints - 1 ? ", " : "]\n");
            }
        }

        if (result < 0) {
            if (debug) std::cout << "  FAILED: TracIK returned error\n";
            return false; // IK 失败
        }

        // 提取手臂关节（跳过第一个 roll_input_joint）
        // 检查 J2 是否在肘部下垂范围内（J2 在整体关节中是索引 2，在手臂关节中是索引 1）
        double j2_value = q_out(2); // roll_input(0) + j1(1) + j2(2) ...
        if (j2_value < joint2_min_ || j2_value > joint2_max_) {
            if (debug) {
                std::cout << "  FAILED: J2=" << j2_value << " out of range [" << joint2_min_
                          << ", " << joint2_max_ << "]\n";
            }
            return false;
        }

        if (debug) {
            std::cout << "  SUCCESS: J2=" << j2_value << " in range [" << joint2_min_ << ", "
                      << joint2_max_ << "]\n";
        }

        // 输出结果（只输出 7 个手臂关节，跳过 roll_input）
        for (int i = 0; i < kArmJoints; ++i)
            out[i] = q_out(i + 1);

        return true;
    }

    bool solveFk(Side side, const std::array<double, 7> &q, pinocchio::SE3 &out) override {
        int idx = static_cast<int>(side);

        // 构建完整的关节配置（包含 roll_input_joint）
        int num_joints = chains_[idx].getNrOfJoints();
        KDL::JntArray q_kdl(num_joints);
        q_kdl(0) = 0.0; // roll_input_joint = 0
        for (int i = 0; i < kArmJoints; ++i)
            q_kdl(i + 1) = q[i];

        KDL::Frame frame;
        if (fk_solvers_[idx]->JntToCart(q_kdl, frame) < 0)
            return false;

        out = frameToSe3(frame);
        return true;
    }

    double jointLower(Side side, int axis) const override {
        // KDL Chain 存储了关节限位
        int idx = static_cast<int>(side);
        const KDL::Segment &seg = chains_[idx].getSegment(axis);
        return -M_PI; // TODO: 从 URDF 正确提取
    }

    double jointUpper(Side side, int axis) const override {
        int idx = static_cast<int>(side);
        const KDL::Segment &seg = chains_[idx].getSegment(axis);
        return M_PI; // TODO: 从 URDF 正确提取
    }

  private:
    std::array<KDL::Chain, 2> chains_;
    std::array<std::unique_ptr<TRAC_IK::TRAC_IK>, 2> ik_solvers_;
    std::array<std::unique_ptr<KDL::ChainFkSolverPos_recursive>, 2> fk_solvers_;
    double joint2_min_, joint2_max_;
};

std::unique_ptr<Kinematics> makeTracIkKinematics(const std::string &urdf_path, double joint2_min,
                                                  double joint2_max) {
    return std::make_unique<TracIkKinematics>(urdf_path, joint2_min, joint2_max);
}

} // namespace aviator
