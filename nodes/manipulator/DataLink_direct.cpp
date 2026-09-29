#include "DataLink_direct.hpp"
#include "aviator/backend.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <urdf_parser/urdf_parser.h>
#include <algorithm>

namespace aviator {

namespace {
[[noreturn]] void fail(const std::string &message) { throw std::runtime_error(message); }

void require(bool ok, const std::string &message) {
    if (!ok)
        fail(message);
}

// 位置伺服增益，与原 rocos_mujoco::MujocoSimulator 保持一致：
//   KP_ = 1000.0 [Nm/rad]     比例增益
//   KD_ = 80.0   [Nm/(rad/s)] 关节阻尼
// 原仿真器在关节没有内置 actuator 时会把这部分阻尼叠加到 dof_damping 上；缺少它会
// 使关节欠阻尼，跟踪过程中冲过关节限位（表现为 J2 越过 95° 后绕到 445°）。
constexpr double kPositionGain = 1000.0;
constexpr double kJointDamping = 80.0;

// 非实时模式下允许物理线程领先控制器的步数。
//
// 取 0：物理线程只在控制器消费完上一拍后再推进一步，形成严格 1:1 锁步。
// 这是无头验证需要的确定性——否则控制器在做逆解规划（几十毫秒墙钟）期间，
// 物理会推进一大段仿真时间，且用的是上一阶段的旧目标，结果不可复现。
constexpr uint64_t kLockstepLead = 0;
} // namespace

MuJoCoDirectDataLink::MuJoCoDirectDataLink(mjModel *model, mjData *data,
                                           const std::string &urdf_path)
    : model_(model), data_(data) {
    require(model_ != nullptr && data_ != nullptr, "MuJoCo model/data cannot be null");
    require(model_->nu == 0,
            "Expected a torque-driven MJCF without actuators (nu == 0), got nu = " +
                std::to_string(model_->nu));

    initJointMapping(urdf_path);

    // 抓取 site 与 weld 约束
    const char *names[2] = {"left", "right"};
    for (int side = 0; side < 2; ++side) {
        const std::string tcp = std::string(names[side]) + "_tcp";
        const std::string handle = std::string(names[side]) + "_handle";
        const std::string weld = std::string(names[side]) + "_grasp";

        tcp_site_[side] = mj_name2id(model_, mjOBJ_SITE, tcp.c_str());
        handle_site_[side] = mj_name2id(model_, mjOBJ_SITE, handle.c_str());
        weld_id_[side] = mj_name2id(model_, mjOBJ_EQUALITY, weld.c_str());

        require(tcp_site_[side] >= 0, "Cannot find site: " + tcp);
        require(handle_site_[side] >= 0, "Cannot find site: " + handle);
        require(weld_id_[side] >= 0, "Cannot find equality: " + weld);

        require(model_->eq_type[weld_id_[side]] == mjEQ_WELD &&
                    model_->eq_objtype[weld_id_[side]] == mjOBJ_SITE &&
                    model_->eq_obj1id[weld_id_[side]] == tcp_site_[side] &&
                    model_->eq_obj2id[weld_id_[side]] == handle_site_[side],
                "Grasp equality must weld " + tcp + " to " + handle);

        // 运行时开关在 mjData::eq_active
        data_->eq_active[weld_id_[side]] = 0;
    }

    // 计算一次正向动力学，使 qfrc_bias 在第一次步进前就有效
    mj_forward(model_, data_);

    // 补上关节阻尼：MJCF 没有内置 actuator，原仿真器同样在无 actuator 时把 KD_
    // 叠加到 joint damping，用于抑制位置伺服的超调（否则会冲过关节限位）。
    for (int i = 0; i < 14; ++i) {
        original_damping_[i] = model_->dof_damping[joint_dof_adr_[i]];
        model_->dof_damping[joint_dof_adr_[i]] += kJointDamping;
    }

    // 目标与保持位形都对齐到当前位形（keyframe 载入后的姿态）
    for (int i = 0; i < 14; ++i) {
        target_[i] = data_->qpos[joint_qpos_adr_[i]];
        hold_[i] = target_[i];
    }

    updateAlignmentLocked();

    // 启动物理线程（等价于原架构里独立的仿真器进程）。
    // 默认实时模式：按 1 ms 墙钟节拍推进，使可视化下仿真时间≈真实时间。
    real_time_ = true;
    next_tick_ = std::chrono::steady_clock::now();
    running_ = true;
    physics_thread_ = std::thread([this] { physicsLoop(); });
}

MuJoCoDirectDataLink::~MuJoCoDirectDataLink() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
    }
    tick_cv_.notify_all();
    if (physics_thread_.joinable())
        physics_thread_.join();
    for (int i = 0; i < 14; ++i)
        model_->dof_damping[joint_dof_adr_[i]] = original_damping_[i];
}

void MuJoCoDirectDataLink::initJointMapping(const std::string &urdf_path) {
    // 关节速度限位从 URDF 读取，与原后端使用同一份模型定义
    const auto urdf = urdf::parseURDFFile(urdf_path);
    require(urdf != nullptr, "Cannot parse control URDF: " + urdf_path);
    const char *sides[2] = {"L", "R"};
    for (int s = 0; s < 2; ++s) {
        for (int j = 0; j < 7; ++j) {
            const int idx = s * 7 + j;
            char name[64];
            snprintf(name, sizeof(name), "AR5-5_07%s-W4C4A2_joint_%d", sides[s], j + 1);

            const int joint_id = mj_name2id(model_, mjOBJ_JOINT, name);
            require(joint_id >= 0, std::string("Cannot find joint: ") + name);

            joint_qpos_adr_[idx] = model_->jnt_qposadr[joint_id];
            joint_dof_adr_[idx] = model_->jnt_dofadr[joint_id];
            const auto joint = urdf->getJoint(name);
            require(joint && joint->limits && std::isfinite(joint->limits->velocity) &&
                        joint->limits->velocity > 0, std::string("Missing/invalid velocity limit: ") + name);
            joint_vel_limit_[idx] = joint->limits->velocity;
        }
    }

    // 轮盘是被动关节，无驱动，由双臂经 weld 拖动
    const char *wheel_names[2] = {"roll_input_joint", "pitch_input_joint"};
    for (int i = 0; i < 2; ++i) {
        const int joint_id = mj_name2id(model_, mjOBJ_JOINT, wheel_names[i]);
        require(joint_id >= 0, std::string("Cannot find wheel joint: ") + wheel_names[i]);
        wheel_qpos_adr_[i] = model_->jnt_qposadr[joint_id];
        wheel_dof_adr_[i] = model_->jnt_dofadr[joint_id];
    }
}

void MuJoCoDirectDataLink::physicsLoop() {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(model_->opt.timestep));
    while (running_) {
        // 节流：非实时模式下严格锁步（最多领先 kLockstepLead 步），
        // 实时模式下不限制，让物理按墙钟自由推进。
        if (!real_time_) {
            tick_cv_.wait(lock, [this] {
                return !running_ || real_time_ || tick_ <= consumed_ + kLockstepLead;
            });
            if (!running_)
                break;
            if (real_time_)
                continue;
        } else {
            // waitTick() also notifies this CV. A consumed tick is NOT permission to
            // step early: only a mode change/shutdown may interrupt the deadline.
            // Always wait (even for a late deadline) to release the state lock.
            tick_cv_.wait_until(lock, next_tick_, [this] { return !running_ || !real_time_; });
            if (!running_)
                break;
            if (!real_time_)
                continue; // re-evaluate the lockstep predicate after a mode change
        }

        // 施加位置伺服力矩：重力前馈 + 比例力矩，对应原仿真器的 CSP 分支。
        // MJCF 没有 actuator，只能通过 qfrc_applied 驱动；阻尼由关节本身的
        // dof_damping（构造时已补上 kJointDamping）提供。
        for (int i = 0; i < 14; ++i) {
            const int dof = joint_dof_adr_[i];
            const int qpos = joint_qpos_adr_[i];
            const double setpoint = enabled_[i / 7] ? target_[i] : hold_[i];
            data_->qfrc_applied[dof] =
                data_->qfrc_bias[dof] + kPositionGain * (setpoint - data_->qpos[qpos]);
        }

        // 推进物理一步。原架构里这属于仿真器进程，单进程内由本线程承担，
        // 使控制器与仿真严格同步（一个控制周期 = 一个仿真步）。
        // mutex_ also protects the viewer's scene snapshot.
        mj_step(model_, data_);
        for (int side = 0; side < 2; ++side) {
            const double q = data_->qpos[joint_qpos_adr_[7 * side + 1]];
            elbow_range_[0] = std::min(elbow_range_[0], q);
            elbow_range_[1] = std::max(elbow_range_[1], q);
            if (!std::isfinite(q)) fault_ = 1;
        }
        ++tick_;
        sim_step_span_ = model_->opt.timestep;

        updateAlignmentLocked();
        tick_cv_.notify_all();

        if (real_time_) {
            next_tick_ += period;
            const auto now = std::chrono::steady_clock::now();
            if (next_tick_ < now)
                next_tick_ = now; // 落后太多时不再追赶，避免累积
        }
    }
}

void MuJoCoDirectDataLink::updateAlignmentLocked() {
    feedback_mono_ = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    // 抓取对齐误差：TCP site 与 handle site 的位姿差与相对速度
    for (int side = 0; side < 2; ++side) {
        const int tcp = tcp_site_[side];
        const int target_site = handle_site_[side];

        mjtNum delta[3], qa[4], qb[4], rotation[3];
        mju_sub3(delta, data_->site_xpos + 3 * tcp, data_->site_xpos + 3 * target_site);
        mju_mat2Quat(qa, data_->site_xmat + 9 * tcp);
        mju_mat2Quat(qb, data_->site_xmat + 9 * target_site);
        mju_subQuat(rotation, qa, qb);
        position_error_[side] = mju_norm3(delta);
        rotation_error_[side] = mju_norm3(rotation);
    }

    ready_ = enabled_[0] && enabled_[1] && !fault_;
    // Alignment errors are diagnostics only. Weld activation is an open-loop
    // simulation action, never a TCP tolerance/arrival test.

}

double MuJoCoDirectDataLink::getJointPosition(Side side, int axis) const {
    std::lock_guard<std::mutex> lock(mutex_);
    require(axis >= 0 && axis < 7, "Joint axis out of range");
    return data_->qpos[joint_qpos_adr_[sideOffset(side) + axis]];
}

double MuJoCoDirectDataLink::getJointVelocity(Side side, int axis) const {
    std::lock_guard<std::mutex> lock(mutex_);
    require(axis >= 0 && axis < 7, "Joint axis out of range");
    return data_->qvel[joint_dof_adr_[sideOffset(side) + axis]];
}

void MuJoCoDirectDataLink::setJointPositions(const std::array<double, 14> &q) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = q;
}

std::array<double, 14> MuJoCoDirectDataLink::jointTargets() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_;
}

double MuJoCoDirectDataLink::jointVelLimit(Side side, int axis) const {
    require(axis >= 0 && axis < 7, "Joint axis out of range");
    return joint_vel_limit_[sideOffset(side) + axis];
}

bool MuJoCoDirectDataLink::isEnabled(Side side) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_[static_cast<int>(side)];
}

void MuJoCoDirectDataLink::enable(Side side) {
    std::lock_guard<std::mutex> lock(mutex_);
    const int offset = sideOffset(side);
    // 使能瞬间把目标对齐到实测位形，避免力矩跳变
    for (int axis = 0; axis < 7; ++axis)
        target_[offset + axis] = data_->qpos[joint_qpos_adr_[offset + axis]];
    enabled_[static_cast<int>(side)] = true;
}

void MuJoCoDirectDataLink::disable(Side side) {
    std::lock_guard<std::mutex> lock(mutex_);
    const int offset = sideOffset(side);
    for (int axis = 0; axis < 7; ++axis)
        hold_[offset + axis] = data_->qpos[joint_qpos_adr_[offset + axis]];
    enabled_[static_cast<int>(side)] = false;
}

ArmFeedback MuJoCoDirectDataLink::armFeedback() const {
    std::lock_guard<std::mutex> lock(mutex_);
    ArmFeedback f; f.target = target_;
    for(int i=0;i<14;++i) { f.q[i]=data_->qpos[joint_qpos_adr_[i]]; f.dq[i]=data_->qvel[joint_dof_adr_[i]]; }
    for(int side=0;side<2;++side) {
        f.enabled[side]=enabled_[side];f.valid[side]=true;f.sample_time[side]=feedback_mono_;
        const auto* p=data_->site_xpos+3*tcp_site_[side];
        mjtNum q[4];mju_mat2Quat(q,data_->site_xmat+9*tcp_site_[side]);
        for(int i=0;i<3;++i)f.tcp[side*7+i]=p[i];
        for(int i=0;i<3;++i)f.tcp[side*7+3+i]=q[i+1];
        f.tcp[side*7+6]=q[0];
    }
    return f;
}

GraspState MuJoCoDirectDataLink::graspState() const {
    std::lock_guard<std::mutex> lock(mutex_);

    GraspState state;
    // 心跳用仿真时间，与 Aviator 的心跳超时判据（datalink_->time()）同一时基
    state.heartbeat = data_->time;

    // 锁定位 = 两个 weld 的激活状态（与原协议 bit0 / bit1 语义一致）
    const bool locked_l = data_->eq_active[weld_id_[0]] != 0;
    const bool locked_r = data_->eq_active[weld_id_[1]] != 0;
    state.locked = (locked_l ? 1u : 0u) | (locked_r ? 2u : 0u);

    state.ready = ready_;
    state.fault = fault_;

    // 轮盘实测位形（被动关节，由双臂拖动）
    state.angle = data_->qpos[wheel_qpos_adr_[0]];
    state.displacement = data_->qpos[wheel_qpos_adr_[1]];

    state.position_error[0] = position_error_[0];
    state.position_error[1] = position_error_[1];
    state.rotation_error[0] = rotation_error_[0];
    state.rotation_error[1] = rotation_error_[1];

    state.ack = ack_seq_;
    state.result = last_result_;
    return state;
}

uint64_t MuJoCoDirectDataLink::sendGraspCommand(GraspCommand command) {
    std::lock_guard<std::mutex> lock(mutex_);

    ++command_seq_;

    switch (command) {
    case GraspCommand::Lock:
        // 只要求设备可用；不检查 TCP 偏差或抓握就绪。
        if (fault_)
            last_result_ = GraspResult::Fault;
        else if (!enabled_[0] || !enabled_[1])
            last_result_ = GraspResult::NotEnabled;
        else {
            data_->eq_active[weld_id_[0]] = 1;
            data_->eq_active[weld_id_[1]] = 1;
            last_result_ = GraspResult::Ok;
        }
        break;

    case GraspCommand::Unlock:
        data_->eq_active[weld_id_[0]] = 0;
        data_->eq_active[weld_id_[1]] = 0;
        last_result_ = GraspResult::Ok;
        break;

    case GraspCommand::ResetFault:
        if (data_->eq_active[weld_id_[0]] || data_->eq_active[weld_id_[1]]) {
            last_result_ = GraspResult::Fault;
        } else {
            fault_ = 0;
            last_result_ = GraspResult::Ok;
        }
        break;
    }

    ack_seq_ = command_seq_;
    return command_seq_;
}

void MuJoCoDirectDataLink::waitTick() {
    std::unique_lock<std::mutex> lock(mutex_);
    require(running_, "Physics thread is not running");
    // 等待物理线程推进出下一拍。物理线程负责推进，这里只同步，
    // 与原架构里控制器等待仿真周期信号量的语义一致。
    tick_cv_.wait(lock, [this] { return !running_ || tick_ > consumed_; });
    consumed_ = tick_;
    lock.unlock();
    tick_cv_.notify_all(); // wake lockstep physics after consuming the previous tick
}

double MuJoCoDirectDataLink::time() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_->time;
}

void MuJoCoDirectDataLink::setRealTime(bool enabled) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        real_time_ = enabled;
        next_tick_ = std::chrono::steady_clock::now();
    }
    tick_cv_.notify_all();
}

uint64_t MuJoCoDirectDataLink::stepCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return tick_;
}

std::array<double, 2> MuJoCoDirectDataLink::elbowRange() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return elbow_range_;
}

// —— 后端工厂 ——
std::unique_ptr<DataLink> makeMuJoCoDirectDataLink(mjModel *model, mjData *data,
                                                   const std::string &urdf_path) {
    return std::make_unique<MuJoCoDirectDataLink>(model, data, urdf_path);
}

} // namespace aviator
