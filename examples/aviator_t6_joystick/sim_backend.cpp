#include "sim_backend.hpp"

#include <cstdio>
#include <stdexcept>

namespace aviator {

namespace {
// Servo + alignment constants, taken verbatim from
// examples/AviatorRobot_simple/src/DataLink_direct.cpp.
constexpr double kPositionGain = 1000.0;  // arm joint P gain (N·m/rad)
constexpr double kHandGain = 10.0;        // hand joint P gain (finger links are
                                          // ~18 g; kp=1000 blows up the soft mimic
                                          // constraints -> huge QACC -> implicitfast
                                          // churn; 10 is stable and still closes)
constexpr double kJointDamping = 80.0;    // added to arm dof_damping
constexpr double kHandDamping = 20.0;     // added to hand dof_damping

constexpr double kPositionTol = 0.003;   // m, grasp alignment
constexpr double kRotationTol = 0.035;   // rad
constexpr double kSpeedTol = 0.02;       // m/s relative TCP/handle speed
constexpr double kStableTime = 0.1;      // s of in-tolerance to report aligned
constexpr double kLostPosition = 0.02;   // m, lost-grip threshold
constexpr double kLostRotation = 0.15;   // rad
constexpr double kLostDuration = 0.1;    // s

int jid(mjModel* m, const char* n) { return mj_name2id(m, mjOBJ_JOINT, n); }
int sid(mjModel* m, const char* n) { return mj_name2id(m, mjOBJ_SITE, n); }
int eid(mjModel* m, const char* n) { return mj_name2id(m, mjOBJ_EQUALITY, n); }
}  // namespace

Sim::Sim(const std::string& model_path, const Vec12& hand_grip,
         const Vec14& initial_arm_q, bool realtime, bool start_at_grasp)
    : hand_grip_(hand_grip), realtime_(realtime),
      grasp_probe_(start_at_grasp) {
    char err[1024];
    m_ = mj_loadXML(model_path.c_str(), nullptr, err, sizeof(err));
    if (!m_) throw std::runtime_error(std::string("mj_loadXML failed: ") + err);
    d_ = mj_makeData(m_);

    const int home = mj_name2id(m_, mjOBJ_KEY, "aviator_home");
    if (home < 0) throw std::runtime_error("missing aviator_home keyframe");
    mj_resetDataKeyframe(m_, d_, home);
    mj_forward(m_, d_);

    initMapping();
    // Start at the same grasp-manifold configuration as the first arm target.
    // The old aviator_home arm pose put the hands through the floor and forced
    // a large collision-driven approach before the controller could weld.
    for (int i = 0; i < 14; ++i) {
        d_->qpos[arm_qadr_[i]] = initial_arm_q[i];
        arm_target_[i] = initial_arm_q[i];
    }
    if (grasp_probe_) {
        // Start from the physical pinch instead of closing fingers through
        // the handle. The passive mimic joints are also servoed below.
        for (int i = 0; i < 12; ++i) d_->qpos[hand_qadr_[i]] = hand_grip_[i];
        for (int i = 0; i < 12; ++i) d_->qpos[mimic_qadr_[i]] = mimic_target_[i];
    }
    mj_forward(m_, d_);
    if (grasp_probe_) {
        for (int s = 0; s < 2; ++s) {
            const double error = mju_dist3(d_->site_xpos + 3 * tcp_[s],
                                           d_->site_xpos + 3 * handle_[s]);
            if (error > kPositionTol)
                throw std::runtime_error("candidate TCP is not aligned with its handle");
            d_->eq_active[weld_[s]] = 1;
        }
        mj_forward(m_, d_);
    }
    const int wheel_body = mj_name2id(m_, mjOBJ_BODY, "steering_wheel");
    const int left_palm = mj_name2id(m_, mjOBJ_BODY, "l_base_link");
    const int right_palm = mj_name2id(m_, mjOBJ_BODY, "r_base_link");
    double deepest_palm_overlap = 0.0;
    for (int i = 0; i < d_->ncon; ++i) {
        const auto& c = d_->contact[i];
        const int a = m_->geom_bodyid[c.geom1], b = m_->geom_bodyid[c.geom2];
        if ((a == wheel_body && (b == left_palm || b == right_palm)) ||
            (b == wheel_body && (a == left_palm || a == right_palm)))
            deepest_palm_overlap = std::min(deepest_palm_overlap, double(c.dist));
    }
    if (deepest_palm_overlap < -0.005)
        std::fprintf(stderr, "initial palm/wheel collision penetration: %.1f mm; "
                             "the grasp geometry must be corrected before welding%s\n",
                     -1000.0 * deepest_palm_overlap,
                     grasp_probe_ ? "" : "; recalibrate the fixed-mount grasp before welding");

    running_.store(true);
    th_ = std::thread(&Sim::physicsLoop, this);
}

Sim::~Sim() {
    running_.store(false);
    if (th_.joinable()) th_.join();
    mj_deleteData(d_);
    mj_deleteModel(m_);
}

void Sim::initMapping() {
    // 14 arm joints (L/R x 7), start servo holding the home pose.
    const char* arm_side[2] = {"L", "R"};
    for (int s = 0; s < 2; ++s) {
        for (int j = 0; j < 7; ++j) {
            char n[64];
            std::snprintf(n, sizeof(n), "AR5-5_07%s-W4C4A2_joint_%d", arm_side[s], j + 1);
            const int id = jid(m_, n);
            if (id < 0) throw std::runtime_error(std::string("missing arm joint ") + n);
            const int i = s * 7 + j;
            arm_qadr_[i] = m_->jnt_qposadr[id];
            arm_dofadr_[i] = m_->jnt_dofadr[id];
            arm_target_[i] = d_->qpos[arm_qadr_[i]];
            m_->dof_damping[arm_dofadr_[i]] += kJointDamping;
        }
    }

    // 12 independent hand joints (6/side): index/middle/ring/little _1, thumb 1..2.
    const char* hand_side[2] = {"left", "right"};
    const char* finger1[4] = {"index_1", "middle_1", "ring_1", "little_1"};
    for (int s = 0; s < 2; ++s) {
        const int base = s * 6;
        for (int f = 0; f < 4; ++f) {
            char n[64];
            std::snprintf(n, sizeof(n), "%s_%s_joint", hand_side[s], finger1[f]);
            const int id = jid(m_, n);
            if (id < 0) throw std::runtime_error(std::string("missing hand joint ") + n);
            hand_qadr_[base + f] = m_->jnt_qposadr[id];
            hand_dofadr_[base + f] = m_->jnt_dofadr[id];
            m_->dof_damping[hand_dofadr_[base + f]] += kHandDamping;
        }
        for (int t = 1; t <= 2; ++t) {
            char n[64];
            std::snprintf(n, sizeof(n), "%s_thumb_%d_joint", hand_side[s], t);
            const int id = jid(m_, n);
            if (id < 0) throw std::runtime_error(std::string("missing hand joint ") + n);
            hand_qadr_[base + 4 + (t - 1)] = m_->jnt_qposadr[id];
            hand_dofadr_[base + 4 + (t - 1)] = m_->jnt_dofadr[id];
            m_->dof_damping[hand_dofadr_[base + 4 + (t - 1)]] += kHandDamping;
        }
    }

    if (grasp_probe_) {
        const char* finger2[4] = {"index", "middle", "ring", "little"};
        for (int s = 0; s < 2; ++s) {
            const char* side = hand_side[s];
            const int base = s * 6;
            for (int f = 0; f < 4; ++f) {
                char name[64];
                std::snprintf(name, sizeof(name), "%s_%s_2_joint", side,
                              finger2[f]);
                const int id = jid(m_, name);
                if (id < 0) throw std::runtime_error(std::string("missing mimic joint ") + name);
                mimic_qadr_[base + f] = m_->jnt_qposadr[id];
                mimic_dofadr_[base + f] = m_->jnt_dofadr[id];
                mimic_target_[base + f] = 1.0843 * hand_grip_[base + f];
                m_->dof_damping[mimic_dofadr_[base + f]] += kHandDamping;
            }
            for (int t = 3; t <= 4; ++t) {
                char name[64];
                std::snprintf(name, sizeof(name), "%s_thumb_%d_joint", side, t);
                const int id = jid(m_, name);
                if (id < 0) throw std::runtime_error(std::string("missing mimic joint ") + name);
                const int i = base + 4 + (t - 3);
                mimic_qadr_[i] = m_->jnt_qposadr[id];
                mimic_dofadr_[i] = m_->jnt_dofadr[id];
                mimic_target_[i] = (t == 3 ? 0.8392 : 0.891 * 0.8392) * hand_grip_[base + 5];
                m_->dof_damping[mimic_dofadr_[i]] += kHandDamping;
            }
        }
    }

    // Passive wheel joints.
    roll_qadr_ = m_->jnt_qposadr[jid(m_, "roll_input_joint")];
    roll_dofadr_ = m_->jnt_dofadr[jid(m_, "roll_input_joint")];
    pitch_qadr_ = m_->jnt_qposadr[jid(m_, "pitch_input_joint")];
    pitch_dofadr_ = m_->jnt_dofadr[jid(m_, "pitch_input_joint")];

    // TCP / handle sites + grasp welds.
    const bool physical_sites = grasp_probe_;
    tcp_[0] = sid(m_, physical_sites ? "left_physical_tcp" : "left_tcp");
    tcp_[1] = sid(m_, physical_sites ? "right_physical_tcp" : "right_tcp");
    handle_[0] = sid(m_, "left_handle");
    handle_[1] = sid(m_, "right_handle");
    weld_[0] = eid(m_, physical_sites ? "left_physical_grasp" : "left_grasp");
    weld_[1] = eid(m_, physical_sites ? "right_physical_grasp" : "right_grasp");
    if (tcp_[0] < 0 || tcp_[1] < 0 || handle_[0] < 0 || handle_[1] < 0 || weld_[0] < 0 ||
        weld_[1] < 0)
        throw std::runtime_error(physical_sites
            ? "--grasp-only requires physical_tcp and physical_grasp sites absent from this model"
            : "missing legacy tcp/handle site or grasp weld");
}

void Sim::setArmTarget(const Vec14& q) {
    std::lock_guard<std::mutex> lk(mutex_);
    arm_target_ = q;
}

void Sim::lock() {
    std::lock_guard<std::mutex> lk(mutex_);
    d_->eq_active[weld_[0]] = 1;
    d_->eq_active[weld_[1]] = 1;
}

void Sim::unlock() {
    std::lock_guard<std::mutex> lk(mutex_);
    d_->eq_active[weld_[0]] = 0;
    d_->eq_active[weld_[1]] = 0;
}

void Sim::resetFault() {
    std::lock_guard<std::mutex> lk(mutex_);
    fault_ = false;
    lost_time_ = 0.0;
}

SimState Sim::state() const {
    SimState st;
    std::lock_guard<std::mutex> lk(mutex_);
    st.theta = d_->qpos[roll_qadr_];
    st.s = d_->qpos[pitch_qadr_];
    st.theta_vel = d_->qvel[roll_dofadr_];
    st.s_vel = d_->qvel[pitch_dofadr_];
    for (int i = 0; i < 14; ++i) st.q[i] = d_->qpos[arm_qadr_[i]];
    st.time = d_->time;
    for (int s = 0; s < 2; ++s) {
        st.position_error[s] = position_error_[s];
        st.rotation_error[s] = rotation_error_[s];
    }
    st.aligned = stable_time_ >= kStableTime;
    st.fault = fault_;
    st.locked = (d_->eq_active[weld_[0]] > 0) || (d_->eq_active[weld_[1]] > 0);
    return st;
}

double Sim::time() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return d_->time;
}

void Sim::updateAlignmentLocked() {
    const bool locked = (d_->eq_active[weld_[0]] > 0) || (d_->eq_active[weld_[1]] > 0);
    double max_speed = 0.0;
    for (int s = 0; s < 2; ++s) {
        mjtNum delta[3], qa[4], qb[4], qdiff[3], vrel[3];
        mju_sub3(delta, d_->site_xpos + 3 * tcp_[s], d_->site_xpos + 3 * handle_[s]);
        mju_mat2Quat(qa, d_->site_xmat + 9 * tcp_[s]);
        mju_mat2Quat(qb, d_->site_xmat + 9 * handle_[s]);
        mju_subQuat(qdiff, qa, qb);
        position_error_[s] = mju_norm3(delta);
        rotation_error_[s] = mju_norm3(qdiff);
        mjtNum v_tcp[6], v_handle[6];
        mj_objectVelocity(m_, d_, mjOBJ_SITE, tcp_[s], v_tcp, 0);
        mj_objectVelocity(m_, d_, mjOBJ_SITE, handle_[s], v_handle, 0);
        mju_sub3(vrel, v_tcp, v_handle);
        max_speed = std::max(max_speed, mju_norm3(vrel));
    }

    const bool pos_ok = position_error_[0] < kPositionTol && position_error_[1] < kPositionTol;
    const bool rot_ok = rotation_error_[0] < kRotationTol && rotation_error_[1] < kRotationTol;
    const bool speed_ok = max_speed < kSpeedTol;
    if (pos_ok && rot_ok && speed_ok) {
        stable_time_ += m_->opt.timestep;
    } else {
        stable_time_ = 0.0;
    }

    const bool lost =
        position_error_[0] > kLostPosition || position_error_[1] > kLostPosition ||
        rotation_error_[0] > kLostRotation || rotation_error_[1] > kLostRotation;
    if (locked && lost) {
        lost_time_ += m_->opt.timestep;
        if (lost_time_ > kLostDuration) fault_ = true;
    } else {
        lost_time_ = 0.0;
    }
}

void Sim::physicsLoop() {
    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    const auto period = std::chrono::microseconds(1000);
    while (running_.load(std::memory_order_relaxed)) {
        {
            std::lock_guard<std::mutex> lk(mutex_);
            for (int i = 0; i < 14; ++i)
                d_->qfrc_applied[arm_dofadr_[i]] =
                    d_->qfrc_bias[arm_dofadr_[i]] +
                    kPositionGain * (arm_target_[i] - d_->qpos[arm_qadr_[i]]);
            for (int i = 0; i < 12; ++i)
                d_->qfrc_applied[hand_dofadr_[i]] =
                    d_->qfrc_bias[hand_dofadr_[i]] +
                    kHandGain * (hand_grip_[i] - d_->qpos[hand_qadr_[i]]);
            if (grasp_probe_)
                for (int i = 0; i < 12; ++i)
                    d_->qfrc_applied[mimic_dofadr_[i]] =
                        d_->qfrc_bias[mimic_dofadr_[i]] +
                        kHandGain * (mimic_target_[i] - d_->qpos[mimic_qadr_[i]]);
            mj_step(m_, d_);
            updateAlignmentLocked();
        }
        if (realtime_.load(std::memory_order_relaxed)) {
            next += period;
            std::this_thread::sleep_until(next);
        }
    }
}

}  // namespace aviator
