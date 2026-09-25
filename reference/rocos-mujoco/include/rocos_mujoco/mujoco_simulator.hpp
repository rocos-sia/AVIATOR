// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// mujoco_simulator.hpp
// MuJoCo-based robot simulation that exposes hardware state via POSIX
// shared memory, following the rocos-ecm protocol.  PDO variable names
// and transform parameters are read from hardware_config.yaml so the
// controller (rocos-app) can connect unchanged.

#pragma once

#include <cstdint>
#include <array>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include "rocos_mujoco/aviator_protocol.hpp"

#include <mujoco/mujoco.h>

// Forward declarations
namespace YAML { class Node; }
namespace rocos { class SharedMemoryConfig; }

namespace rocos_mujoco {

class HardwareMJ;

// ==========================================================================
// PDO variable descriptor — built from hardware_config.yaml at runtime
// ==========================================================================
struct PdoVarInfo {
    std::string yaml_key; ///< YAML key name (e.g. "status_word", "control_word")
    std::string name;     ///< PDO variable name (from YAML, e.g. "Status word")
    int         offset;   ///< byte offset within this slave's PD area
    int         size;     ///< size in bytes (sizeof(T))
};

/// Per-slave PDO layout, built dynamically from YAML config.
struct SlavePdoLayout {
    std::vector<PdoVarInfo> inputs;
    std::vector<PdoVarInfo> outputs;
    int input_size  = 0;   ///< total bytes for this slave's input area
    int output_size = 0;   ///< total bytes for this slave's output area

    /// @brief 按 yaml_key 查找 PDO 变量偏移，未找到返回 -1
    int findOffset(const std::string& yaml_key, bool is_input) const {
        const auto& vec = is_input ? inputs : outputs;
        for (const auto& v : vec) {
            if (v.yaml_key == yaml_key) return v.offset;
        }
        return -1;
    }
};

/// @brief 精简版 CiA 402 驱动状态机（仿真用）
/// 根据 control_word 位自动切换状态，返回对应的 status_word
class Cia402Sim {
public:
    Cia402Sim() = default;

    /// @brief 处理一个周期的 control_word，更新内部状态
    /// @return 当前的 status_word（uint16）
    uint16_t process(uint16_t control_word);

    /// @brief 当前是否处于 OperationEnabled 状态
    bool isEnabled() const { return state_ == 4; }  // OperationEnabled

    /// @brief 当前 CiA 402 状态值
    uint8_t state() const { return state_; }

    /// @brief 获取当前 status_word
    uint16_t statusWord() const;

    /// @brief 触发故障 —— 模拟驱动检测到过流/过温/跟随误差等异常
    ///        状态转为 FaultReactionActive→Fault，需 Fault Reset 才能恢复
    void triggerFault() { if (state_ != 0 && state_ != 7) state_ = 6; }

private:
    uint8_t  state_{1};         // 初始 = SwitchOnDisabled
    bool     fault_reset_prev_{false};  // bit7 上升沿检测
};

// ==========================================================================
// MujocoSimulator
// ==========================================================================

class MujocoSimulator {
public:
    MujocoSimulator(const std::string &urdf_path,
                    const std::string &hw_config_path,
                    int                ecat_id       = 0,
                    double             cycle_time_us = 1000.0);

    ~MujocoSimulator();

    MujocoSimulator(const MujocoSimulator &)            = delete;
    MujocoSimulator &operator=(const MujocoSimulator &) = delete;

    // ---- Lifecycle ----------------------------------------------------
    bool initialize();
    void step();
    void run(double duration_sec = -1.0);

    void enableVisualization(bool enable = true) { visualize_ = enable; }
    void setVerbose(bool verbose = true)         { verbose_  = verbose; }

    // ---- Accessors ----------------------------------------------------
    double getSimTime()    const { return sim_time_; }
    int    getJointCount() const { return static_cast<int>(joints_.size()); }
    double getCycleTimeUs() const { return cycle_time_us_; }
    const std::string& getModelPath()  const { return urdf_path_; }
    const std::string& getConfigPath() const { return hw_config_path_; }

    /// @brief 获取 HardwareMJ 可视化后端指针（仅当 visualize_ 启用时非空）
    HardwareMJ* getVisualizer() const { return visualizer_; }

private:
    // ---- Initialization helpers ---------------------------------------
    bool loadModel(const std::string &urdf_path);
    bool loadHardwareConfig(const std::string &yaml_path);
    bool setupSharedMemory();
    bool initializeCrawling();
    void updateCrawling();

    /// Build PDO layout for one slave from its YAML config node.
    SlavePdoLayout buildPdoLayout(const std::string &type,
                                  const YAML::Node  &node);

    // ---- Per-step helpers ---------------------------------------------
    void readCommands();
    void applyControl();
    void stepPhysics();
    void writeSensorData();
    void signalClients();
    bool initializeDocking();
    void updateDocking();
    bool initializeAviator();
    void updateAviator();
    std::unique_ptr<aviator::Channel> aviator_channel_;
    int aviator_bus_id_ = 0;
    int aviator_tcp_[2]{-1, -1}, aviator_handle_[2]{-1, -1}, aviator_weld_[2]{-1, -1};
    int aviator_wheel_[2]{-1, -1};
    double aviator_position_tol_ = .003, aviator_rotation_tol_ = .035;
    double aviator_speed_tol_ = .02, aviator_stable_time_ = .1;
    double aviator_stable_ = 0, aviator_bad_time_ = 0;
    bool aviator_fault_holding_ = false;
    std::vector<double> aviator_fault_positions_;

    // ---- Unit conversions ---------------------------------------------
    double  cntToUnit(double cnt, double cnt_per_unit, double ratio,
                       int32_t offset_pos_cnt = 0) const;
    int32_t unitToCnt(double unit, double cnt_per_unit, double ratio,
                      int32_t offset_pos_cnt = 0) const;
    double  torqueToUnit(int16_t raw, double torque_per_unit) const;

    // ---- MuJoCo -------------------------------------------------------
    mjModel *m_ = nullptr;
    mjData  *d_ = nullptr;

    // ---- Shared memory ------------------------------------------------
    rocos::SharedMemoryConfig *shm_ = nullptr;

    // ---- Joint mapping ------------------------------------------------
    struct JointEntry {
        int         slave_id       = -1;
        int         mj_joint_qpos  = -1;
        int         mj_joint_dof   = -1;
        int         mj_actuator    = -1;
        std::string joint_name;
        int         pd_input_base  = 0;
        int         pd_output_base = 0;
        double      cnt_per_unit    = 1.0;
        double      torque_per_unit = 1.0;
        double      ratio           = 1.0;
        int32_t     offset_pos_cnt  = 0;
        SlavePdoLayout pdo;            ///< dynamic PDO layout from YAML
        // Cached command values (read from shm each cycle)
        uint16_t control_word      = 0;
        int8_t   mode_of_operation = 0;
        int32_t  target_position   = 0;
        int32_t  target_velocity   = 0;
        int16_t  target_torque     = 0;

        // CiA 402 state machine (仿真驱动状态迁移)
        Cia402Sim cia;

        // 下使能时锁死的目标位置（记录下使能瞬间的 qpos）
        double hold_position{0.0};
        bool   hold_valid{false};

        // 位置 PID 积分器（消除重力等持续扰动引起的稳态误差）
        double pos_integral{0.0};
        int8_t prev_mode_of_operation{-1};  // 用于检测模式切换以重置积分
    };
    std::vector<JointEntry> joints_;

    // Optional one-joint step-response trace, enabled only by environment.
    struct JointTraceSample {
        double sim_time, monotonic_time, target, q_before, q_after;
        int mode;
    };
    std::string step_trace_file_, step_trace_joint_;
    int step_trace_joint_index_ = -1;
    std::vector<JointTraceSample> step_trace_;

    // ---- FT sensor mapping --------------------------------------------
    struct FTEntry {
        int            slave_id      = -1;
        int            pd_input_base = 0;
        SlavePdoLayout pdo;            ///< dynamic PDO layout from YAML
    };
    std::vector<FTEntry> ft_sensors_;

    // ---- IO module mapping --------------------------------------------
    struct IOEntry {
        int            slave_id       = -1;
        int            pd_input_base  = 0;
        int            pd_output_base = 0;
        SlavePdoLayout pdo;
        int di_channels = 0, do_channels = 0;
        int ai_channels = 0, ao_channels = 0;
    };
    std::vector<IOEntry> io_modules_;

    // Step 1 is opt-in through the docking section of the hardware YAML.
    struct DockingState {
        bool enabled = false, b_activated = false, b_reported = false;
        int slave = 0, site_a = -1, target_a = -1, weld_a = -1;
        int site_b = -1, target_b = -1, weld_b = -1;
        int32_t status = 0, previous_command = 0;
        double position_tolerance = .001, rotation_tolerance = .01745329252;
        double linear_speed_tolerance = .003, angular_speed_tolerance = .01745329252;
        double stable_time = .15, a_stable = 0, b_stable = 0;
        double report_elapsed = 0;
    } docking_;

    struct CrawlingState {
        struct End {
            int site = -1, attached = -1;
            std::array<int, 6> targets{}, welds{};
            bool confirmed = false;
            double stable = 0;
        };
        bool enabled = false;
        int slave = 0, status = 0, ack = 0, seen = 0, pending = 0;
        std::array<End, 2> ends;
        std::array<int, 6> markers{};
        double position_tolerance = .001, rotation_tolerance = .01745329252;
        double linear_speed_tolerance = .003, angular_speed_tolerance = .01745329252;
        double stable_time = .15, candidate_stable = 0, report_elapsed = 0;
        double max_position_error = 0, max_rotation_error = 0;
        double self_check_elapsed = 0, min_self_clearance = 1;
    } crawling_;

    // ---- Paths & parameters -------------------------------------------
    std::string urdf_path_;
    std::string hw_config_path_;
    double      sim_time_      = 0.0;
    double      cycle_time_us_ = 1000.0;

    // ---- Visualization ------------------------------------------------
    HardwareMJ *visualizer_ = nullptr;
    bool        visualize_  = false;
    bool        verbose_    = true;

    // ---- PID gains (position loop, applied via qfrc_applied) -----------
    static constexpr double KP_ = 10000.0;   // 比例增益 [Nm/rad] (无内置执行器时使用)
    static constexpr double KI_ = 0.0;   // 积分增益 [Nm/(rad·s)]
    static constexpr double KD_ = 85.0;     // 速度阻尼 [Nm/(rad/s)] (无内置执行器时使用被动阻尼)
    static constexpr double KV_ = 50.0;
    double position_kp_ = KP_, position_kd_ = KD_;
};

}  // namespace rocos_mujoco
