#pragma once
#include <boost/sml.hpp>
#include <cstdint>
#include <string>
#include <optional>
namespace aviator::fsm {
namespace sml = boost::sml;
struct INIT {}; struct INITIALIZING {}; struct READY {}; struct HOMING {}; struct STANDBY {};
struct GRASPING {}; struct FOLLOWING {}; struct CONTROL {};
struct RELEASING {}; struct SAFE {}; struct ERROR {};
struct EMERGENCY_STOP {};
struct Boot {};
struct ENTER_STANDBY {}; struct GRASP_WHEEL {}; struct START_CONTROL {};
struct EXIT_CONTROL {}; struct LEAVE_WHEEL {}; struct RESET_ERROR {};
struct Done { std::uint64_t generation; };
struct Fault {}; struct SafetyLost {}; struct Emergency {};
enum class Job { none, initialize, home, grasp, release, following_impedance };
struct Context {
  bool ready{}, following_authorized{}, source_authorized{}, input_ready{};
  bool settled{}, clear_of_wheel{}, fault_cleared{}, emergency_latched{};
  bool executor_idle{}, release_authorized{}, pending_job{}, stop_requested{};
  bool accepts_control{}, brake_requested{};
  Job job{Job::none};
  std::uint64_t generation{};
  void cancel() { accepts_control = false; job = Job::none; pending_job = false; ++generation; }
  void begin(Job value) { cancel(); job = value; pending_job = true; }
};
struct RobotMachine {
  auto operator()() const {
    using namespace sml;
    const auto free = [](const Context& c) { return c.job == Job::none && c.executor_idle && !c.emergency_latched; };
    const auto grasp_ok = [](const Context& c) {
      return c.ready && c.settled && c.following_authorized && c.job == Job::none && c.executor_idle;
    };
    const auto control_ok = [](const Context& c) {
      return c.ready && c.settled && c.following_authorized &&
             c.source_authorized && c.input_ready && c.job == Job::none && c.executor_idle;
    };
    const auto release_ok = [](const Context& c) {
      return c.ready && c.settled && c.release_authorized && c.job == Job::none && c.executor_idle;
    };
    const auto recover_ok = [](const Context& c) {
      return c.ready && c.settled && c.clear_of_wheel && c.job == Job::none && c.executor_idle;
    };
    const auto reset_ok = [](const Context& c) {
      return c.fault_cleared && c.settled && c.executor_idle && !c.emergency_latched;
    };
    const auto init_done = [](const Done& e, const Context& c) {
      return e.generation == c.generation && c.job == Job::initialize &&
             c.ready && c.settled && c.clear_of_wheel && c.executor_idle;
    };
    const auto home_done = [](const Done& e, const Context& c) {
      return e.generation == c.generation && c.job == Job::home &&
             c.ready && c.settled && c.clear_of_wheel && c.executor_idle;
    };
    const auto grasp_done = [](const Done& e, const Context& c) {
      return e.generation == c.generation && c.job == Job::grasp &&
             c.ready && c.following_authorized && c.executor_idle;
    };
    const auto release_done = [](const Done& e, const Context& c) {
      return e.generation == c.generation && c.job == Job::release &&
             c.settled && c.clear_of_wheel && c.executor_idle;
    };
    const auto following_done = [](const Done& e, const Context& c) {
      return e.generation == c.generation && c.job == Job::following_impedance &&
             c.ready && c.settled && c.following_authorized && c.executor_idle;
    };
    const auto following = [](Context& c) { c.begin(Job::following_impedance); };
    const auto initialize = [](Context& c) { c.begin(Job::initialize); };
    const auto home = [](Context& c) { c.begin(Job::home); };
    const auto grasp = [](Context& c) { c.begin(Job::grasp); };
    const auto release = [](Context& c) { c.begin(Job::release); };
    const auto finished = [](Context& c) { c.job = Job::none; };
    const auto enable = [](Context& c) { c.job = Job::none; c.accepts_control = true; };
    const auto disable = [](Context& c) {
      c.begin(Job::following_impedance); c.settled = false; c.stop_requested = true;
    };
    const auto stop = [](Context& c) { c.cancel(); c.stop_requested = true; };
    const auto emergency = [](Context& c) {
      c.cancel(); c.emergency_latched = true; c.brake_requested = true; c.stop_requested = true;
    };
    return make_transition_table(
      *state<INIT> + event<Boot>[free] / initialize = state<INITIALIZING>,
      state<INITIALIZING> + event<Done>[init_done] / finished = state<READY>,
      state<READY> + event<ENTER_STANDBY>[recover_ok] / home = state<HOMING>,
      state<HOMING> + event<Done>[home_done] / finished = state<STANDBY>,
      state<STANDBY> + event<ENTER_STANDBY> / [] {},
      state<STANDBY> + event<GRASP_WHEEL>[grasp_ok] / grasp = state<GRASPING>,
      state<GRASPING> + event<Done>[grasp_done] / following = state<FOLLOWING>,
      state<FOLLOWING> + event<Done>[following_done] / finished,
      state<FOLLOWING> + event<START_CONTROL>[control_ok] / enable = state<CONTROL>,
      state<CONTROL> + event<EXIT_CONTROL>[([](const Context& c) { return c.following_authorized; })] / disable = state<FOLLOWING>,
      state<FOLLOWING> + event<EXIT_CONTROL> / [] {},
      state<FOLLOWING> + event<LEAVE_WHEEL>[release_ok] / release = state<RELEASING>,
      state<FOLLOWING> + event<ENTER_STANDBY>[release_ok] / release = state<RELEASING>,
      state<RELEASING> + event<Done>[release_done] / finished = state<STANDBY>,
      state<SAFE> + event<ENTER_STANDBY>[recover_ok] / home = state<HOMING>,
      state<SAFE> + event<LEAVE_WHEEL>[release_ok] / release = state<RELEASING>,
      state<ERROR> + event<RESET_ERROR>[reset_ok] / stop = state<SAFE>,
      state<READY> + event<SafetyLost> / stop = state<SAFE>,
      state<HOMING> + event<SafetyLost> / stop = state<SAFE>,
      state<STANDBY> + event<SafetyLost> / stop = state<SAFE>,
      state<GRASPING> + event<SafetyLost> / stop = state<SAFE>,
      state<FOLLOWING> + event<SafetyLost> / stop = state<SAFE>,
      state<CONTROL> + event<SafetyLost> / stop = state<SAFE>,
      state<RELEASING> + event<SafetyLost> / stop = state<SAFE>,
      state<INIT> + event<Fault> / stop = state<ERROR>,
      state<INITIALIZING> + event<Fault> / stop = state<ERROR>,
      state<STANDBY> + event<Fault> / stop = state<ERROR>,
      state<READY> + event<Fault> / stop = state<ERROR>,
      state<HOMING> + event<Fault> / stop = state<ERROR>,
      state<GRASPING> + event<Fault> / stop = state<ERROR>,
      state<FOLLOWING> + event<Fault> / stop = state<ERROR>,
      state<CONTROL> + event<Fault> / stop = state<ERROR>,
      state<RELEASING> + event<Fault> / stop = state<ERROR>,
      state<SAFE> + event<Fault> / stop = state<ERROR>,
      state<INIT> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<INITIALIZING> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<STANDBY> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<READY> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<HOMING> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<GRASPING> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<FOLLOWING> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<CONTROL> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<RELEASING> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<SAFE> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<ERROR> + event<Emergency> / emergency = state<EMERGENCY_STOP>
    );
  }
};
// Snapshot is supplied by the owner thread from actual executor/safety evidence.
struct Snapshot {
  bool ready{}, settled{}, clear_of_wheel{}, following_authorized{};
  bool source_authorized{}, input_ready{}, fault_cleared{}, executor_idle{};
  bool release_authorized{}, emergency_known{}, emergency_latched{};
  std::string fault;
};
enum class Operation { enter_standby, grasp_wheel, start_control, exit_control, leave_wheel, reset_error };
enum class Reply { accepted, completed, busy, invalid_state, capability_unavailable };
struct Task { Job job; std::uint64_t generation, deadline; };
struct Timeouts { std::uint64_t initialize = 30000000, grasp = 180000000, release = 180000000, home = 180000000; };
class RobotStateMachine {
 public:
  explicit RobotStateMachine(Timeouts = {});
  void boot(const Snapshot&, std::uint64_t now);
  void supervise(const Snapshot&, std::uint64_t now);
  Reply request(Operation, const Snapshot&, std::uint64_t now);
  void done(std::uint64_t generation, const Snapshot&, std::uint64_t now = 0);
  void failed(std::uint64_t generation, const std::string&);
  void fault(const std::string&);
  void safetyLost(const std::string&);
  void emergency(const std::string&);
  std::optional<Task> takeTask();
  bool takeStop();
  bool takeBrake();
  const char* state() const;
  unsigned stateCode() const;
  bool acceptsControl() const { return machine_.is(sml::state<CONTROL>) && context_.accepts_control; }
  std::uint64_t controlSince() const { return control_since_; }
  std::uint64_t generation() const { return context_.generation; }
  Job job() const { return context_.job; }
  const std::string& currentError() const { return error_; }
  const std::string& lastError() const { return last_error_; }
 private:
  void snapshot(const Snapshot&);
  void deadline(std::uint64_t now);
  Context context_;
  sml::sm<RobotMachine> machine_{context_};
  Timeouts timeouts_;
  std::uint64_t deadline_ = 0, control_since_ = 0, last_now_ = 0;
  std::string error_, last_error_;
};
const char* replyName(Reply);
} // namespace aviator::fsm
