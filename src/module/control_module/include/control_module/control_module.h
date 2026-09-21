#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <set>
#include "aimrt_module_cpp_interface/module_base.h"
#include "control_module/debug_recorder.h"
#include "control_module/pd_controller.h"
#include "control_module/rl_controller.h"
#include "control_module/state_machine.h"

using namespace std::chrono;

namespace mybipedal_deploy::rl_control_module {

class ControlModule : public aimrt::ModuleBase {
 public:
  ControlModule() = default;
  ~ControlModule() override = default;
  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "ControlModule"};
  }
  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  bool MainLoop();
  void InitTrace();  // bật khi có env MYBIPEDAL_LOG_DIR
  void DumpTrace();  // ghi CSV 1 lần khi MainLoop kết thúc

 private:
  aimrt::CoreRef core_;
  aimrt::executor::ExecutorRef executor_;

  std::vector<aimrt::channel::SubscriberRef> subs_;
  aimrt::channel::PublisherRef joint_cmd_pub_;

  StateMachine state_machine_;
  std::set<std::string> trigger_topics_;//// 添加重复的 trigger_topic 会报错，这里用 set 来存储
  std::map<std::string, std::shared_ptr<ControllerBase>> controller_map_;
  std::unordered_map<std::string, int> joint_state_index_map_;
  std::unordered_map<std::string, int> joint_cmd_index_map_;
  std::map<std::string, double> joint_offset_map_;

  // ---- debug trace: joint_cmd vs joint_states (xem debug_recorder.h) ----
  std::vector<std::string> joint_names_;  // thứ tự = joint_list trong cfg
  std::vector<std::string> ctrl_names_;   // ctrl_id trong CSV = index ở đây
  RingTrace cmd_trace_;                   // writer: thread MainLoop
  RingTrace state_trace_;                 // writer: thread publish /joint_states (callback inline)
  RingTrace imu_trace_;                   // writer: thread publish /imu/data (callback inline)
  std::array<float, 10> imu_row_{};
  std::vector<float> cmd_row_;
  std::vector<float> state_row_;
  std::vector<int> state_idx_;            // joint_names_[i] -> index trong JointState msg
  std::array<std::atomic<float>, 3> last_cmd_{};  // vx, vy, wz gần nhất từ /cmd_vel_limiter
  std::string trace_dir_;
  int64_t trace_t0_ns_{0};
  std::atomic_flag trace_dumped_;

  bool use_sim_handles_;
  int32_t freq_;
  std::atomic_bool run_flag_{true};
  time_point<high_resolution_clock> last_trigger_time_;
};

}  // namespace mybipedal_deploy::rl_control_module
