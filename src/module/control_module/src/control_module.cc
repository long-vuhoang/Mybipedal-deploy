#include "control_module/control_module.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "control_module/global.h"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cstdlib>
#include <filesystem>

namespace mybipedal_deploy::rl_control_module {

bool ControlModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;
  SetLogger(core_.GetLogger());
  subs_.clear();

  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  try {
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(file_path.data());
      freq_ = cfg_node["control_frequecy"].as<int32_t>();
      use_sim_handles_ = cfg_node["use_sim_handles"].as<bool>();

      // 解析状态机
      last_trigger_time_ = high_resolution_clock::now();
      state_machine_.Init(cfg_node["robot_states"]);
      for (auto iter = cfg_node["robot_states"].begin(); iter != cfg_node["robot_states"].end(); iter++) {
        auto trigger_topic = iter->second["trigger_topic"].as<std::string>();
        if (trigger_topics_.find(trigger_topic) != trigger_topics_.end()) {
          continue;
        }
        trigger_topics_.insert(trigger_topic);
        subs_.push_back(core_.GetChannelHandle().GetSubscriber(trigger_topic));
        bool ret = aimrt::channel::Subscribe<std_msgs::msg::Float32>(subs_.back(),
          [this, trigger_topic](const std::shared_ptr<const std_msgs::msg::Float32>& msg) {
            if (Throttler(high_resolution_clock::now(), last_trigger_time_, milliseconds(1000)) && state_machine_.OnEvent(trigger_topic)) {
              auto now_state = state_machine_.GetCurrentState();
              auto controller_names = state_machine_.GetCurrentControllerNames();
              for (auto name : controller_names) {
                // printf("RestartController: %s\n", name.c_str());
                controller_map_[name]->RestartController();
              }
              AIMRT_INFO("Trigger event: [{}] -> {}", trigger_topic, now_state);
            }
          });
        AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed.");
      }
      // auto controller_names = state_machine_.GetCurrentControllerNames();
      // for (auto name : controller_names) {
      //   printf("name: %s\n", name.c_str());
      // }

      // 解析控制器
      for (auto iter = cfg_node["controllers"].begin(); iter != cfg_node["controllers"].end(); iter++) {
        std::string controller_name = iter->first.as<std::string>();
        // printf("controller: %s\n", controller_name.c_str());

        if (controller_name.substr(0, 3) == "rl_") {
          controller_map_[controller_name] = std::make_shared<RLController>(use_sim_handles_);
        } else if (controller_name.substr(0, 3) == "pd_") {
          controller_map_[controller_name] = std::make_shared<PDController>(use_sim_handles_);
        } else {
          AIMRT_ERROR("Unknown controller type: {}", controller_name);
        }
        controller_map_[controller_name]->Init(iter->second);
        ctrl_names_.push_back(controller_name);
      }

      // 设置 joint_xxx_index_map_ 的尺度
      for (const auto& joint : cfg_node["joint_list"]) {
        joint_state_index_map_[joint.as<std::string>()] = -1;
      }
      std::vector<std::string> joint_list = cfg_node["joint_list"].as<std::vector<std::string>>();
      for (size_t ii = 0; ii < joint_list.size(); ++ii) {
        joint_cmd_index_map_[joint_list[ii]] = ii;
      }
      joint_offset_map_ = cfg_node["joint_offset"].as<std::map<std::string, double>>();
      joint_names_ = joint_list;
      InitTrace();
      // printf("joint_cmd_index_map_: ");
      // for (const auto& pair : joint_cmd_index_map_) {
      //   printf("%s: %d, ", pair.first.c_str(), pair.second);
      // }
      // printf("\n");

      // 控制器订阅
      subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_node["sub_joy_vel_name"].as<std::string>()));
      bool ret = aimrt::channel::Subscribe<geometry_msgs::msg::Twist>(subs_.back(), 
        [this](const std::shared_ptr<const geometry_msgs::msg::Twist>& msg) {
          last_cmd_[0].store(static_cast<float>(msg->linear.x), std::memory_order_relaxed);
          last_cmd_[1].store(static_cast<float>(msg->linear.y), std::memory_order_relaxed);
          last_cmd_[2].store(static_cast<float>(msg->angular.z), std::memory_order_relaxed);
          auto controller_names = state_machine_.GetCurrentControllerNames();
          for (const auto& name : controller_names) {
            controller_map_[name]->SetCmdData(*msg);
          }
        });

      subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_node["sub_imu_data_name"].as<std::string>()));
      ret &= aimrt::channel::Subscribe<sensor_msgs::msg::Imu>(subs_.back(), 
        [this](const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) {
          auto controller_names = state_machine_.GetCurrentControllerNames();
          for (const auto& name : controller_names) {
            controller_map_[name]->SetImuData(*msg);
          }
          //AIMRT_INFO("Receive imu data, linear_acceleration: [{:.2f}, {:.2f}, {:.2f}]", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        });

      subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_node["sub_joint_state_name"].as<std::string>()));
      ret &= aimrt::channel::Subscribe<sensor_msgs::msg::JointState>(subs_.back(), 
        [this](const std::shared_ptr<const sensor_msgs::msg::JointState>& msg) {
          // 仅初始化一次 joint_state_index_map_
          if (joint_state_index_map_.begin()->second == -1) {
            for (size_t i = 0; i < msg->name.size(); i++) {
              joint_state_index_map_[msg->name[i]] = i;
            }
          }

          // debug trace: ghi state thô (chưa trừ offset) theo thứ tự joint_list
          if (state_trace_.enabled()) {
            const size_t n = joint_names_.size();
            if (state_idx_.empty()) {
              for (const auto& jn : joint_names_) {
                auto it = joint_state_index_map_.find(jn);
                state_idx_.push_back(it == joint_state_index_map_.end() ? -1 : it->second);
              }
            }
            constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
            auto pick = [&](const std::vector<double>& v, int idx) {
              return (idx >= 0 && static_cast<size_t>(idx) < v.size()) ? static_cast<float>(v[idx]) : kNaN;
            };
            for (size_t ii = 0; ii < n; ++ii) {
              state_row_[ii] = pick(msg->position, state_idx_[ii]);
              state_row_[n + ii] = pick(msg->velocity, state_idx_[ii]);
              state_row_[2 * n + ii] = pick(msg->effort, state_idx_[ii]);
            }
            state_trace_.Push(SteadyNowNs(), state_row_.data());
          }

          // 新设置的 offset
          sensor_msgs::msg::JointState temp_msg = *msg;
          for (const auto& joint : joint_offset_map_) {
            temp_msg.position[joint_state_index_map_.at(joint.first)] -= joint.second;
          }

          for (const auto& controller : controller_map_) {
            controller.second->SetJointStateData(temp_msg, joint_state_index_map_);
          }
        });
      AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed.");

      // 控制器发布
      joint_cmd_pub_ = core_.GetChannelHandle().GetPublisher(cfg_node["pub_joint_cmd_name"].as<std::string>());
      executor_ = core_.GetExecutorManager().GetExecutor("rl_control_pub_thread");
      AIMRT_CHECK_ERROR_THROW(executor_, "Can not get executor 'rl_control_pub_thread'.");
      aimrt::channel::RegisterPublishType<my_ros2_proto::msg::JointCommand>(joint_cmd_pub_);
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");
  return true;
}

bool ControlModule::Start() {
  AIMRT_INFO("thread safe [{}]", executor_.ThreadSafe());
  try {
    executor_.Execute([this]() { MainLoop(); });
    AIMRT_INFO("Started succeeded.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }
  return true;
}

void ControlModule::Shutdown() {
  run_flag_.store(false);
  // Ghi log ngay tại đây, không chờ MainLoop thoát (module khác có thể làm process chết trước đó).
  // RingTrace::Dump() tự đóng băng buffer nên an toàn dù MainLoop còn chạy thêm ~1 ms.
  DumpTrace();
}

bool ControlModule::MainLoop() {
  try {
    AIMRT_INFO("Start MainLoop.");
    auto const period = nanoseconds(1'000'000'000 / freq_);
    time_point<high_resolution_clock, nanoseconds> next_iteration_time = high_resolution_clock::now();

    my_ros2_proto::msg::JointCommand cmd_msg;
    cmd_msg.name.resize(joint_cmd_index_map_.size(), "");
    cmd_msg.position.resize(joint_cmd_index_map_.size(), 0.0);
    cmd_msg.velocity.resize(joint_cmd_index_map_.size(), 0.0);
    cmd_msg.effort.resize(joint_cmd_index_map_.size(), 0.0);
    cmd_msg.damping.resize(joint_cmd_index_map_.size(), 0.0);
    cmd_msg.stiffness.resize(joint_cmd_index_map_.size(), 0.0);

    while (run_flag_) {
      next_iteration_time += period;
      std::this_thread::sleep_until(next_iteration_time);

      auto controller_names = state_machine_.GetCurrentControllerNames();
      for (const auto& name : controller_names) {
        controller_map_[name]->Update();
        my_ros2_proto::msg::JointCommand tmp_cmd = controller_map_[name]->GetJointCmdData();
        // 将 tmp_cmd 中的数据复制到 cmd_msg 中
        for (size_t ii = 0; ii < tmp_cmd.name.size(); ii++) {
          int index = joint_cmd_index_map_[tmp_cmd.name[ii].c_str()];
          cmd_msg.name[index] = tmp_cmd.name[ii];
          cmd_msg.position[index] = tmp_cmd.position[ii] + joint_offset_map_[tmp_cmd.name[ii]];
          cmd_msg.velocity[index] = tmp_cmd.velocity[ii];
          cmd_msg.effort[index] = tmp_cmd.effort[ii];
          cmd_msg.damping[index] = tmp_cmd.damping[ii];
          cmd_msg.stiffness[index] = tmp_cmd.stiffness[ii];
        }
      }
      if (cmd_trace_.enabled()) {
        const size_t n = joint_names_.size();
        for (size_t ii = 0; ii < n; ++ii) cmd_row_[ii] = static_cast<float>(cmd_msg.position[ii]);
        cmd_row_[n + 0] = last_cmd_[0].load(std::memory_order_relaxed);
        cmd_row_[n + 1] = last_cmd_[1].load(std::memory_order_relaxed);
        cmd_row_[n + 2] = last_cmd_[2].load(std::memory_order_relaxed);
        float cid = -1.0f;  // controller cuối cùng trong danh sách = controller quyết định cmd
        if (!controller_names.empty()) {
          for (size_t k = 0; k < ctrl_names_.size(); ++k) {
            if (ctrl_names_[k] == controller_names.back()) { cid = static_cast<float>(k); break; }
          }
        }
        cmd_row_[n + 3] = cid;
        cmd_trace_.Push(SteadyNowNs(), cmd_row_.data());
      }
      aimrt::channel::Publish<my_ros2_proto::msg::JointCommand>(joint_cmd_pub_, cmd_msg);
    }
    DumpTrace();
    AIMRT_INFO("Exit MainLoop.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit MainLoop with exception, {}", e.what());
    DumpTrace();
    return false;
  }
  return true;
}

void ControlModule::InitTrace() {
  const char* dir = std::getenv("MYBIPEDAL_LOG_DIR");
  if (!dir || !*dir) return;  // mặc định: tắt, không tốn gì
  double secs = 300.0;
  if (const char* v = std::getenv("MYBIPEDAL_LOG_SECONDS")) {
    try { secs = std::stod(v); } catch (...) {}
  }
  secs = std::max(secs, 1.0);
  const size_t n = joint_names_.size();

  std::vector<std::string> cmd_cols, state_cols;
  for (const auto& j : joint_names_) cmd_cols.push_back("cmd_" + j);
  for (const auto& j : joint_names_) state_cols.push_back("pos_" + j);
  for (const auto& j : joint_names_) state_cols.push_back("vel_" + j);
  for (const auto& j : joint_names_) state_cols.push_back("eff_" + j);
  for (const char* c : {"vx", "vy", "wz", "ctrl_id"}) cmd_cols.emplace_back(c);

  // ring giữ `secs` giây gần nhất; state có thể nhanh hơn control loop nên dư gấp đôi
  cmd_trace_.Init(cmd_cols, static_cast<size_t>(secs * freq_));
  state_trace_.Init(state_cols, static_cast<size_t>(secs * freq_ * 2));
  cmd_row_.assign(n + 4, 0.0f);
  state_row_.assign(3 * n, 0.0f);
  trace_dir_ = dir;
  trace_t0_ns_ = SteadyNowNs();
  AIMRT_INFO("Debug trace ON: dir={}, last {}s (joint_cmd.csv, joint_state.csv, meta.txt written on exit)", trace_dir_, secs);
}

void ControlModule::DumpTrace() {
  if (!cmd_trace_.enabled() && !state_trace_.enabled()) return;
  if (trace_dumped_.test_and_set()) return;  // chỉ dump 1 lần
  try {
    std::filesystem::create_directories(trace_dir_);
  } catch (const std::exception& e) {
    AIMRT_ERROR("Trace: cannot create dir {}: {}", trace_dir_, e.what());
    return;
  }
  const size_t nc = cmd_trace_.Dump(trace_dir_ + "/joint_cmd.csv", trace_t0_ns_);
  const size_t ns = state_trace_.Dump(trace_dir_ + "/joint_state.csv", trace_t0_ns_);

  if (std::FILE* f = std::fopen((trace_dir_ + "/meta.txt").c_str(), "w")) {
    std::fprintf(f, "control_frequency_hz=%d\n", freq_);
    std::fprintf(f, "cmd_rows=%zu\nstate_rows=%zu\n", nc, ns);
    std::fprintf(f, "cmd_total_pushed=%llu\nstate_total_pushed=%llu\n",
                 static_cast<unsigned long long>(cmd_trace_.total_pushed()),
                 static_cast<unsigned long long>(state_trace_.total_pushed()));
    std::fprintf(f, "joints=");
    for (size_t i = 0; i < joint_names_.size(); ++i) std::fprintf(f, "%s%s", i ? "," : "", joint_names_[i].c_str());
    std::fprintf(f, "\nctrl_ids=");
    for (size_t i = 0; i < ctrl_names_.size(); ++i) std::fprintf(f, "%s%zu:%s", i ? "," : "", i, ctrl_names_[i].c_str());
    std::fprintf(f, "\n");
    std::fclose(f);
  }
  AIMRT_INFO("Trace dumped to {}: joint_cmd.csv ({} rows), joint_state.csv ({} rows)", trace_dir_, nc, ns);
}

}  // namespace mybipedal_deploy::rl_control_module



