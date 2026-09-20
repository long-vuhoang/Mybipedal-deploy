// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.
#include "sim_module/sim_module.h"
#include <yaml-cpp/yaml.h>
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

namespace mybipedal_deploy::sim_module {

bool SimModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  start_time_ = high_resolution_clock::now();

  core_ = core;
  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  if (file_path.empty()) {
    AIMRT_ERROR("Init failed, [file_path] Empty");
    return false;
  }
  try {
    YAML::Node cfg_node = YAML::LoadFile(file_path.data());
    filename_ = cfg_node["model_file"].as<std::string>();

    joint_cmd_sub_ = core_.GetChannelHandle().GetSubscriber(cfg_node["sub_joint_cmd_topic"].as<std::string>());
    aimrt::channel::Subscribe<my_ros2_proto::msg::JointCommand>(joint_cmd_sub_, std::bind(&SimModule::CmdCallback, this, std::placeholders::_1));
    imu_data_pub_ = core_.GetChannelHandle().GetPublisher(cfg_node["pub_imu_data_topic"].as<std::string>());
    aimrt::channel::RegisterPublishType<sensor_msgs::msg::Imu>(imu_data_pub_);
    joint_state_pub_ =core_.GetChannelHandle().GetPublisher(cfg_node["pub_joint_state_topic"].as<std::string>());
    aimrt::channel::RegisterPublishType<sensor_msgs::msg::JointState>(joint_state_pub_);

    render_executor_ = core_.GetExecutorManager().GetExecutor("sim_render_thread");

    AIMRT_INFO("Init succeeded.");
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit MainLoop with exception, {}", e.what());
    return false;
  }
}

bool SimModule::Start() {
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultPerturb(&pert_);

  // Render thread
  render_executor_.Execute([this]() {
    sim_ = std::make_shared<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &cam_, &opt_, &pert_, false);
    sim_->LoadMessage(filename_.data());
    const int kErrorLength = 1024;
    char loadError[kErrorLength] = "";
    m_ = mj_loadXML(filename_.data(), nullptr, loadError, kErrorLength);
    mju::strcpy_arr(sim_->load_error, loadError);
    if (m_) {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      d_ = mj_makeData(m_);
    }
    is_render_thread_running_ = true;
    sim_->RenderLoop();
  });

  while (!is_render_thread_running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (d_) {
    sim_->Load(m_, d_, filename_.data());
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    mj_forward(m_, d_);
    free(ctrl_noise_);
    ctrl_noise_ = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m_->nu));
    mju_zero(ctrl_noise_, m_->nu);
  } else {
    sim_->LoadMessageClear();
  }

  joint_names_.clear();
  for (int i = 0; i < m_->njnt; ++i) {
    if (m_->jnt_type[i] == mjJNT_FREE) {
      continue;
    }
    const char* joint_name = mj_id2name(m_, mjOBJ_JOINT, i);
    joint_names_.push_back(std::string(joint_name));
  }

  // init pid
  target_q_.resize(joint_names_.size());
  target_dq_.resize(joint_names_.size());
  target_tq_.resize(joint_names_.size());
  kp_.resize(joint_names_.size());
  kd_.resize(joint_names_.size());
  motor_torque_.resize(joint_names_.size());

  running_.store(true, std::memory_order_release);
  AIMRT_INFO("Started succeeded.");
  return true;
}

void SimModule::Shutdown() {
  // 1) Từ giờ CmdCallback (chạy inline trong thread điều khiển 1 kHz) sẽ bỏ qua.
  running_.store(false, std::memory_order_release);

  // 2) Dừng render thread TRƯỚC khi giải phóng m_/d_ (render thread đọc thẳng m_/d_ mỗi frame).
  //    RenderLoop() kết thúc thì tự đặt exitrequest = 2.
  if (sim_) {
    int expected = 0;  // nếu cửa sổ đã bị đóng, RenderLoop đã đặt =2 rồi: đừng ghi đè
    sim_->exitrequest.compare_exchange_strong(expected, 1);
    for (int i = 0; i < 400 && sim_->exitrequest.load() != 2; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (sim_->exitrequest.load() != 2) {
      AIMRT_WARN("Render thread did not exit in 2s; skip freeing MuJoCo model/data to avoid use-after-free.");
      return;
    }
  }

  // 3) Giữ mutex để chắc chắn không có CmdCallback đang dở dang, rồi mới giải phóng.
  std::unique_lock<std::recursive_mutex> lock;
  if (sim_) lock = std::unique_lock<std::recursive_mutex>(sim_->mtx);
  free(ctrl_noise_);
  ctrl_noise_ = nullptr;
  mj_deleteData(d_);
  d_ = nullptr;
  mj_deleteModel(m_);
  m_ = nullptr;
  AIMRT_INFO("Shutdown succeeded.");
}

void SimModule::CmdCallback(const std::shared_ptr<const my_ros2_proto::msg::JointCommand>& msg) {
  if (!running_.load(std::memory_order_acquire)) return;  // chưa Start xong hoặc đã Shutdown
  sensor_msgs::msg::Imu imu_data_msg;
  sensor_msgs::msg::JointState joint_states_msg;

  auto elapsed = high_resolution_clock::now() - start_time_;
  if (elapsed <= milliseconds(3000)) {
    return;
  }

  const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
  if (!running_.load(std::memory_order_acquire) || !d_) return;  // kiểm tra lại dưới lock (Shutdown có thể vừa chạy)
  WriteMotorCmd(*msg);
  mj_step(m_, d_);
  ReadSensorData(imu_data_msg, joint_states_msg);

  aimrt::channel::Publish<sensor_msgs::msg::Imu>(imu_data_pub_, imu_data_msg);
  aimrt::channel::Publish<sensor_msgs::msg::JointState>(joint_state_pub_, joint_states_msg);
}

void SimModule::ReadSensorData(sensor_msgs::msg::Imu& imu_data, sensor_msgs::msg::JointState& joint_state) {
  auto duration = high_resolution_clock::now().time_since_epoch();
  auto sec = duration_cast<seconds>(duration);
  auto nanosec = duration_cast<nanoseconds>(duration - sec);

  imu_data.orientation.w = d_->sensordata[0];
  imu_data.orientation.x = d_->sensordata[1];
  imu_data.orientation.y = d_->sensordata[2];
  imu_data.orientation.z = d_->sensordata[3];
  imu_data.angular_velocity.x = d_->sensordata[4];
  imu_data.angular_velocity.y = d_->sensordata[5];
  imu_data.angular_velocity.z = d_->sensordata[6];
  imu_data.linear_acceleration.x = d_->sensordata[13];
  imu_data.linear_acceleration.y = d_->sensordata[14];
  imu_data.linear_acceleration.z = d_->sensordata[15];
  imu_data.header.stamp.sec = sec.count();
  imu_data.header.stamp.nanosec = nanosec.count();

  joint_state.name = joint_names_;
  joint_state.position.resize(joint_names_.size(), 0.0);
  joint_state.velocity.resize(joint_names_.size(), 0.0);
  joint_state.effort.resize(joint_names_.size(), 0.0);
  memcpy((void*)joint_state.position.data(), d_->qpos+7, joint_names_.size() * sizeof(double));
  memcpy((void*)joint_state.velocity.data(), d_->qvel+6, joint_names_.size() * sizeof(double));
  memcpy((void*)joint_state.effort.data(), d_->qfrc_actuator+6, joint_names_.size() * sizeof(double));
  joint_state.header.stamp.sec = sec.count();
  joint_state.header.stamp.nanosec = nanosec.count();
}

void SimModule::WriteMotorCmd(my_ros2_proto::msg::JointCommand cmd) {
  for (size_t ii = 0; ii < cmd.name.size(); ii++) {
    joint_state_index_map_[cmd.name[ii]] = ii;
  }

  for (size_t ii = 0; ii < joint_names_.size(); ++ii) {
    int index = joint_state_index_map_[joint_names_[ii]];
    target_q_(ii) = cmd.position[index];
    target_dq_(ii) = cmd.velocity[index];
    target_tq_(ii) = cmd.effort[index];
    kp_(ii) = cmd.stiffness[index];
    kd_(ii) = cmd.damping[index];
  }
  array_t q = Eigen::Map<array_t>(d_->qpos + 7, joint_names_.size());
  array_t dq = Eigen::Map<array_t>(d_->qvel + 6, joint_names_.size());
  motor_torque_ = target_tq_ + (target_q_ - q) * kp_ + (target_dq_ - dq) * kd_;
  d_->ctrl = motor_torque_.data();

  // 添加控制噪声
  if (sim_->ctrl_noise_std) {
    mjtNum rate = mju_exp(-m_->opt.timestep / mju_max(sim_->ctrl_noise_rate, mjMINVAL));
    mjtNum scale = sim_->ctrl_noise_std * mju_sqrt(1-rate*rate);
    for (int i=0; i<m_->nu; i++) {
      ctrl_noise_[i] = rate * ctrl_noise_[i] + scale * mju_standardNormal(nullptr);
      d_->ctrl[i] += ctrl_noise_[i];
    }
  }
}

}  // namespace mybipedal_deploy::sim_module
