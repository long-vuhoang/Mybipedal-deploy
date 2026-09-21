#include "control_module/rl_controller.h"
#include <string.h>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>

namespace mybipedal_deploy::rl_control_module {

RLController::RLController(const bool use_sim_handles)
    : ControllerBase(use_sim_handles),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
}

void RLController::Init(const YAML::Node& cfg_node) {
  //get joint_names_ list
  joint_names_.clear();
  joint_names_ = cfg_node["joint_list"].as<std::vector<std::string>>();
  joint_state_data_.name = joint_names_;
  joint_state_data_.position.resize(joint_names_.size(), 0.0);
  joint_state_data_.velocity.resize(joint_names_.size(), 0.0);
  joint_state_data_.effort.resize(joint_names_.size(), 0.0);

  //get joint_conf_ list
  joint_conf_.init_state = Eigen::Map<vector_t>(cfg_node["init_state"].as<std::vector<double>>().data(), cfg_node["init_state"].as<std::vector<double>>().size());
  joint_conf_.tau_limit = Eigen::Map<vector_t>(cfg_node["tau_limit"].as<std::vector<double>>().data(), cfg_node["tau_limit"].as<std::vector<double>>().size());
  joint_conf_.stiffness = Eigen::Map<vector_t>(cfg_node["stiffness"].as<std::vector<double>>().data(), cfg_node["stiffness"].as<std::vector<double>>().size());
  joint_conf_.damping = Eigen::Map<vector_t>(cfg_node["damping"].as<std::vector<double>>().data(), cfg_node["damping"].as<std::vector<double>>().size());

  // Other RL arguments
  // clang-format off
  walk_step_conf_.action_scale  = cfg_node["walk_step_conf"]["action_scale"].as<double>();
  walk_step_conf_.decimation    = cfg_node["walk_step_conf"]["decimation"].as<int32_t>();
  walk_step_conf_.sw_mode       = cfg_node["walk_step_conf"]["sw_mode"].as<bool>();
  walk_step_conf_.cmd_threshold = cfg_node["walk_step_conf"]["cmd_threshold"].as<double>();
  // loop_dt: dt của vòng điều khiển tổng (vd 1/control_frequecy). Dùng để
  // tích phân pha bước chân: gait_indices += freq * (loop_dt * decimation).
  loop_dt_                      = cfg_node["walk_step_conf"]["loop_dt"].as<double>();

  obs_scales_.lin_vel           = cfg_node["obs_scales"]["lin_vel"].as<double>();
  obs_scales_.ang_vel           = cfg_node["obs_scales"]["ang_vel"].as<double>();
  obs_scales_.dof_pos           = cfg_node["obs_scales"]["dof_pos"].as<double>();
  obs_scales_.dof_vel           = cfg_node["obs_scales"]["dof_vel"].as<double>();
  obs_scales_.quat              = cfg_node["obs_scales"]["quat"].as<double>();

  policy_onnx_conf_.policy_file        = cfg_node["policy_onnx_conf"]["policy_file"].as<std::string>();
  policy_onnx_conf_.actions_size       = cfg_node["policy_onnx_conf"]["actions_size"].as<int32_t>();
  policy_onnx_conf_.observations_size  = cfg_node["policy_onnx_conf"]["observations_size"].as<int32_t>();
  policy_onnx_conf_.num_hist           = cfg_node["policy_onnx_conf"]["num_hist"].as<int32_t>();
  policy_onnx_conf_.observations_clip  = cfg_node["policy_onnx_conf"]["observations_clip"].as<double>();
  policy_onnx_conf_.actions_clip       = cfg_node["policy_onnx_conf"]["actions_clip"].as<double>();

  // ---- Add new: encoder (velocity estimator) ----
  encoder_onnx_conf_.encoder_file    = cfg_node["encoder_onnx_conf"]["encoder_file"].as<std::string>();
  encoder_onnx_conf_.est_size        = cfg_node["encoder_onnx_conf"]["est_size"].as<int32_t>();

  // ---- Add new: gait command ----
  gait_conf_.freq          = cfg_node["gait_command"]["freq"].as<double>();
  gait_conf_.offset        = cfg_node["gait_command"]["offset"].as<double>();
  gait_conf_.duration      = cfg_node["gait_command"]["duration"].as<double>();
  gait_conf_.swing_height  = cfg_node["gait_command"]["swing_height"].as<double>();

  lpf_conf_.wc                  = cfg_node["lpf_conf"]["wc"].as<double>();
  lpf_conf_.ts                  = cfg_node["lpf_conf"]["ts"].as<double>();
  LoadModel();
  // clang-format on
  actions_.resize(policy_onnx_conf_.actions_size);
  last_actions_.resize(policy_onnx_conf_.actions_size);
  last_actions_.setZero();
  single_obs_.resize(policy_onnx_conf_.observations_size);
  single_obs_.setZero();
  propri_history_buffer_.resize(policy_onnx_conf_.observations_size * policy_onnx_conf_.num_hist);
  propri_history_buffer_.setZero();

  commands_.assign(3, 0.0f);
  encoder_input_.assign(policy_onnx_conf_.observations_size * policy_onnx_conf_.num_hist, 0.0f);
  est_.assign(encoder_onnx_conf_.est_size, 0.0f);
  actor_input_.assign(encoder_onnx_conf_.est_size + policy_onnx_conf_.observations_size + 3, 0.0f);

  gait_indices_ = 0.0;
  loop_count_ = 0;
  low_pass_filters_.clear();
  for (size_t i = 0; i < policy_onnx_conf_.actions_size; ++i) {
    low_pass_filters_.emplace_back(lpf_conf_.wc, lpf_conf_.ts);
  }
  propri_.joint_pos.resize(policy_onnx_conf_.actions_size);
  propri_.joint_vel.resize(policy_onnx_conf_.actions_size);

  // ---- debug trace: đúng vector đưa vào policy, để so real vs sim ----
  if (const char* dir = std::getenv("MYBIPEDAL_LOG_DIR"); dir && *dir) {
    double secs = 300.0;
    if (const char* v = std::getenv("MYBIPEDAL_LOG_SECONDS")) {
      try { secs = std::stod(v); } catch (...) {}
    }
    const int32_t N = policy_onnx_conf_.actions_size;
    std::vector<std::string> cols;
    for (const char* a : {"gyro_x", "gyro_y", "gyro_z"}) cols.emplace_back(std::string("obs_") + a);  // *ang_vel_scale
    for (const char* a : {"grav_x", "grav_y", "grav_z"}) cols.emplace_back(std::string("obs_") + a);
    for (int i = 0; i < N; ++i) cols.emplace_back("obs_dq_pos_" + std::to_string(i));   // (q - init)*scale
    for (int i = 0; i < N; ++i) cols.emplace_back("obs_dq_vel_" + std::to_string(i));   // dq*scale
    for (int i = 0; i < N; ++i) cols.emplace_back("obs_last_act_" + std::to_string(i));
    for (const char* a : {"sin", "cos", "gait_freq", "gait_offset", "gait_duration", "gait_swing"}) cols.emplace_back(std::string("obs_") + a);
    for (int i = 0; i < encoder_onnx_conf_.est_size; ++i) cols.emplace_back("est_" + std::to_string(i));
    for (const char* a : {"cmd_vx", "cmd_vy", "cmd_wz"}) cols.emplace_back(a);
    for (int i = 0; i < N; ++i) cols.emplace_back("act_" + std::to_string(i));
    obs_row_.assign(cols.size(), 0.0f);
    const double rate = 1.0 / (loop_dt_ * walk_step_conf_.decimation);
    obs_trace_.Init(cols, static_cast<size_t>(std::max(secs, 1.0) * rate * 1.2) + 16);
  }
}

size_t RLController::DumpTrace(const std::string& dir, int64_t t0_ns) {
  if (!obs_trace_.enabled()) return 0;
  return obs_trace_.Dump(dir + "/obs.csv", t0_ns);
}

void RLController::RestartController() {
  is_first_frame_ = true;
  gait_indices_ = 0.0;
  loop_count_ = 0;
  std::fill(actions_.begin(), actions_.end(), 0.0f);
  last_actions_.setZero();
}

void RLController::Update() {
  UpdateStateEstimation();
  // compute observation & actions
  if (loop_count_ % walk_step_conf_.decimation == 0) {
    ComputeObservation();
    ComputeActions();
    if (obs_trace_.enabled()) {
      size_t k = 0;
      for (int i = 0; i < policy_onnx_conf_.observations_size; ++i) obs_row_[k++] = static_cast<float>(single_obs_[i]);
      for (int i = 0; i < encoder_onnx_conf_.est_size; ++i) obs_row_[k++] = est_[i];
      for (int i = 0; i < 3; ++i) obs_row_[k++] = commands_[i];
      for (int i = 0; i < policy_onnx_conf_.actions_size; ++i) obs_row_[k++] = actions_[i];
      obs_trace_.Push(SteadyNowNs(), obs_row_.data());
    }
  }
  loop_count_++;
}

my_ros2_proto::msg::JointCommand RLController::GetJointCmdData() {
  my_ros2_proto::msg::JointCommand joint_cmd;
  joint_cmd.name = joint_names_;
  joint_cmd.position.resize(joint_names_.size());
  joint_cmd.velocity.resize(joint_names_.size());
  joint_cmd.effort.resize(joint_names_.size());
  joint_cmd.damping.resize(joint_names_.size());
  joint_cmd.stiffness.resize(joint_names_.size());
  for (int ii = 0; ii < policy_onnx_conf_.actions_size; ii++) {
    scalar_t pos_des = actions_[ii] * walk_step_conf_.action_scale + joint_conf_.init_state(ii);
    double stiffness = joint_conf_.stiffness(ii);
    double damping = joint_conf_.damping(ii);
    low_pass_filters_[ii].input(pos_des);
    double pos_des_lp = low_pass_filters_[ii].output();
    joint_cmd.position[ii] = pos_des_lp;
    joint_cmd.velocity[ii] = 0.0;
    joint_cmd.effort[ii] = 0.0;
    joint_cmd.stiffness[ii] = stiffness;
    joint_cmd.damping[ii] = damping;
    last_actions_(ii, 0) = actions_[ii];
  }
  return joint_cmd;
}

void RLController::LoadModel() {
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetIntraOpNumThreads(1);
  sessionOptions.SetInterOpNumThreads(1);
  sessionOptions.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
  sessionOptions.AddConfigEntry("session.intra_op.allow_spinning", "0");
  Ort::AllocatorWithDefaultOptions allocator;

  // ---- policy (actor) ----
  policy_onnx_env_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController");
  policy_session_ptr_ = std::make_unique<Ort::Session>(*policy_onnx_env_, policy_onnx_conf_.policy_file.c_str(), sessionOptions);

  policy_input_names_.clear();
  policy_output_names_.clear();
  policy_input_shapes_.clear();
  policy_output_shapes_.clear();
  for (size_t ii = 0; ii < policy_session_ptr_->GetInputCount(); ++ii) {
    char* tempstring = new char[strlen(policy_session_ptr_->GetInputNameAllocated(ii, allocator).get()) + 1];
    strcpy(tempstring, policy_session_ptr_->GetInputNameAllocated(ii, allocator).get());
    policy_input_names_.push_back(tempstring);
    policy_input_shapes_.push_back(policy_session_ptr_->GetInputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
  }
  for (size_t ii = 0; ii < policy_session_ptr_->GetOutputCount(); ++ii) {
    char* tempstring = new char[strlen(policy_session_ptr_->GetOutputNameAllocated(ii, allocator).get()) + 1];
    strcpy(tempstring, policy_session_ptr_->GetOutputNameAllocated(ii, allocator).get());
    policy_output_names_.push_back(tempstring);
    policy_output_shapes_.push_back(policy_session_ptr_->GetOutputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
  }

  // ---- encoder (velocity estimator) ----
  encoder_onnx_env_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxEncoder");
  encoder_session_ptr_ = std::make_unique<Ort::Session>(*encoder_onnx_env_, encoder_onnx_conf_.encoder_file.c_str(), sessionOptions);

  encoder_input_names_.clear();
  encoder_output_names_.clear();
  encoder_input_shapes_.clear();
  encoder_output_shapes_.clear();
  for (size_t ii = 0; ii < encoder_session_ptr_->GetInputCount(); ++ii) {
    char* tempstring = new char[strlen(encoder_session_ptr_->GetInputNameAllocated(ii, allocator).get()) + 1];
    strcpy(tempstring, encoder_session_ptr_->GetInputNameAllocated(ii, allocator).get());
    encoder_input_names_.push_back(tempstring);
    encoder_input_shapes_.push_back(encoder_session_ptr_->GetInputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
  }
  for (size_t ii = 0; ii < encoder_session_ptr_->GetOutputCount(); ++ii) {
    char* tempstring = new char[strlen(encoder_session_ptr_->GetOutputNameAllocated(ii, allocator).get()) + 1];
    strcpy(tempstring, encoder_session_ptr_->GetOutputNameAllocated(ii, allocator).get());
    encoder_output_names_.push_back(tempstring);
    encoder_output_shapes_.push_back(encoder_session_ptr_->GetOutputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
  }
}

void RLController::UpdateStateEstimation() {
  {
    std::shared_lock<std::shared_mutex> lock(joint_state_mutex_);
    for (size_t ii = 0; ii < policy_onnx_conf_.actions_size; ++ii) {
      std::string joint_name = joint_names_[ii];
      propri_.joint_pos(ii) = joint_state_data_.position[ii];
      propri_.joint_vel(ii) = joint_state_data_.velocity[ii];
    }
  }

  {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    propri_.base_ang_vel(0) = imu_data_.angular_velocity.x;
    propri_.base_ang_vel(1) = imu_data_.angular_velocity.y;
    propri_.base_ang_vel(2) = imu_data_.angular_velocity.z;

    vector3_t gravity_vector(0, 0, -1);
    quaternion_t quat;
    quat.x() = imu_data_.orientation.x;
    quat.y() = imu_data_.orientation.y;
    quat.z() = imu_data_.orientation.z;
    quat.w() = imu_data_.orientation.w;
    matrix_t inverse_rot = GetRotationMatrixFromZyxEulerAngles(QuatToZyx(quat)).inverse();
    propri_.projected_gravity = inverse_rot * gravity_vector;
    propri_.base_euler_xyz = QuatToXyz(quat);
  }
}

// ----------------------------------------------------------------------------
// Build obs frame
//   [0:3]        ang_vel * ang_vel_scale
//   [3:6]        projected_gravity
//   [6:6+N]      (joint_pos - init_state) * dof_pos_scale
//   [6+N:6+2N]   joint_vel * dof_vel_scale
//   [6+2N:6+3N]  last_action
//   [6+3N]       sin(2*pi*gait_indices)
//   [6+3N+1]     cos(2*pi*gait_indices)
//   [6+3N+2:+4]  gait_command = [freq, offset, duration, swing_height]
//
// commands_ (vx, vy, yaw) concat later
// ComputeActions()
// ----------------------------------------------------------------------------
void RLController::ComputeObservation() {
  const int32_t N = policy_onnx_conf_.actions_size;

  {
    std::shared_lock<std::shared_mutex> lock(joy_mutex_);
    commands_[0] = static_cast<float>(joy_data_.linear.x);
    commands_[1] = static_cast<float>(joy_data_.linear.y);
    commands_[2] = static_cast<float>(joy_data_.angular.z);
    commands_[0] = std::clamp(commands_[0], -0.6f, 0.9f);
    if (walk_step_conf_.sw_mode) {
      double cmd_norm = std::sqrt(Square(joy_data_.linear.x) + Square(joy_data_.linear.y) + Square(joy_data_.angular.z));
      if (cmd_norm <= walk_step_conf_.cmd_threshold) {
        commands_[0] = 0.0f;
        commands_[1] = 0.0f;
        commands_[2] = 0.0f;
      }
    }
  }

  // ---- The gait phase integrator: gait_indices += freq * step_dt (mod 1) ----
  // step_dt = dt main loop * decimation
  const double step_dt = loop_dt_ * walk_step_conf_.decimation;
  gait_indices_ = std::fmod(gait_indices_ + gait_conf_.freq * step_dt, 1.0);
  const double sin_phase = std::sin(2.0 * M_PI * gait_indices_);
  const double cos_phase = std::cos(2.0 * M_PI * gait_indices_);

  // ---- build obs 1 frame ----
  single_obs_.segment(0, 3) = propri_.base_ang_vel * obs_scales_.ang_vel;
  single_obs_.segment(3, 3) = propri_.projected_gravity;
  single_obs_.segment(6, N) = (propri_.joint_pos - joint_conf_.init_state) * obs_scales_.dof_pos;
  single_obs_.segment(6 + N, N) = propri_.joint_vel * obs_scales_.dof_vel;
  single_obs_.segment(6 + 2 * N, N) = last_actions_;
  single_obs_(6 + 3 * N) = sin_phase;
  single_obs_(6 + 3 * N + 1) = cos_phase;
  single_obs_(6 + 3 * N + 2) = gait_conf_.freq;
  single_obs_(6 + 3 * N + 3) = gait_conf_.offset;
  single_obs_(6 + 3 * N + 4) = gait_conf_.duration;
  single_obs_(6 + 3 * N + 5) = gait_conf_.swing_height;

  if (is_first_frame_) {
    for (size_t ii = 0; ii < joint_names_.size(); ++ii) {
      low_pass_filters_[ii].init(propri_.joint_pos[ii]);
    }
    single_obs_.segment(6 + 2 * N, N).setZero();  // last_action = 0 in first frame

    // Duplicate first frame for all history buffer
    for (int ii = 0; ii < policy_onnx_conf_.num_hist; ++ii) {
      propri_history_buffer_.segment(ii * policy_onnx_conf_.observations_size, policy_onnx_conf_.observations_size) =
          single_obs_.cast<float>();
    }
    is_first_frame_ = false;
  }

  // ---- dịch history (FIFO) và chèn frame mới vào cuối — dùng làm input cho ENCODER ----
  propri_history_buffer_.head(propri_history_buffer_.size() - policy_onnx_conf_.observations_size) =
      propri_history_buffer_.tail(propri_history_buffer_.size() - policy_onnx_conf_.observations_size);
  propri_history_buffer_.tail(policy_onnx_conf_.observations_size) = single_obs_.cast<float>();

  for (int ii = 0; ii < (policy_onnx_conf_.observations_size * policy_onnx_conf_.num_hist); ++ii) {
    encoder_input_[ii] = static_cast<float>(propri_history_buffer_[ii]);
  }

  // clip obs của frame hiện tại (input cho policy), không clip encoder_input_
  scalar_t obs_min = -policy_onnx_conf_.observations_clip;
  scalar_t obs_max = policy_onnx_conf_.observations_clip;
  single_obs_ = single_obs_.cwiseMax(obs_min).cwiseMin(obs_max);
}

// ----------------------------------------------------------------------------
// 1) encoder(obs_history_flatten) -> est (velocity estimate)
// 2) policy(concat(est, single_obs, commands)) -> action
// ----------------------------------------------------------------------------
void RLController::ComputeActions() {
  // ---- 1. run encoder ----
  std::vector<Ort::Value> encoder_input_tensor;
  encoder_input_tensor.push_back(Ort::Value::CreateTensor<float>(
      memory_info_, encoder_input_.data(), encoder_input_.size(),
      encoder_input_shapes_[0].data(), encoder_input_shapes_[0].size()));

  std::vector<Ort::Value> encoder_output_values = encoder_session_ptr_->Run(
      Ort::RunOptions{}, encoder_input_names_.data(), encoder_input_tensor.data(), 1,
      encoder_output_names_.data(), 1);

  for (int i = 0; i < encoder_onnx_conf_.est_size; ++i) {
    est_[i] = *(encoder_output_values[0].GetTensorMutableData<float>() + i);
  }

  // ---- 2. build actor_input_ = concat(est, single_obs, commands) ----
  int32_t offset = 0;
  for (int i = 0; i < encoder_onnx_conf_.est_size; ++i) actor_input_[offset++] = est_[i];
  for (int i = 0; i < policy_onnx_conf_.observations_size; ++i) actor_input_[offset++] = static_cast<float>(single_obs_[i]);
  for (int i = 0; i < 3; ++i) actor_input_[offset++] = commands_[i];

  // ---- 3. run policy ----
  std::vector<Ort::Value> input_tensor;
  input_tensor.push_back(Ort::Value::CreateTensor<float>(
      memory_info_, actor_input_.data(), actor_input_.size(),
      policy_input_shapes_[0].data(), policy_input_shapes_[0].size()));

  std::vector<Ort::Value> policy_output_values = policy_session_ptr_->Run(
      Ort::RunOptions{}, policy_input_names_.data(), input_tensor.data(), 1, policy_output_names_.data(), 1);

  for (int i = 0; i < policy_onnx_conf_.actions_size; ++i) {
    actions_[i] = *(policy_output_values[0].GetTensorMutableData<float>() + i);
  }
  // limit action range
  scalar_t action_min = -policy_onnx_conf_.actions_clip;
  scalar_t action_max = policy_onnx_conf_.actions_clip;
  std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                 [action_min, action_max](scalar_t x) {
                   return std::max(action_min, std::min(action_max, x));
                 });
}

} // namespace mybipedal_deploy::rl_control_module