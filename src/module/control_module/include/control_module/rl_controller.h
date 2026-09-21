#pragma once
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <memory>
#include <set>
#include <atomic>

#include "control_module/controller_base.h"
#include "control_module/debug_recorder.h"
#include "control_module/rotation_tools.h"

namespace mybipedal_deploy::rl_control_module {

class RLController : public ControllerBase {
 public:
  RLController(const bool use_sim_handles);
  ~RLController() = default;

  void Init(const YAML::Node &cfg_node) override;
  void RestartController() override;

  void Update() override;
  size_t DumpTrace(const std::string& dir, int64_t t0_ns) override;
  my_ros2_proto::msg::JointCommand GetJointCmdData() override;

 private:
  void LoadModel();
  void UpdateStateEstimation();
  void ComputeObservation();
  void ComputeActions();

 private:
  struct WalkStepConf {
    double action_scale;
    int decimation;
    bool sw_mode;
    double cmd_threshold;
  } walk_step_conf_;

  struct GaitCommandConf {
    double freq{0.0};
    double offset{0.0};
    double duration{0.0};
    double swing_height{0.0};
  } gait_conf_;

  double gait_indices_{0.0};
  double loop_dt_{0.001};

  struct ObsScales {
    double lin_vel;
    double ang_vel;
    double dof_pos;
    double dof_vel;
    double quat;
  } obs_scales_;

  struct PolicyOnnxConf {
    std::string policy_file;
    int actions_size;
    int observations_size;
    int num_hist;
    double observations_clip;
    double actions_clip;
  } policy_onnx_conf_;

  struct EncoderOnnxConf {
    std::string encoder_file;
    int32_t est_size{3};
  } encoder_onnx_conf_;

  struct LPFConf {
    double wc;
    double ts;
  } lpf_conf_;

  // onnxy policy
  std::shared_ptr<Ort::Env> policy_onnx_env_;
  std::unique_ptr<Ort::Session> policy_session_ptr_;
  Ort::MemoryInfo memory_info_;
  std::vector<const char *> policy_input_names_;
  std::vector<const char *> policy_output_names_;
  std::vector<std::vector<int64_t>> policy_input_shapes_;
  std::vector<std::vector<int64_t>> policy_output_shapes_;

  // onnx encoder
  std::shared_ptr<Ort::Env> encoder_onnx_env_;
  std::unique_ptr<Ort::Session> encoder_session_ptr_;
  std::vector<char*> encoder_input_names_;
  std::vector<char*> encoder_output_names_;
  std::vector<std::vector<int64_t>> encoder_input_shapes_;
  std::vector<std::vector<int64_t>> encoder_output_shapes_;

  // compute in algorithm
  std::vector<float> actions_;
  vector_t single_obs_;
  std::vector<float> commands_;      // size 3
  std::vector<float> encoder_input_; // size observations_size * num_hist
  std::vector<float> est_;           // size encoder_conf_.est_size
  std::vector<float> actor_input_;   // size est_size + observations_size + 3
  vector_t last_actions_;
  // vector_t propri_history_buffer_;
  Eigen::Matrix<float, Eigen::Dynamic, 1> propri_history_buffer_;
  struct Proprioception {
    vector_t joint_pos;
    vector_t joint_vel;
    vector3_t base_ang_vel;
    vector3_t base_euler_xyz;
    vector3_t projected_gravity;
  } propri_;

  // debug trace obs/est/cmd/action @ policy rate (bật bằng env MYBIPEDAL_LOG_DIR)
  RingTrace obs_trace_;
  std::vector<float> obs_row_;

  // other
  int64_t loop_count_;
  std::vector<digital_lp_filter<double>> low_pass_filters_;
  std::atomic_bool is_first_frame_{true};
};

}  // namespace mybipedal_deploy::rl_control_module
