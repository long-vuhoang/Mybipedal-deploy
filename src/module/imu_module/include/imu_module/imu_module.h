// Copyright (c) 2026, Mybipedal.
// All rights reserved.
#pragma once

// cpp
#include <future>
#include <thread>

// projects
#include "aimrt_module_cpp_interface/module_base.h"
#include "dm_imu/imu_driver.h"
#include "imu_module/imu_frame.h"

namespace mybipedal_deploy::imu_module {

class ImuModule : public aimrt::ModuleBase {
 public:
  ImuModule() = default;
  ~ImuModule() override = default;

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "ImuModule"};
  }

 private:
  void PublishLoop();
  auto GetLogger() { return core_.GetLogger(); }

 private:
  std::atomic_bool is_running_ = false;
  std::thread publish_thread_;
  std::string port_;
  int    baud_{921600};
  bool   do_config_{false};
  double publish_frequency_ = 100.0f;
  int bind_cpu_ = -1;
  int rt_priority_ = -1;
  std::shared_ptr<dm_imu::ImuDriver> imu_;
  ImuFrameFix frame_fix_;  // chuyển khung cảm biến -> FLU/ENU (tắt mặc định, xem imu_frame.h)
  aimrt::CoreRef core_;
  aimrt::channel::PublisherRef pub_imu_;
};

}  // namespace mybipedal_deploy::imu_module