#include <cstring>

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "joint_driver_module/joint_driver_module.h"
#include "joy_stick_module/joy_stick_module.h"
#include "control_module/control_module.h"
#include "imu_module/imu_module.h"
#include "sim_module/sim_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>>
    aimrt_module_register_array[]{
        {"JointDriverModule",
         []() -> aimrt::ModuleBase* {
           return new mybipedal_deploy::joint_driver_module::JointDriverModule();
         }},
        {"JoyStickModule",
         []() -> aimrt::ModuleBase* {
           return new mybipedal_deploy::joy_stick_module::JoyStickModule();
         }},
        {"ControlModule",
         []() -> aimrt::ModuleBase* {
           return new mybipedal_deploy::rl_control_module::ControlModule();
         }},
        {"ImuModule",
         []() -> aimrt::ModuleBase* {
           return new mybipedal_deploy::imu_module::ImuModule();
         }},
        {"SimModule",
         []() -> aimrt::ModuleBase* {
           return new mybipedal_deploy::sim_module::SimModule();
         }},
    };

AIMRT_PKG_MAIN(aimrt_module_register_array)
