#pragma once

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>
#include <rm_msgs/ChassisCmd.h>
#include <ros/ros.h>

#include <cstdint>
#include <string>

namespace ayanami_ros {

// cmd_chassis_publisher ROS extension (dlopen -> create_extension_module ->
// CmdChassisPublisher : ayanami::ExtensionModule).
//
// Sole chassis command publisher: subscribes the core NavigationCallbacks::
// on_velocity_command slot and republishes rm_msgs/ChassisCmd on /cmd_chassis
// (mode / ramped accel / power limit parameters, rm_chassis_controllers
// ChassisBase semantics). Knows nothing about any other module or algorithm
// state -- pure output bridge. rm_msgs is allowed HERE (rviz_viewer must stay
// rm_msgs-free).
class CmdChassisPublisher final : public ayanami::ExtensionModule {
 public:
  CmdChassisPublisher();
  ~CmdChassisPublisher() override;

  /// @brief Parameters arrive AFTER a successful load, injected by the loader
  /// (ModuleContext = logical name + the extensions config). The module no
  /// longer resolves the config path itself: the section key is its own name,
  /// so the config layout stays outside the module.
  void configureParameters(const ayanami::ModuleContext& context) override;

 private:
  void publishChassisCommand(
      const ayanami::geometry_msgs::TwistStamped& velocity);

  /// @brief Every module parameter lives here, with its default value as the
  /// field initializer.
  struct CmdChassisPublisherParam {
    std::string cmd_chassis_topic{"/cmd_chassis"};
    std::uint8_t mode{0};
    double accel_linear_x{3.0};
    double accel_linear_y{3.0};
    double accel_angular_z{6.0};
    double power_limit{100.0};
  };

  ros::NodeHandle nh;
  ros::Publisher cmd_chassis_pub;
  CmdChassisPublisherParam param;
  ayanami::CallbackSubscription cb_velocity_cmd;
};

}  // namespace ayanami_ros
