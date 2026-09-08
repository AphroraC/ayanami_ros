// cmd_chassis_publisher ROS extension .so (dlopen -> create_extension_module
// -> CmdChassisPublisher : ayanami::ExtensionModule).
//
// Sole chassis command publisher: subscribes the core NavigationCallbacks::
// on_velocity_command slot and republishes rm_msgs/ChassisCmd on /cmd_chassis
// (mode / ramped accel / power limit parameters, rm_chassis_controllers
// ChassisBase semantics). Knows
// nothing about any other plugin or algorithm state -- pure output bridge.
// rm_msgs is allowed HERE (rviz_viewer must stay rm_msgs-free).
//
// Parameters: config_extensions.json::cmd_chassis_publisher
//   cmd_chassis_topic (default /cmd_chassis)
//   mode (default 0 = RAW)
//   accel_linear_x / accel_linear_y (default 3.0 m/s^2)
//   accel_angular_z (default 6.0 rad/s^2)
//   power_limit (default 100.0 W; effective on real hardware only)

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>
#include <rm_msgs/ChassisCmd.h>
#include <ros/ros.h>

#include <atomic>
#include <string>

#include <spdlog/spdlog.h>

namespace ayanami_ros {

namespace {
// Test count hook (cmd_chassis_publisher_test reads it via dlsym).
std::atomic<int> g_chassis_cmd_pub_count{0};
}  // namespace

class CmdChassisPublisher final : public ayanami::ExtensionModule {
 public:
  CmdChassisPublisher() : nh("~") {
    // Params single source = config_extensions.json::cmd_chassis_publisher
    // (asuka pattern); missing file -> defaults.
    const ayanami::Config extensions_config(
        ayanami::GlobalConfig::getConfigPath("config_extensions"));
    const std::string sec = "cmd_chassis_publisher";
    cmd_chassis_topic = extensions_config.param<std::string>(
        sec, "cmd_chassis_topic", "/cmd_chassis");
    mode = static_cast<std::uint8_t>(
        extensions_config.param<int>(sec, "mode", 0));
    accel_linear_x = extensions_config.param<double>(sec, "accel_linear_x", 3.0);
    accel_linear_y = extensions_config.param<double>(sec, "accel_linear_y", 3.0);
    accel_angular_z =
        extensions_config.param<double>(sec, "accel_angular_z", 6.0);
    power_limit = extensions_config.param<double>(sec, "power_limit", 100.0);

    // Global topic: advertised on the root namespace handle.
    ros::NodeHandle root;
    cmd_chassis_pub =
        root.advertise<rm_msgs::ChassisCmd>(cmd_chassis_topic, 1);

    cb_velocity_cmd =
        ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
            [this](const ayanami::geometry_msgs::TwistStamped& velocity) {
              publishChassisCommand(velocity);
            });

    spdlog::info("cmd_chassis_publisher: topic={} mode={} accel=({},{},{}) "
                 "power_limit={}",
                 cmd_chassis_topic, mode, accel_linear_x, accel_linear_y,
                 accel_angular_z, power_limit);
  }

  ~CmdChassisPublisher() override { cb_velocity_cmd.reset(); }

 private:
  void publishChassisCommand(
      const ayanami::geometry_msgs::TwistStamped& velocity) {
    (void)velocity;
    // ChassisCmd carries mode/ramped-accel/power limit only (velocity
    // itself flows through the chassis controller's own command channel).
    // One ChassisCmd per received core velocity command keeps the command
    // timeout stamp fresh.
    g_chassis_cmd_pub_count.fetch_add(1);
    rm_msgs::ChassisCmd cmd;
    cmd.mode = mode;
    cmd.accel.linear.x = accel_linear_x;
    cmd.accel.linear.y = accel_linear_y;
    cmd.accel.angular.z = accel_angular_z;
    cmd.power_limit = power_limit;
    cmd_chassis_pub.publish(cmd);
  }

  ros::NodeHandle nh;
  ros::Publisher cmd_chassis_pub;
  std::string cmd_chassis_topic{"/cmd_chassis"};
  std::uint8_t mode{0};
  double accel_linear_x{3.0};
  double accel_linear_y{3.0};
  double accel_angular_z{6.0};
  double power_limit{100.0};
  ayanami::CallbackSubscription cb_velocity_cmd;
};

}  // namespace ayanami_ros

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami_ros::CmdChassisPublisher();
}

// ---- TDD test hook (cmd_chassis_publisher_test reads it via dlsym) ----
extern "C" int cmdChassisPubCount() {
  return ayanami_ros::g_chassis_cmd_pub_count.load();
}
