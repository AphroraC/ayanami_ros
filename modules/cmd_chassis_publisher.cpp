// cmd_chassis_publisher ROS extension .so (dlopen -> create_extension_module
// -> CmdChassisPublisher : ayanami::ExtensionModule).
//
// Sole chassis command publisher: subscribes the core NavigationCallbacks::
// on_velocity_command slot and republishes rm_msgs/ChassisCmd on /cmd_chassis
// (mode / ramped accel / power limit parameters, rm_chassis_controllers
// ChassisBase semantics). Knows
// nothing about any other module or algorithm state -- pure output bridge.
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
    cb_velocity_cmd =
        ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
            [this](const ayanami::geometry_msgs::TwistStamped& velocity) {
              publishChassisCommand(velocity);
            });
  }

  ~CmdChassisPublisher() override { cb_velocity_cmd.reset(); }

  /// @brief Parameters arrive AFTER a successful load, injected by the loader
  /// (ModuleContext = logical name + the extensions config). The module no
  /// longer resolves the config path itself: the section key is its own name,
  /// so the config layout stays outside the module.
  void configureParameters(const ayanami::ModuleContext& context) override {
    const ayanami::Config& file = context.config;
    const std::string& section = context.name;
    param.cmd_chassis_topic = file.param<std::string>(
        section, "cmd_chassis_topic", param.cmd_chassis_topic);
    param.mode = static_cast<std::uint8_t>(
        file.param<int>(section, "mode", param.mode));
    param.accel_linear_x =
        file.param<double>(section, "accel_linear_x", param.accel_linear_x);
    param.accel_linear_y =
        file.param<double>(section, "accel_linear_y", param.accel_linear_y);
    param.accel_angular_z =
        file.param<double>(section, "accel_angular_z", param.accel_angular_z);
    param.power_limit =
        file.param<double>(section, "power_limit", param.power_limit);

    // The topic comes from the parameters, so the publisher is created here
    // rather than in the constructor (a ros::Publisher cannot change its topic
    // afterwards). Still before navigation_module->start(), which the node
    // orders after extension loading.
    ros::NodeHandle root;
    cmd_chassis_pub =
        root.advertise<rm_msgs::ChassisCmd>(param.cmd_chassis_topic, 1);
    spdlog::info(
        "cmd_chassis_publisher: topic={} mode={} accel=({},{},{}) "
        "power_limit={}",
        param.cmd_chassis_topic, param.mode, param.accel_linear_x,
        param.accel_linear_y, param.accel_angular_z, param.power_limit);
  }

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
    cmd.mode = param.mode;
    cmd.accel.linear.x = param.accel_linear_x;
    cmd.accel.linear.y = param.accel_linear_y;
    cmd.accel.angular.z = param.accel_angular_z;
    cmd.power_limit = param.power_limit;
    cmd_chassis_pub.publish(cmd);
  }

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

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami_ros::CmdChassisPublisher();
}

// ---- TDD test hook (cmd_chassis_publisher_test reads it via dlsym) ----
extern "C" int cmdChassisPubCount() {
  return ayanami_ros::g_chassis_cmd_pub_count.load();
}
