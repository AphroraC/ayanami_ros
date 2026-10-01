#include "cmd_chassis_publisher.hpp"

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

#include <atomic>

namespace ayanami_ros {

namespace {
// Test count hook (cmd_chassis_publisher_test reads it via dlsym).
std::atomic<int> g_chassis_cmd_pub_count{0};
}  // namespace

CmdChassisPublisher::CmdChassisPublisher() {
  cb_velocity_cmd =
      ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
          [this](const ayanami::geometry_msgs::TwistStamped& velocity) {
            publishChassisCommand(velocity);
          });
}

CmdChassisPublisher::~CmdChassisPublisher() { cb_velocity_cmd.reset(); }

void CmdChassisPublisher::configureParameters(
    const ayanami::ModuleContext& context) {
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
  cmd_chassis_pub = root.advertise<rm_msgs::ChassisCmd>(param.cmd_chassis_topic, 1);
  SPDLOG_LOGGER_INFO(
    ayanami::rosLogger(),
    "cmd_chassis_publisher: topic={} mode={} accel=({},{},{}) "
    "power_limit={}",
    param.cmd_chassis_topic,
    param.mode,
    param.accel_linear_x,
    param.accel_linear_y,
    param.accel_angular_z,
    param.power_limit);
}

void CmdChassisPublisher::publishChassisCommand(
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

}  // namespace ayanami_ros

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami_ros::CmdChassisPublisher();
}

// ---- TDD test hook (cmd_chassis_publisher_test reads it via dlsym) ----
extern "C" int cmdChassisPubCount() {
  return ayanami_ros::g_chassis_cmd_pub_count.load();
}
