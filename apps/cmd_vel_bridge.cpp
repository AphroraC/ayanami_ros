// cmd_vel_bridge: ayanami 输出的 geometry_msgs/Twist (/cmd_vel) 适配 rm 底盘
// 控制器的 rm_msgs/ChassisCmd (/cmd_chassis)。
//
// 为什么需要: rm_chassis_controllers 的 ChassisBase 同时订阅 /cmd_vel (速度)
// 与 /cmd_chassis (模式/斜率加速度/功率上限)。从未收到 /cmd_chassis 时
// cmd_chassis.accel 恒为 0, 而 ChassisBase::update 用 RampFilter(acc*dt) 对
// cmd_vel 做斜率限制 —— acc=0 意味着速度命令永远无法从 0 爬升, 机器人不动。
// 本节点把 mode / accel / power_limit 以参数注入, 每收到一条 /cmd_vel 转发一条
// /cmd_chassis (ChassisBase 的命令超时戳只由 /cmd_vel 回调刷新, 无需额外
// 定时器; /cmd_chassis 回调不更新戳, 只更新 accel/mode)。
//
// 参数 (private ~):
// cmd_vel_topic (默认 /cmd_vel)
// cmd_chassis_topic (默认 /cmd_chassis)
// mode (默认 0 = RAW; FOLLOW=1 需要 follow_source_frame 语义)
// accel_linear_x (默认 3.0 m/s^2)
// accel_linear_y (默认 3.0 m/s^2)
// accel_angular_z (默认 6.0 rad/s^2)
// power_limit (默认 100.0 W; gazebo 版 powerLimit 为空, 真机生效)

#include <geometry_msgs/Twist.h>
#include <rm_msgs/ChassisCmd.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmd_vel_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  const std::string cmd_vel_topic =
      private_nh.param<std::string>("cmd_vel_topic", "/cmd_vel");
  const std::string cmd_chassis_topic =
      private_nh.param<std::string>("cmd_chassis_topic", "/cmd_chassis");
  const auto mode =
      static_cast<std::uint8_t>(private_nh.param<int>("mode", 0));
  const double accel_linear_x =
      private_nh.param<double>("accel_linear_x", 3.0);
  const double accel_linear_y =
      private_nh.param<double>("accel_linear_y", 3.0);
  const double accel_angular_z =
      private_nh.param<double>("accel_angular_z", 6.0);
  const double power_limit = private_nh.param<double>("power_limit", 100.0);

  ros::Publisher cmd_chassis_pub =
      nh.advertise<rm_msgs::ChassisCmd>(cmd_chassis_topic, 1);

  ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(
      cmd_vel_topic, 1,
      [&](const geometry_msgs::Twist::ConstPtr& msg) {
        rm_msgs::ChassisCmd cmd;
        cmd.mode = mode;
        cmd.accel.linear.x = accel_linear_x;
        cmd.accel.linear.y = accel_linear_y;
        cmd.accel.angular.z = accel_angular_z;
        cmd.power_limit = power_limit;
        cmd_chassis_pub.publish(cmd);
      });

  ros::spin();
  return 0;
}
