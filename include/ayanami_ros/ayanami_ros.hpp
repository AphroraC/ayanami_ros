#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <ayanami_ros/tf2_listener.hpp>

namespace ayanami_ros {

/**
 * @brief ayanami ROS1 边界宿主节点 (spec §4.1-§4.2)。
 *
 * 职责: 加载核心 AbstractNavigation 宿主插件 (dlopen
 * libayanami_costmap_navigation.so)
 * + 加载 ayanami::ExtensionModule 扩展 (rviz_viewer 等), 把 ROS topics → 核心
 * callback 转换, tf sync (T7 实接) 与 spinOnce 周期驱动。
 *
 * 关键顺序 (spec §4.2 verbatim): loadExtensionModules → loadNavigationPlugin →
 * subscribeTopics → loop_thread 启。扩展必须先于 navigation_plugin->start()
 * 注册 NavigationCallbacks 订阅, 否则启动期第一波可视化 emit 会被错过 (M2-P1
 * ruling)。
 */
class AyanamiROS {
public:
  explicit AyanamiROS(ros::NodeHandle& nh);
  ~AyanamiROS();

  void start();
  void stop();

  // ---- 测试/宿主支撑 getter (brief: getter for tests) ----
  const std::vector<ayanami::ExtensionModule::Ptr>& extensionModules() const { return extension_modules; }
  ayanami::abstract::AbstractNavigation::Ptr navigationPlugin() const { return navigation_plugin; }
  const std::string& configPath() const { return config_path; }

  // ---- 输入接线支持 (供 callbacks / loop / 测试) ----
  /// @brief latest-wins goal 邮箱投递 (ROS goalCallback 转换后调用; 覆写待发
  /// goal, 不排队)。
  void submitGoal(const ayanami::geometry_msgs::PoseStamped& goal);
  /// @brief loop 线程侧: 取当前最新 goal (若有) 调 navigation_plugin->setGoal。
  /// @return true 表示确实下发了一个 goal。
  bool drainPendingGoal();
  /// @brief 测试/观察: 邮箱内是否还有待发 goal。
  bool hasPendingGoal() const;
  /// @brief odom 速度直写全局 RobotStateController (ROS odomCallback 转换后
  /// 调用; 位姿读取仍由核心 controller execution 经 TF 查询)。
  void updateOdomVelocity(const ayanami::geometry_msgs::TwistStamped& twist);

private:
  void loadExtensionModules(const ayanami::Config& config_ros);
  void loadNavigationPlugin(const ayanami::Config& config_ros);
  void subscribeTopics();
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void syncTransformsToCore();
  void syncTransformsToCore(const ros::Time& stamp, double timeout);
  void loop();

private:
  ros::NodeHandle& nh;
  std::string config_path;

  ayanami::abstract::AbstractNavigation::Ptr navigation_plugin;
  // 扩展模块统一继承 ayanami::ExtensionModule (无 ROS 特有子类): 输入归
  // AyanamiROS, 输出发布由 rviz_viewer 之类的扩展自身在构造期完成。
  std::vector<ayanami::ExtensionModule::Ptr> extension_modules;

  ayanami_ros::TF2Listener tf_listener;

  ros::Subscriber goal_sub, pointcloud_sub, odom_sub, laserscan_sub;

  std::atomic<bool> kill_switch{false};
  std::thread loop_thread;
  // latest-wins goal 邮箱 (spec §4.2): ROS 回调只转换 + 覆写 pending_goal,
  // loop 线程每次取当前最新一条下发 setGoal (不排队, 不丢未处理)。
  mutable std::mutex goal_mutex;
  std::optional<ayanami::geometry_msgs::PoseStamped> pending_goal;
  std::mutex tf_edge_mutex;
  std::map<std::pair<std::string, std::string>, bool> tf_edge_static;
  std::set<std::pair<std::string, std::string>> configured_static_edges;
  double tf_sync_rate = 50.0;

  // config_ros.json::ayanami_ros 边界参数 (spec §8.1)。
  std::string laserscan_topic, pointcloud_topic, odom_topic, goal_topic;
  std::string map_frame, odom_frame, robot_frame;
  std::string navigation_so_name;
};

}  // namespace ayanami_ros
