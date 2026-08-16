#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <ayanami/core/abstract_navigation.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/utility/config.hpp>

#include <ayanami_ros/extension_module_ros.hpp>
#include <ayanami_ros/tf2_listener.hpp>

namespace ayanami_ros {

/**
 * @brief ayanami ROS1 边界宿主节点 (spec §4.1-§4.2)。
 *
 * 职责: 加载核心 AbstractNavigation 宿主插件 (dlopen libayanami_costmap_navigation.so)
 * + 加载 ExtensionModuleROS 扩展 (rviz_viewer 等), 把 ROS topics → 核心 callback
 * 转换, tf sync (T7 实接) 与 spinOnce 周期驱动。
 *
 * 关键顺序 (spec §4.2 verbatim): loadExtensionModules → loadNavigationPlugin →
 * subscribeTopics → loop_thread_ 启。扩展必须先于 navigation_plugin->start() 注册
 * NavigationCallbacks 订阅, 否则启动期第一波可视化 emit 会被错过 (M2-P1 ruling)。
 */
class AyanamiROS {
  ros::NodeHandle& nh_;
  std::string config_path_;

  ayanami::abstract::AbstractNavigation::Ptr navigation_plugin_;
  std::vector<ExtensionModuleROS::Ptr> extension_modules_;
  std::vector<ros::Subscriber> extension_subs_;

  TF2Listener tf_listener_;

  ros::Subscriber goal_sub_, pointcloud_sub_, odom_sub_, laserscan_sub_;
  ayanami::geometry_msgs::TwistStamped last_odom_;
  std::mutex odom_mutex_;

  std::atomic<bool> kill_switch_{false};
  std::thread loop_thread_;
  std::mutex goal_mutex_;
  double tf_sync_rate_ = 50.0;

  // config_ros.json::ayanami_ros 边界参数 (spec §8.1)。
  std::string laserscan_topic_, pointcloud_topic_, odom_topic_, goal_topic_;
  std::string map_frame_, odom_frame_, robot_frame_;
  std::string navigation_so_name_, navigation_create_symbol_;

public:
  explicit AyanamiROS(ros::NodeHandle& nh);
  ~AyanamiROS();

  void start();
  void stop();

  // ---- 测试/宿主支撑 getter (brief: getter for tests) ----
  const std::vector<ExtensionModuleROS::Ptr>& extensionModules() const {
    return extension_modules_;
  }
  ayanami::abstract::AbstractNavigation::Ptr navigationPlugin() const {
    return navigation_plugin_;
  }
  const std::string& configPath() const { return config_path_; }

private:
  void loadExtensionModules(const ayanami::Config& config_ros);
  void loadNavigationPlugin(const ayanami::Config& config_ros);
  void subscribeTopics();
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void syncTransformsToCore();
  void loop();
};

}  // namespace ayanami_ros
