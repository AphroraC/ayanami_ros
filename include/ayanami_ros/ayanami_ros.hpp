#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Geometry>

#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/sensor_msgs.hpp>
#include <ayanami/messages/timestamp.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <ayanami_ros/tf2_listener.hpp>

namespace ayanami_ros {

/**
 * @brief ayanami ROS1 boundary host node.
 *
 * Responsibilities: load the core AbstractNavigation host module (dlopen
 * libayanami_costmap_navigation.so) + load ayanami::ExtensionModule
 * extensions (rviz_viewer etc.), convert ROS topics -> core calls, and
 * drive the periodic spinOnce.
 *
 * TF model (on-demand bridge): the TF2Listener's auto-subscription feeds
 * the ROS tf2 buffer; at construction AyanamiROS installs that listener
 * into the core RobotStateController as the injected TF source. Core
 * threads query TF on demand (stamp-time interpolation included) -- there
 * is no periodic sampling into a core-side store. The loop only runs the
 * low-frequency TF health diagnostics.
 *
 * Key ordering (verbatim): loadExtensionModules -> loadNavigationModule ->
 * subscribeTopics -> loop_thread start. Extensions must register their
 * NavigationCallbacks subscriptions before navigation_module->start(),
 * otherwise the first wave of startup visualization emits is missed.
 */
class AyanamiROS {
public:
  explicit AyanamiROS(ros::NodeHandle& nh);
  ~AyanamiROS();

  void start();
  void stop();

  // ---- test/host support getters (brief: getter for tests) ----
  const std::vector<ayanami::ExtensionModule::Ptr>& extensionModules() const { return extension_modules; }
  ayanami::abstract::AbstractNavigation::Ptr navigationModule() const { return navigation_module; }
  const std::string& configPath() const { return config_path; }

  // ---- input wiring (core boundary, used by ROS callbacks / loop / tests) ----
  /// @brief Writes the odom velocity directly into the core
  /// RobotStateController (pose reads go through the injected tf2 source).
  void setOdometry(const ayanami::geometry_msgs::TwistStamped& twist);
  /// @brief Obstacle observation input: point cloud (delivered directly to
  /// the navigation host).
  void submitSensor(const ayanami::sensor_msgs::PointCloud2& cloud);
  /// @brief Obstacle observation input: laser scan (same as above).
  void submitSensor(const ayanami::sensor_msgs::LaserScan& scan);
  /// @brief latest-wins goal mailbox delivery (called after conversion in the
  /// ROS goalCallback; overwrites the pending goal, no queueing).
  void submitGoal(const ayanami::geometry_msgs::PoseStamped& goal);
  /// @brief loop-thread side: takes the current latest goal (if any) and calls
  /// navigation_module->setGoal.
  /// @return true if a goal was actually dispatched.
  bool drainPendingGoal();
  /// @brief Test/observation: whether a pending goal remains in the mailbox.
  bool hasPendingGoal() const;

private:
  void loadExtensionModules();
  void loadNavigationModule(const ayanami::Config& config_ros);
  void subscribeTopics();
  /// @brief Install the tf2-backed lookup into the core RobotStateController
  /// (the bridge: core threads query TF on demand, ayanami_ros answers via
  /// the tf2 buffer).
  void installTfSource();
  /// @brief Refuse to start when config_ros's robot_frame mirror disagrees with
  /// the core's single source (config_navigation.json::navigation.robot_frame).
  /// Throws std::runtime_error on mismatch or when either side is unreadable.
  void verifyRobotFrameMatchesNavigation();
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  /// @brief ROS TF tree health diagnostics (called at low frequency by the
  /// loop): checks whether the configured frame-chain edges
  /// (map<-odom, odom<-robot, map<-robot) are resolvable on the ROS side;
  /// on failure, logs a warning once with the full frame tree (to pin down
  /// environment / leftover-instance problems).
  void checkTfHealth();
  void loop();

private:
  ros::NodeHandle& nh;
  std::string config_path;

  ayanami::abstract::AbstractNavigation::Ptr navigation_module;
  // Extension modules all derive from ayanami::ExtensionModule (no
  // ROS-specific subclasses): input belongs to AyanamiROS; output publishing is
  // done by the extension itself (e.g. rviz_viewer) at construction time.
  std::vector<ayanami::ExtensionModule::Ptr> extension_modules;

  ayanami_ros::TF2Listener tf_listener;

  ros::Subscriber goal_sub, pointcloud_sub, odom_sub, laserscan_sub;

  std::atomic<bool> kill_switch{false};
  std::thread loop_thread;
  // latest-wins goal mailbox: the ROS callback only converts + overwrites
  // pending_goal; each round the loop thread takes the current latest one and
  // dispatches setGoal (no queueing, no unhandled goal is lost).
  mutable std::mutex goal_mutex;
  std::optional<ayanami::geometry_msgs::PoseStamped> pending_goal;
  // TF health diagnostic state (used by checkTfHealth).
  bool tf_health_was_bad{false};
  std::chrono::steady_clock::time_point tf_health_last_log{};

  // config_ros.json::ayanami_ros boundary parameters.
  std::string laserscan_topic, pointcloud_topic, odom_topic, goal_topic;
  std::string map_frame, odom_frame, robot_frame;
  std::string navigation_so_name;
};

}  // namespace ayanami_ros
