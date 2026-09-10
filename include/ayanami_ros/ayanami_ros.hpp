#pragma once

#include <atomic>
#include <chrono>
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
#include <tf2_msgs/TFMessage.h>

#include <Eigen/Geometry>

#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/sensor_msgs.hpp>
#include <ayanami/messages/timestamp.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <ayanami_ros/tf2_listener.hpp>

namespace ayanami_ros {

/**
 * @brief ayanami ROS1 boundary host node (-§4.2).
 *
 * Responsibilities: load the core AbstractNavigation host module (dlopen
 * libayanami_costmap_navigation.so)
 * + load ayanami::ExtensionModule extensions (rviz_viewer etc.), convert ROS
 * topics -> core callbacks, and drive tf sync plus the periodic spinOnce.
 *
 * Key ordering (verbatim): loadExtensionModules -> loadNavigationModule ->
 * subscribeTopics -> loop_thread start. Extensions must register their
 * NavigationCallbacks subscriptions before navigation_module->start(),
 * otherwise the first wave of startup visualization emits is missed
 * (ruling).
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
  /// @brief TF edge write: the TransformStamped looked up on the ROS side is
  /// written directly into the core RobotStateController (the core's only tf
  /// query provider).
  void setTransform(const std::string& parent, const std::string& child,
                    ayanami::Timestamp stamp,
                    const Eigen::Isometry3d& transform, bool is_static);
  /// @brief Writes the odom velocity directly into the core
  /// RobotStateController (pose reads are still done by the core execution
  /// layer via TF queries).
  void setOdometry(const ayanami::geometry_msgs::TwistStamped& twist);
  /// @brief Obstacle observation input: point cloud (syncs currently available
  /// TF edges first, then delivers directly to the navigation host).
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
  /// @brief Refuse to start when config_ros's robot_frame mirror disagrees with
  /// the core's single source (config_navigation.json::navigation.robot_frame).
  /// Throws std::runtime_error on mismatch or when either side is unreadable.
  void verifyRobotFrameMatchesNavigation();
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void syncTransformsToCore();
  void syncTransformsToCore(const ros::Time& stamp, double timeout);
  /// @brief /tf message callback (event-driven sync): each message immediately
  /// triggers one full-edge sync, cutting the ROS TF -> core latency from half
  /// a polling period down to message-processing time.
  void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);
  /// @brief ROS TF tree health diagnostics (called at low frequency by the
  /// loop): checks whether each tf_sync_edges edge is resolvable on the ROS
  /// side; on failure, logs a warning once with the full frame tree (to pin
  /// down environment / leftover-instance problems).
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
  // Event-driven TF sync: dedicated /tf callback queue + thread (bypasses the
  // 50Hz loop's ros::spinOnce, otherwise the callback rate is clamped by the
  // polling ceiling); sync_mutex serializes concurrent syncs from the three
  // entry points: tf thread / loop thread / sensor callbacks.
  ros::CallbackQueue tf_queue;
  ros::NodeHandle tf_nh;
  ros::Subscriber tf_sub;
  std::thread tf_queue_thread;
  std::mutex sync_mutex;
  // latest-wins goal mailbox: the ROS callback only converts + overwrites
  // pending_goal; each round the loop thread takes the current latest one and
  // dispatches setGoal (no queueing, no unhandled goal is lost).
  mutable std::mutex goal_mutex;
  std::optional<ayanami::geometry_msgs::PoseStamped> pending_goal;
  std::mutex tf_edge_mutex;
  std::map<std::pair<std::string, std::string>, bool> tf_edge_static;
  /// @brief Per-edge last sync timestamp (rate gating for tf_sync_edges).
  /// Lazily resized to tfSyncEdges().size() (edges are fixed after
  /// construction). Guarded by sync_mutex.
  std::vector<std::chrono::steady_clock::time_point> edge_last_sync;
  // TF missing-edge diagnostics (maintained by the 50Hz loop): first failure
  // time per edge + last diagnostic log time. The sensor callback path does not
  // block (timeout=0); diagnostics are only emitted by the loop's low-frequency
  // path.
  std::map<std::pair<std::string, std::string>,
           std::chrono::steady_clock::time_point>
      tf_missing_since;
  std::map<std::pair<std::string, std::string>,
           std::chrono::steady_clock::time_point>
      tf_diag_logged_at;
  // TF health diagnostic state (used by checkTfHealth).
  bool tf_health_was_bad{false};
  std::chrono::steady_clock::time_point tf_health_last_log{};
  std::set<std::pair<std::string, std::string>> configured_static_edges;
  double tf_sync_rate = 50.0;

  // config_ros.json::ayanami_ros boundary parameters.
  std::string laserscan_topic, pointcloud_topic, odom_topic, goal_topic;
  std::string map_frame, odom_frame, robot_frame;
  // Global dynamic costmap rebuild trigger (odom<-base_footprint edge):
  // accumulates robot rotation/translation "since the last clear". Rotation is
  // the main source of marker misalignment -- wheeled odometry slips while
  // rotating in place and under-reports the turned angle (~6%/rad measured),
  // while AMCL corrections catch up at a finite rate; in the meantime scans are
  // rasterized with the lagging transform and the global map's dynamic markers
  // drift out of place; once accumulated rotation/translation exceeds the
  // threshold, clear and rebuild.
  // Note: the map<-odom correction increment must not be used as the signal --
  // AMCL corrections are gradual and may not converge for a long time (never
  // converges with recovery injection disabled); the correction increment stays
  // small, so the misalignment accumulation goes undetected.
  bool localization_jump_prev_valid{false};
  double localization_clear_yaw_threshold{1.0};  // rad, accumulated since last clear
  double localization_clear_pos_threshold{2.0};  // m, accumulated since last clear
  double localization_accum_yaw{0.0};
  double localization_accum_pos{0.0};
  Eigen::Isometry3d localization_jump_prev{Eigen::Isometry3d::Identity()};
  std::mutex localization_jump_mutex;
  std::string navigation_so_name;
};

}  // namespace ayanami_ros
