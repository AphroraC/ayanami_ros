#pragma once

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>

#include <mutex>
#include <string>

namespace ayanami_ros {

// Core -> ROS output bridge: the ayanami_ros body only reads ROS data and
// feeds it into the core; all core outputs (the four action
// goal/feedback/result kinds, cmd_vel) are read through this extension's
// NavigationCallbacks global callback subscriptions and published as the
// corresponding ROS messages. This module derives directly from
// ayanami::ExtensionModule; at construction time it creates its own
// ros::NodeHandle and adds subscriptions (triggered by the dlopen in
// AyanamiROS::loadExtensionModules, before navigation_module->start,
// satisfying the ordering); at destruction time it removes the callback ids.
// Costmap publishing lives in the dedicated costmap_viewer extension.
class RvizViewer final : public ayanami::ExtensionModule {
 public:
  RvizViewer();
  ~RvizViewer() override;

  /// @brief Parameters arrive AFTER a successful load, injected by the loader
  /// (ModuleContext = logical name + the extensions config). The section key is
  /// this module's own name, so the config layout stays outside the module --
  /// it no longer resolves the config path itself.
  void configureParameters(const ayanami::ModuleContext& context) override;

 private:
  void readConfig(const ayanami::ModuleContext& context);
  void advertiseTopics();
  void registerCallbacks();
  void unregisterCallbacks();

  void publishPlannerGoal(const ayanami::actions::GetPathGoal& goal);
  void publishPlannerFeedback(
      const ayanami::actions::GetPathFeedback& feedback);
  void publishPlannerPath(
      const ayanami::actions::GoalResult<ayanami::actions::GetPathResult>&
          result);
  void publishSmootherGoal(const ayanami::actions::SmoothPathGoal& goal);
  void publishSmootherFeedback(
      const ayanami::actions::SmoothPathFeedback& feedback);
  void publishSmootherPath(
      const ayanami::actions::GoalResult<ayanami::actions::SmoothPathResult>&
          result);
  void publishExePathGoal(const ayanami::actions::ExePathGoal& goal);
  void publishExePathFeedback(
      const ayanami::actions::ExePathFeedback& feedback);
  void publishExePathResult(
      const ayanami::actions::GoalResult<ayanami::actions::ExePathResult>&
          result);
  void publishRecoveryGoal(const ayanami::actions::RecoveryGoal& goal);
  void publishRecoveryFeedback(
      const ayanami::actions::RecoveryFeedback& feedback);
  void publishRecoveryResult(
      const ayanami::actions::GoalResult<ayanami::actions::RecoveryResult>&
          result);
  void publishNavigationResult(
      const ayanami::actions::NavigationResult& result);

  ros::NodeHandle nh;
  ros::Publisher planner_path_pub, smoother_path_pub,
      planner_goal_pub,
      planner_feedback_pub, smoother_input_path_pub, smoother_feedback_pub,
      controller_path_pub, controller_feedback_pub, controller_status_pub,
      recovery_goal_pub, recovery_feedback_pub, recovery_status_pub,
      navigation_status_pub, cmd_vel_pub;
  std::string cmd_vel_topic = "/cmd_vel";
  std::string planner_path_topic = "/ayanami/planner/path";
  std::string smoother_path_topic = "/ayanami/smoother/path";
  std::string planner_goal_topic = "/ayanami/planner/goal";
  std::string planner_feedback_topic = "/ayanami/planner/feedback";
  std::string smoother_input_path_topic = "/ayanami/smoother/input_path";
  std::string smoother_feedback_topic = "/ayanami/smoother/feedback";
  std::string controller_path_topic = "/ayanami/controller/path";
  std::string controller_feedback_topic = "/ayanami/controller/feedback";
  std::string controller_status_topic = "/ayanami/controller/status";
  std::string recovery_goal_topic = "/ayanami/recovery/goal";
  std::string recovery_feedback_topic = "/ayanami/recovery/feedback";
  std::string recovery_status_topic = "/ayanami/recovery/status";
  std::string navigation_status_topic = "/ayanami/navigation_status";
  std::mutex last_recovery_mutex;
  std::string last_recovery_id;
  ayanami::CallbackSubscription cb_get_path, cb_smooth_path, cb_velocity_cmd,
      cb_get_path_goal,
      cb_get_path_feedback, cb_smooth_goal, cb_smooth_feedback, cb_exe_goal,
      cb_exe_feedback, cb_exe_result, cb_recovery_goal, cb_recovery_feedback,
      cb_recovery_result, cb_navigation_result;
};

}  // namespace ayanami_ros
