#pragma once

#include <actionlib/server/action_server.h>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>
#include <mbf_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>

namespace ayanami_ros {

// mbf_action_server ROS extension (dlopen -> create_extension_module ->
// MbfActionServer : ayanami::ExtensionModule).
//
// Compatibility layer for the rm_behavior_tree / new_behavior_tree clients:
// an actionlib ActionServer<mbf_msgs::MoveBaseAction> under the historical
// name /move_base_flex/move_base, so the BT's SimpleActionClient keeps
// working unchanged while the navigation stack behind it is ayanami.
//
// Wiring: goal -> NavigationInputs::on_goal_request (AyanamiROS forwards it
// into the same latest-wins goal mailbox the ROS goal topic feeds);
// NavigationCallbacks::on_navigation_result -> action terminal state
// (SUCCEEDED / CANCELED / ABORTED with an MBF outcome mapped from
// FailureContext.stage).
//
// Known semantic gaps (deliberate, documented):
// - goal.direct_track / controller / planner / recovery_behaviors are ignored:
//   the ayanami goal mailbox carries a bare PoseStamped.
// - Cancel has no core counterpart: a cancel request is answered by emitting a
//   "current pose" goal (planner reaches it immediately, the controller stops
//   tracking) -- see emitCurrentPoseGoal(). Replace with the core cancel API
//   once one exists.
// - result.feedback is never published (no BT consumer).
class MbfActionServer final : public ayanami::ExtensionModule {
 public:
  MbfActionServer();
  ~MbfActionServer() override;

  /// @brief Parameters arrive AFTER a successful load, injected by the loader
  /// (ModuleContext = logical name + the extensions config). The action name
  /// comes from the parameters, so the ActionServer (whose topics cannot
  /// change after construction) is created here, still before
  /// navigation_module->start().
  void configureParameters(const ayanami::ModuleContext& context) override;

 private:
  using GoalHandle = actionlib::ServerGoalHandle<mbf_msgs::MoveBaseAction>;

  void goalCallback(GoalHandle goal_handle);
  void cancelCallback(GoalHandle goal_handle);
  void handleNavigationResult(const ayanami::actions::NavigationResult& result);
  /// @brief Cancel workaround: submit the robot's current pose as a goal so
  /// the planner/controller wind down. Best effort (TF may be unavailable).
  void emitCurrentPoseGoal();

  /// @brief Every module parameter lives here, with its default value as the
  /// field initializer.
  struct MbfActionServerParam {
    std::string action_name{"/move_base_flex/move_base"};
    std::string map_frame{"map"};
    std::string base_frame{"base_link"};
    double stop_goal_tf_timeout{0.1};
  };

  ros::NodeHandle nh;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener{tf_buffer};
  std::unique_ptr<actionlib::ActionServer<mbf_msgs::MoveBaseAction>> action_server;

  MbfActionServerParam param;

  ayanami::CallbackSubscription cb_navigation_result;

  // At most one action goal may be in flight: the navigation core is
  // latest-wins, so a new goal preempts the previous handle here first.
  // Guarded by goal_handle_mutex; an invalid handle means "no action goal
  // active" (navigations started via the plain goal topic report results that
  // are simply ignored).
  std::mutex goal_handle_mutex;
  GoalHandle active_goal_handle;
};

}  // namespace ayanami_ros
