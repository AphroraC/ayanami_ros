#include "mbf_action_server.hpp"

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami_ros/ros_msg_converter.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ayanami_ros {

namespace {

// FailureStage -> MBF outcome. The BT's reachGoalJudgement only checks for
// SUCCESS, so the precise error code is diagnostic; keep the mapping to the
// coarse MBF families rather than inventing codes.
std::uint32_t mapOutcomeToMbf(const ayanami::FailureContext& context) {
  switch (context.stage) {
    case ayanami::FailureStage::Planner:
      return mbf_msgs::MoveBaseResult::PLAN_FAILURE;
    case ayanami::FailureStage::Controller:
      return mbf_msgs::MoveBaseResult::CTRL_FAILURE;
    default:
      return mbf_msgs::MoveBaseResult::FAILURE;
  }
}

std::string describeContext(const ayanami::FailureContext& context) {
  return "[" + context.module_id + "] " + context.message;
}

}  // namespace

MbfActionServer::MbfActionServer() {
  // The core emits on_navigation_result exactly once per navigation attempt
  // (terminal states only); subscribing here keeps the hook live before
  // navigation_module->start(), per the node's loading order.
  cb_navigation_result =
      ayanami::interfaces::NavigationCallbacks::on_navigation_result.subscribe(
          [this](const ayanami::actions::NavigationResult& result) {
            handleNavigationResult(result);
          });
}

MbfActionServer::~MbfActionServer() {
  cb_navigation_result.reset();
  std::lock_guard<std::mutex> lock(goal_handle_mutex);
  if (active_goal_handle.isValid()) {
    mbf_msgs::MoveBaseResult result;
    result.outcome = mbf_msgs::MoveBaseResult::CANCELED;
    result.message = "ayanami action server shutting down";
    active_goal_handle.setCanceled(result, result.message);
    active_goal_handle = GoalHandle();
  }
}

void MbfActionServer::configureParameters(const ayanami::ModuleContext& context) {
  const ayanami::Config& file = context.config;
  const std::string& section = context.name;
  param.action_name =
      file.param<std::string>(section, "action_name", param.action_name);
  param.map_frame = file.param<std::string>(section, "map_frame", param.map_frame);
  param.base_frame = file.param<std::string>(section, "base_frame", param.base_frame);
  param.stop_goal_tf_timeout =
      file.param<double>(section, "stop_goal_tf_timeout", param.stop_goal_tf_timeout);

  // auto_start=false + explicit start(): constructing an advertised-but-not-
  // started server can drop early goals with a confusing status.
  action_server = std::make_unique<actionlib::ActionServer<mbf_msgs::MoveBaseAction>>(
      nh, param.action_name,
      [this](GoalHandle goal_handle) { goalCallback(goal_handle); },
      [this](GoalHandle goal_handle) { cancelCallback(goal_handle); },
      false);
  action_server->start();

  SPDLOG_LOGGER_INFO(
    ayanami::rosLogger(),
    "mbf_action_server: action={} map_frame={} base_frame={}",
    param.action_name,
    param.map_frame,
    param.base_frame);
}

void MbfActionServer::goalCallback(GoalHandle goal_handle) {
  const mbf_msgs::MoveBaseGoalConstPtr& goal = goal_handle.getGoal();
  if (!goal || goal->target_pose.header.frame_id.empty()) {
    mbf_msgs::MoveBaseResult result;
    result.outcome = mbf_msgs::MoveBaseResult::INTERNAL_ERROR;
    result.message = "target_pose has no frame_id";
    goal_handle.setRejected(result, result.message);
    return;
  }

  goal_handle.setAccepted();

  {
    std::lock_guard<std::mutex> lock(goal_handle_mutex);
    if (active_goal_handle.isValid() &&
        active_goal_handle.getGoalID().id != goal_handle.getGoalID().id) {
      mbf_msgs::MoveBaseResult result;
      result.outcome = mbf_msgs::MoveBaseResult::CANCELED;
      result.message = "preempted by a newer goal";
      active_goal_handle.setCanceled(result, result.message);
    }
    active_goal_handle = goal_handle;
  }

  // Latest-wins from here on, identical to a goal arriving on the ROS goal
  // topic: the mailbox keeps only the most recent request. The slot fans out
  // to observers by lvalue reference, so the converted goal is a named local.
  const ayanami::geometry_msgs::PoseStamped core_goal = fromRosPose(goal->target_pose);
  ayanami::interfaces::NavigationInputs::on_goal_request(core_goal);
  SPDLOG_LOGGER_INFO(
    ayanami::rosLogger(),
    "mbf_action_server: goal accepted frame={} x={:.3f} y={:.3f}",
    goal->target_pose.header.frame_id,
    goal->target_pose.pose.position.x,
    goal->target_pose.pose.position.y);
}

void MbfActionServer::cancelCallback(GoalHandle goal_handle) {
  {
    std::lock_guard<std::mutex> lock(goal_handle_mutex);
    // Goals and cancels travel on separate connections: a cancel for an
    // already-superseded goal must not kill the newer active one.
    if (!active_goal_handle.isValid() ||
        active_goal_handle.getGoalID().id != goal_handle.getGoalID().id) {
      return;
    }
    mbf_msgs::MoveBaseResult result;
    result.outcome = mbf_msgs::MoveBaseResult::CANCELED;
    result.message = "canceled by client";
    active_goal_handle.setCanceled(result, result.message);
    active_goal_handle = GoalHandle();
  }
  // No core cancel API: stop the robot by navigating to where it already is
  // (the planner succeeds immediately, the controller stops tracking). A goal
  // that arrives right after this cancel simply overwrites it in the mailbox.
  emitCurrentPoseGoal();
}

void MbfActionServer::handleNavigationResult(
    const ayanami::actions::NavigationResult& result) {
  GoalHandle goal_handle;
  {
    std::lock_guard<std::mutex> lock(goal_handle_mutex);
    if (!active_goal_handle.isValid()) {
      // The terminal outcome belongs to a navigation started via the plain
      // goal topic (RViz etc.), not to an action client.
      return;
    }
    goal_handle = active_goal_handle;
    active_goal_handle = GoalHandle();
  }

  mbf_msgs::MoveBaseResult ros_result;
  switch (result.status) {
    case ayanami::actions::GoalStatus::SUCCEEDED: {
      ros_result.outcome = mbf_msgs::MoveBaseResult::SUCCESS;
      ros_result.message = "succeeded";
      // Best effort final pose: TF hiccups must not turn a success into an
      // exception on the core's emit path.
      try {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = param.base_frame;
        pose.header.stamp = ros::Time::now();
        pose.pose.orientation.w = 1.0;
        tf_buffer.transform(
            pose, pose, param.map_frame, ros::Duration(param.stop_goal_tf_timeout));
        ros_result.final_pose = pose;
      } catch (const tf2::TransformException&) {
      }
      goal_handle.setSucceeded(ros_result, ros_result.message);
      break;
    }
    case ayanami::actions::GoalStatus::CANCELED:
      ros_result.outcome = mbf_msgs::MoveBaseResult::CANCELED;
      ros_result.message = "canceled";
      goal_handle.setCanceled(ros_result, ros_result.message);
      break;
    default:
      ros_result.outcome = mapOutcomeToMbf(result.context);
      ros_result.message = describeContext(result.context);
      goal_handle.setAborted(ros_result, ros_result.message);
      break;
  }
  SPDLOG_LOGGER_INFO(
    ayanami::rosLogger(),
    "mbf_action_server: goal finished status={} outcome={} message={}",
    static_cast<int>(result.status),
    ros_result.outcome,
    ros_result.message);
}

void MbfActionServer::emitCurrentPoseGoal() {
  try {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = param.base_frame;
    pose.header.stamp = ros::Time::now();
    pose.pose.orientation.w = 1.0;
    tf_buffer.transform(
        pose, pose, param.map_frame, ros::Duration(param.stop_goal_tf_timeout));
    const ayanami::geometry_msgs::PoseStamped core_goal = fromRosPose(pose);
    ayanami::interfaces::NavigationInputs::on_goal_request(core_goal);
  } catch (const tf2::TransformException& e) {
    // No TF -> cannot build a stop goal. The robot keeps its last controller
    // command until the velocity timeout trips; a pending new goal from the
    // same client supersedes this stop request anyway (patrol cancels are
    // immediately followed by sendGoal).
    SPDLOG_LOGGER_WARN(
        ayanami::rosLogger(),
        "mbf_action_server: cancel requested but no stop goal could be built ({})",
        e.what());
  }
}

}  // namespace ayanami_ros

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami_ros::MbfActionServer();
}
