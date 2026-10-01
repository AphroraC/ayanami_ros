#include "rviz_viewer.hpp"

#include <ayanami_ros/ros_msg_converter.hpp>

#include <atomic>
#include <sstream>

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

namespace ayanami_ros {

namespace {

// Test counting hooks (read by rvizViewerTest via dlsym): call count per
// publish handler.
std::atomic<int> g_planner_path_pub_count{0};
std::atomic<int> g_smoother_path_pub_count{0};
std::atomic<int> g_planner_goal_pub_count{0};
std::atomic<int> g_planner_feedback_pub_count{0};
std::atomic<int> g_smoother_goal_pub_count{0};
std::atomic<int> g_smoother_feedback_pub_count{0};
std::atomic<int> g_exe_path_goal_pub_count{0};
std::atomic<int> g_exe_path_feedback_pub_count{0};
std::atomic<int> g_exe_path_result_pub_count{0};
std::atomic<int> g_recovery_goal_pub_count{0};
std::atomic<int> g_recovery_feedback_pub_count{0};
std::atomic<int> g_recovery_result_pub_count{0};
std::atomic<int> g_navigation_result_pub_count{0};

// Test topic hook: the final topic after readConfig reads config_extensions.json.
std::string g_planner_path_topic;

const char* failureCodeName(ayanami::FailureCode code) {
  switch (code) {
    case ayanami::FailureCode::NONE: return "None";
    case ayanami::FailureCode::CANCELED: return "Canceled";
    case ayanami::FailureCode::TIMEOUT: return "Timeout";
    case ayanami::FailureCode::INVALID_INPUT: return "InvalidInput";
    case ayanami::FailureCode::TRANSFORM_UNAVAILABLE: return "TransformUnavailable";
    case ayanami::FailureCode::MAP_UNAVAILABLE: return "MapUnavailable";
    case ayanami::FailureCode::MAP_STALE: return "MapStale";
    case ayanami::FailureCode::START_INVALID: return "StartInvalid";
    case ayanami::FailureCode::GOAL_INVALID: return "GoalInvalid";
    case ayanami::FailureCode::PATH_NOT_FOUND: return "PathNotFound";
    case ayanami::FailureCode::PATH_INVALID: return "PathInvalid";
    case ayanami::FailureCode::SMOOTHING_FAILED: return "SmoothingFailed";
    case ayanami::FailureCode::NO_VALID_COMMAND: return "NoValidCommand";
    case ayanami::FailureCode::NO_PROGRESS: return "NoProgress";
    case ayanami::FailureCode::GOAL_NOT_REACHED: return "GoalNotReached";
    case ayanami::FailureCode::RECOVERY_FAILED: return "RecoveryFailed";
    case ayanami::FailureCode::INTERNAL_ERROR: return "InternalError";
  }
  return "Unknown";
}

const char* goalStatusName(ayanami::actions::GoalStatus status) {
  switch (status) {
    case ayanami::actions::GoalStatus::PENDING: return "PENDING";
    case ayanami::actions::GoalStatus::ACTIVE: return "ACTIVE";
    case ayanami::actions::GoalStatus::SUCCEEDED: return "SUCCEEDED";
    case ayanami::actions::GoalStatus::CANCELED: return "CANCELED";
    case ayanami::actions::GoalStatus::ABORTED: return "ABORTED";
  }
  return "UNKNOWN";
}

}  // namespace

RvizViewer::RvizViewer() : nh("~") {
  registerCallbacks();
}

RvizViewer::~RvizViewer() { unregisterCallbacks(); }

  /// @brief Parameters arrive AFTER a successful load, injected by the loader
  /// (ModuleContext = logical name + the extensions config). The section key is
  /// this module's own name, so the config layout stays outside the module --
  /// it no longer resolves the config path itself.
void RvizViewer::configureParameters(
    const ayanami::ModuleContext& context) {
    readConfig(context);
    advertiseTopics();
}

void RvizViewer::readConfig(const ayanami::ModuleContext& context) {
    // Single source for the extension's own parameters = this module's section
    // of the extensions config (config_ros only carries the ROS input
    // boundary); every bypass visualization topic published through global
    // callback slots is configured here.
    const ayanami::Config& extensions_config = context.config;
    const std::string& sec = context.name;
    planner_path_topic =
        extensions_config.param<std::string>(sec, "planner_path_topic",
                                             "/ayanami/planner/path");
    smoother_path_topic =
        extensions_config.param<std::string>(sec, "smoother_path_topic",
                                             "/ayanami/smoother/path");
    planner_goal_topic =
        extensions_config.param<std::string>(sec, "planner_goal_topic",
                                             "/ayanami/planner/goal");
    planner_feedback_topic =
        extensions_config.param<std::string>(sec, "planner_feedback_topic",
                                             "/ayanami/planner/feedback");
    smoother_input_path_topic =
        extensions_config.param<std::string>(sec, "smoother_input_path_topic",
                                             "/ayanami/smoother/input_path");
    smoother_feedback_topic =
        extensions_config.param<std::string>(sec, "smoother_feedback_topic",
                                             "/ayanami/smoother/feedback");
    controller_path_topic =
        extensions_config.param<std::string>(sec, "controller_path_topic",
                                             "/ayanami/controller/path");
    controller_feedback_topic =
        extensions_config.param<std::string>(sec, "controller_feedback_topic",
                                             "/ayanami/controller/feedback");
    controller_status_topic =
        extensions_config.param<std::string>(sec, "controller_status_topic",
                                             "/ayanami/controller/status");
    recovery_goal_topic =
        extensions_config.param<std::string>(sec, "recovery_goal_topic",
                                             "/ayanami/recovery/goal");
    recovery_feedback_topic =
        extensions_config.param<std::string>(sec, "recovery_feedback_topic",
                                             "/ayanami/recovery/feedback");
    recovery_status_topic =
        extensions_config.param<std::string>(sec, "recovery_status_topic",
                                             "/ayanami/recovery/status");
    navigation_status_topic =
        extensions_config.param<std::string>(sec, "navigation_status_topic",
                                             "/ayanami/navigation_status");
    // Velocity body output: the controller's global on_velocity_command slot
    // is republished here as the plain twist the chassis controller consumes.
    cmd_vel_topic = extensions_config.param<std::string>(sec, "cmd_vel_topic",
                                                         "/cmd_vel");
    g_planner_path_topic = planner_path_topic;
}

void RvizViewer::advertiseTopics() {
    // Paths and maps are latched (latched=true), so a newly subscribed rviz
    // immediately receives the latest frame.
    planner_path_pub =
        nh.advertise<nav_msgs::Path>(planner_path_topic, 1, true);
    smoother_path_pub =
        nh.advertise<nav_msgs::Path>(smoother_path_topic, 1, true);
    planner_goal_pub =
        nh.advertise<geometry_msgs::PoseStamped>(planner_goal_topic, 1);
    planner_feedback_pub =
        nh.advertise<std_msgs::String>(planner_feedback_topic, 1);
    smoother_input_path_pub =
        nh.advertise<nav_msgs::Path>(smoother_input_path_topic, 1, true);
    smoother_feedback_pub =
        nh.advertise<std_msgs::String>(smoother_feedback_topic, 1);
    controller_path_pub =
        nh.advertise<nav_msgs::Path>(controller_path_topic, 1, true);
    controller_feedback_pub =
        nh.advertise<std_msgs::String>(controller_feedback_topic, 1);
    controller_status_pub =
        nh.advertise<std_msgs::String>(controller_status_topic, 1, true);
    recovery_goal_pub =
        nh.advertise<std_msgs::String>(recovery_goal_topic, 1);
    recovery_feedback_pub =
        nh.advertise<std_msgs::String>(recovery_feedback_topic, 1);
    recovery_status_pub =
        nh.advertise<std_msgs::String>(recovery_status_topic, 1, true);
    navigation_status_pub = nh.advertise<std_msgs::String>(navigation_status_topic, 1);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    SPDLOG_LOGGER_INFO(
      ayanami::rosLogger(),
      "rviz_viewer publishers: planner_path={} smoother_path={} "
      "planner_goal={} smoother_input={} "
      "controller_path={} controller_status={} recovery_status={} "
      "navigation_status={} cmd_vel={}",
      planner_path_topic,
      smoother_path_topic,
      planner_goal_topic,
      smoother_input_path_topic,
      controller_path_topic,
      controller_status_topic,
      recovery_status_topic,
      navigation_status_topic,
      cmd_vel_topic);
}

void RvizViewer::registerCallbacks() {
    // Velocity body output: republish the controller's velocity command as
    // geometry_msgs/Twist on cmd_vel_topic (rm_chassis_controllers
    // ChassisBase input).
    cb_velocity_cmd =
        ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
            [this](const ayanami::geometry_msgs::TwistStamped& velocity) {
              if (cmd_vel_pub) {
                geometry_msgs::Twist cmd;
                cmd.linear.x = velocity.twist.linear.x();
                cmd.linear.y = velocity.twist.linear.y();
                cmd.linear.z = velocity.twist.linear.z();
                cmd.angular.x = velocity.twist.angular.x();
                cmd.angular.y = velocity.twist.angular.y();
                cmd.angular.z = velocity.twist.angular.z();
                cmd_vel_pub.publish(cmd);
}
            });
    cb_get_path_goal =
        ayanami::interfaces::NavigationCallbacks::on_get_path_goal.subscribe(
            [this](const ayanami::actions::GetPathGoal& goal) {
              publishPlannerGoal(goal);
            });
    cb_get_path_feedback =
        ayanami::interfaces::NavigationCallbacks::on_get_path_feedback.subscribe(
            [this](const ayanami::actions::GetPathFeedback& feedback) {
              publishPlannerFeedback(feedback);
            });
    cb_get_path =
        ayanami::interfaces::NavigationCallbacks::on_get_path_result.subscribe(
            [this](
                const ayanami::actions::GoalResult<ayanami::actions::GetPathResult>&
                    result) { publishPlannerPath(result); });
    cb_smooth_goal =
        ayanami::interfaces::NavigationCallbacks::on_smooth_path_goal.subscribe(
            [this](const ayanami::actions::SmoothPathGoal& goal) {
              publishSmootherGoal(goal);
            });
    cb_smooth_feedback =
        ayanami::interfaces::NavigationCallbacks::on_smooth_path_feedback.subscribe(
            [this](const ayanami::actions::SmoothPathFeedback& feedback) {
              publishSmootherFeedback(feedback);
            });
    cb_smooth_path =
        ayanami::interfaces::NavigationCallbacks::on_smooth_path_result.subscribe(
            [this](const ayanami::actions::GoalResult<
                       ayanami::actions::SmoothPathResult>& result) {
              publishSmootherPath(result);
            });
    cb_exe_goal =
        ayanami::interfaces::NavigationCallbacks::on_exe_path_goal.subscribe(
            [this](const ayanami::actions::ExePathGoal& goal) {
              publishExePathGoal(goal);
            });
    cb_exe_feedback =
        ayanami::interfaces::NavigationCallbacks::on_exe_path_feedback.subscribe(
            [this](const ayanami::actions::ExePathFeedback& feedback) {
              publishExePathFeedback(feedback);
            });
    cb_exe_result =
        ayanami::interfaces::NavigationCallbacks::on_exe_path_result.subscribe(
            [this](const ayanami::actions::GoalResult<
                       ayanami::actions::ExePathResult>& result) {
              publishExePathResult(result);
            });
    cb_recovery_goal =
        ayanami::interfaces::NavigationCallbacks::on_recovery_goal.subscribe(
            [this](const ayanami::actions::RecoveryGoal& goal) {
              publishRecoveryGoal(goal);
            });
    cb_recovery_feedback =
        ayanami::interfaces::NavigationCallbacks::on_recovery_feedback.subscribe(
            [this](const ayanami::actions::RecoveryFeedback& feedback) {
              publishRecoveryFeedback(feedback);
            });
    cb_recovery_result =
        ayanami::interfaces::NavigationCallbacks::on_recovery_result.subscribe(
            [this](const ayanami::actions::GoalResult<
                       ayanami::actions::RecoveryResult>& result) {
              publishRecoveryResult(result);
            });
    cb_navigation_result =
        ayanami::interfaces::NavigationCallbacks::on_navigation_result.subscribe(
            [this](const ayanami::actions::NavigationResult& result) {
              publishNavigationResult(result);
            });
}

void RvizViewer::unregisterCallbacks() {
    cb_get_path_goal.reset();
    cb_get_path_feedback.reset();
    cb_get_path.reset();
    cb_smooth_goal.reset();
    cb_smooth_feedback.reset();
    cb_smooth_path.reset();
    cb_exe_goal.reset();
    cb_exe_feedback.reset();
    cb_exe_result.reset();
    cb_recovery_goal.reset();
    cb_recovery_feedback.reset();
    cb_recovery_result.reset();
    cb_navigation_result.reset();
}

void RvizViewer::publishPlannerGoal(const ayanami::actions::GetPathGoal& goal) {
    g_planner_goal_pub_count.fetch_add(1);
    planner_goal_pub.publish(toRosPose(goal.goal));
}

void RvizViewer::publishPlannerFeedback(
      const ayanami::actions::GetPathFeedback& feedback) {
    g_planner_feedback_pub_count.fetch_add(1);
    std_msgs::String msg;
    msg.data = feedback.message;
    planner_feedback_pub.publish(msg);
}

void RvizViewer::publishPlannerPath(
      const ayanami::actions::GoalResult<ayanami::actions::GetPathResult>&
          result) {
    if (result.status != ayanami::actions::GoalStatus::SUCCEEDED) {
      return;
}
    g_planner_path_pub_count.fetch_add(1);
    planner_path_pub.publish(toRosPath(result.result.path));
}

void RvizViewer::publishSmootherGoal(const ayanami::actions::SmoothPathGoal& goal) {
    g_smoother_goal_pub_count.fetch_add(1);
    smoother_input_path_pub.publish(toRosPath(goal.path));
}

void RvizViewer::publishSmootherFeedback(
      const ayanami::actions::SmoothPathFeedback& feedback) {
    g_smoother_feedback_pub_count.fetch_add(1);
    std_msgs::String msg;
    msg.data = feedback.message;
    smoother_feedback_pub.publish(msg);
}

void RvizViewer::publishSmootherPath(
      const ayanami::actions::GoalResult<ayanami::actions::SmoothPathResult>&
          result) {
    if (result.status != ayanami::actions::GoalStatus::SUCCEEDED) {
      return;
}
    g_smoother_path_pub_count.fetch_add(1);
    smoother_path_pub.publish(toRosPath(result.result.smoothed_path));
}

void RvizViewer::publishExePathGoal(const ayanami::actions::ExePathGoal& goal) {
    g_exe_path_goal_pub_count.fetch_add(1);
    controller_path_pub.publish(toRosPath(goal.path));
}

void RvizViewer::publishExePathFeedback(
      const ayanami::actions::ExePathFeedback& feedback) {
    g_exe_path_feedback_pub_count.fetch_add(1);
    std_msgs::String msg;
    msg.data = feedback.message;
    controller_feedback_pub.publish(msg);
}

void RvizViewer::publishExePathResult(
      const ayanami::actions::GoalResult<ayanami::actions::ExePathResult>&
          result) {
    g_exe_path_result_pub_count.fetch_add(1);
    std_msgs::String msg;
    std::ostringstream oss;
    oss << "status=" << goalStatusName(result.status)
        << " failure=" << failureCodeName(result.result.error)
        << " reached=" << (result.result.reached ? "true" : "false")
        << " session=" << result.result.session_id
        << " plan=" << result.result.plan_version;
    if (!result.result.message.empty()) {
      oss << " message='" << result.result.message << "'";
}
    msg.data = oss.str();
    controller_status_pub.publish(msg);
}

void RvizViewer::publishRecoveryGoal(const ayanami::actions::RecoveryGoal& goal) {
    g_recovery_goal_pub_count.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(last_recovery_mutex);
      last_recovery_id = goal.recovery_id;
}
    std_msgs::String msg;
    msg.data = "recovery=" + goal.recovery_id;
    recovery_goal_pub.publish(msg);
}

void RvizViewer::publishRecoveryFeedback(
      const ayanami::actions::RecoveryFeedback& feedback) {
    g_recovery_feedback_pub_count.fetch_add(1);
    std_msgs::String msg;
    msg.data = feedback.message;
    recovery_feedback_pub.publish(msg);
}

void RvizViewer::publishRecoveryResult(
      const ayanami::actions::GoalResult<ayanami::actions::RecoveryResult>&
          result) {
    g_recovery_result_pub_count.fetch_add(1);
    std::string recovery_id;
    {
      std::lock_guard<std::mutex> lock(last_recovery_mutex);
      recovery_id = last_recovery_id;
}
    std_msgs::String msg;
    std::ostringstream oss;
    oss << "recovery=" << recovery_id << " status="
        << goalStatusName(result.status)
        << " failure=" << failureCodeName(result.result.error)
        << " session=" << result.result.session_id
        << " plan=" << result.result.plan_version;
    if (!result.result.message.empty()) {
      oss << " message='" << result.result.message << "'";
}
    msg.data = oss.str();
    recovery_status_pub.publish(msg);
}

void RvizViewer::publishNavigationResult(
      const ayanami::actions::NavigationResult& result) {
    g_navigation_result_pub_count.fetch_add(1);
    std_msgs::String msg;
    std::ostringstream oss;
    oss << "status=" << goalStatusName(result.status)
        << " failure=" << failureCodeName(result.context.code)
        << " session=" << result.context.session_id
        << " plan=" << result.context.plan_version;
    if (!result.context.message.empty()) {
      oss << " message='" << result.context.message << "'";
}
    msg.data = oss.str();
    navigation_status_pub.publish(msg);
}

}  // namespace ayanami_ros

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami_ros::RvizViewer();
}

// ---- TDD test hooks (read via dlsym by rvizViewerTest after dlopen) ----
extern "C" int rvizViewerPlannerPathPubCount() {
  return ayanami_ros::g_planner_path_pub_count.load();
}
extern "C" int rvizViewerSmootherPathPubCount() {
  return ayanami_ros::g_smoother_path_pub_count.load();
}
extern "C" int rvizViewerPlannerGoalPubCount() {
  return ayanami_ros::g_planner_goal_pub_count.load();
}
extern "C" int rvizViewerPlannerFeedbackPubCount() {
  return ayanami_ros::g_planner_feedback_pub_count.load();
}
extern "C" int rvizViewerSmootherGoalPubCount() {
  return ayanami_ros::g_smoother_goal_pub_count.load();
}
extern "C" int rvizViewerSmootherFeedbackPubCount() {
  return ayanami_ros::g_smoother_feedback_pub_count.load();
}
extern "C" int rvizViewerExePathGoalPubCount() {
  return ayanami_ros::g_exe_path_goal_pub_count.load();
}
extern "C" int rvizViewerExePathFeedbackPubCount() {
  return ayanami_ros::g_exe_path_feedback_pub_count.load();
}
extern "C" int rvizViewerExePathResultPubCount() {
  return ayanami_ros::g_exe_path_result_pub_count.load();
}
extern "C" int rvizViewerRecoveryGoalPubCount() {
  return ayanami_ros::g_recovery_goal_pub_count.load();
}
extern "C" int rvizViewerRecoveryFeedbackPubCount() {
  return ayanami_ros::g_recovery_feedback_pub_count.load();
}
extern "C" int rvizViewerRecoveryResultPubCount() {
  return ayanami_ros::g_recovery_result_pub_count.load();
}
extern "C" int rvizViewerNavigationResultPubCount() {
  return ayanami_ros::g_navigation_result_pub_count.load();
}
extern "C" const char* rvizViewerPlannerPathTopic() {
  return ayanami_ros::g_planner_path_topic.c_str();
}
