#include <ayanami_ros/ros_msg_converter.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>

#include <atomic>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace ayanami_ros {

namespace {

// 测试计数 hook (rvizViewerTest 经 dlsym 读): 各 publish handler 被调次数。
std::atomic<int> g_planner_path_pub_count{0};
std::atomic<int> g_smoother_path_pub_count{0};
std::atomic<int> g_velocity_pub_count{0};
std::atomic<int> g_global_costmap_pub_count{0};
std::atomic<int> g_local_costmap_pub_count{0};
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

// 测试话题 hook: readConfig 读 config_ros.json 后的最终 topic。
std::string g_planner_path_topic;

const char* failureCodeName(ayanami::FailureCode code) {
  switch (code) {
    case ayanami::FailureCode::None: return "None";
    case ayanami::FailureCode::Canceled: return "Canceled";
    case ayanami::FailureCode::Timeout: return "Timeout";
    case ayanami::FailureCode::InvalidInput: return "InvalidInput";
    case ayanami::FailureCode::TransformUnavailable: return "TransformUnavailable";
    case ayanami::FailureCode::MapUnavailable: return "MapUnavailable";
    case ayanami::FailureCode::MapStale: return "MapStale";
    case ayanami::FailureCode::StartInvalid: return "StartInvalid";
    case ayanami::FailureCode::GoalInvalid: return "GoalInvalid";
    case ayanami::FailureCode::PathNotFound: return "PathNotFound";
    case ayanami::FailureCode::PathInvalid: return "PathInvalid";
    case ayanami::FailureCode::SmoothingFailed: return "SmoothingFailed";
    case ayanami::FailureCode::NoValidCommand: return "NoValidCommand";
    case ayanami::FailureCode::NoProgress: return "NoProgress";
    case ayanami::FailureCode::GoalNotReached: return "GoalNotReached";
    case ayanami::FailureCode::RecoveryFailed: return "RecoveryFailed";
    case ayanami::FailureCode::InternalError: return "InternalError";
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

// spec §4.5 核心 -> ROS 输出桥接: ayanami_ros 本体只负责读取 ROS 数据并输入
// 核心; 核心的全部输出 (四类 action goal/feedback/result、cmd_vel、costmap
// 可视化、robot state) 一律经本 ext 的 NavigationCallbacks 全局回调订阅读取,
// 并发布为对应 ROS 信息。本插件直接继承 ayanami::ExtensionModule, 构造期自建
// ros::NodeHandle 并 advertise + add 订阅 (由 AyanamiROS::loadExtensionModules
// dlopen 触发, 先于 navigation_plugin->start(), 满足 M2-P1 顺序); 析构期
// remove callback ids。不 include costmap_2d/* (spec §4.6 红线): costmap 用
// core OccupancyGrid 转发为 nav_msgs::OccupancyGrid。
class RvizViewer final : public ayanami::ExtensionModule {
public:
  RvizViewer() : nh("~") {
    readConfig();
    advertiseTopics();
    registerCallbacks();
  }

  ~RvizViewer() override { unregisterCallbacks(); }

private:
  void readConfig() {
    // config_ros.json 缺失/不可读时 Config 为空文档 → 全回退默认 (spec §8.1)。
    const ayanami::Config config_ros(
        ayanami::GlobalConfig::getConfigPath("config_ros"));
    const std::string sec = "ayanami_ros";
    planner_path_topic =
        config_ros.param<std::string>(sec, "planner_path_topic",
                                      "/ayanami/planner/path");
    smoother_path_topic =
        config_ros.param<std::string>(sec, "smoother_path_topic",
                                      "/ayanami/smoother/path");
    velocity_topic =
        config_ros.param<std::string>(sec, "cmd_vel_topic", "/cmd_vel");
    velocity_stamped_topic =
        config_ros.param<std::string>(sec, "cmd_vel_stamped_topic",
                                      "/cmd_vel_stamped");
    global_costmap_topic =
        config_ros.param<std::string>(sec, "global_costmap_topic",
                                      "/ayanami/global_costmap");
    local_costmap_topic =
        config_ros.param<std::string>(sec, "local_costmap_topic",
                                      "/ayanami/local_costmap");
    planner_goal_topic =
        config_ros.param<std::string>(sec, "planner_goal_topic",
                                      "/ayanami/planner/goal");
    planner_feedback_topic =
        config_ros.param<std::string>(sec, "planner_feedback_topic",
                                      "/ayanami/planner/feedback");
    smoother_input_path_topic =
        config_ros.param<std::string>(sec, "smoother_input_path_topic",
                                      "/ayanami/smoother/input_path");
    smoother_feedback_topic =
        config_ros.param<std::string>(sec, "smoother_feedback_topic",
                                      "/ayanami/smoother/feedback");
    controller_path_topic =
        config_ros.param<std::string>(sec, "controller_path_topic",
                                      "/ayanami/controller/path");
    controller_feedback_topic =
        config_ros.param<std::string>(sec, "controller_feedback_topic",
                                      "/ayanami/controller/feedback");
    controller_status_topic =
        config_ros.param<std::string>(sec, "controller_status_topic",
                                      "/ayanami/controller/status");
    recovery_goal_topic =
        config_ros.param<std::string>(sec, "recovery_goal_topic",
                                      "/ayanami/recovery/goal");
    recovery_feedback_topic =
        config_ros.param<std::string>(sec, "recovery_feedback_topic",
                                      "/ayanami/recovery/feedback");
    recovery_status_topic =
        config_ros.param<std::string>(sec, "recovery_status_topic",
                                      "/ayanami/recovery/status");
    map_frame = config_ros.param<std::string>(sec, "map_frame", "map");
    odom_frame = config_ros.param<std::string>(sec, "odom_frame", "odom");
    g_planner_path_topic = planner_path_topic;
  }

  void advertiseTopics() {
    // 路径与地图 latched (latched=true), 新订阅的 rviz 立即拿到最新一帧。
    planner_path_pub =
        nh.advertise<nav_msgs::Path>(planner_path_topic, 1, true);
    smoother_path_pub =
        nh.advertise<nav_msgs::Path>(smoother_path_topic, 1, true);
    velocity_pub = nh.advertise<geometry_msgs::Twist>(velocity_topic, 1);
    // RViz 1.14 has no plain Twist display; publish a TwistStamped twin of the
    // velocity command for the /cmd_vel_stamped display.
    velocity_stamped_pub =
        nh.advertise<geometry_msgs::TwistStamped>(velocity_stamped_topic, 1);
    global_costmap_pub =
        nh.advertise<nav_msgs::OccupancyGrid>(global_costmap_topic, 1, true);
    local_costmap_pub =
        nh.advertise<nav_msgs::OccupancyGrid>(local_costmap_topic, 1, true);
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
    spdlog::info(
        "rviz_viewer publishers: planner_path={} smoother_path={} cmd_vel={} "
        "global_costmap={} local_costmap={} planner_goal={} smoother_input={} "
        "controller_path={} controller_status={} recovery_status={}",
        planner_path_topic, smoother_path_topic, velocity_topic,
        global_costmap_topic, local_costmap_topic, planner_goal_topic,
        smoother_input_path_topic, controller_path_topic,
        controller_status_topic, recovery_status_topic);
  }

  void registerCallbacks() {
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
    cb_velocity_cmd =
        ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
            [this](const ayanami::geometry_msgs::TwistStamped& velocity) {
              publishVelocityCommand(velocity);
            });
    cb_costmap_viz =
        ayanami::interfaces::NavigationCallbacks::on_costmap_visualization.subscribe(
            [this](const std::string& name,
                   const ayanami::nav_msgs::OccupancyGrid& map) {
              publishCostmap(name, map);
            });
  }

  void unregisterCallbacks() {
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
    cb_velocity_cmd.reset();
    cb_costmap_viz.reset();
  }

  void publishPlannerGoal(const ayanami::actions::GetPathGoal& goal) {
    g_planner_goal_pub_count.fetch_add(1);
    planner_goal_pub.publish(toRosPose(goal.goal));
  }

  void publishPlannerFeedback(
      const ayanami::actions::GetPathFeedback& feedback) {
    g_planner_feedback_pub_count.fetch_add(1);
    std_msgs::String msg;
    msg.data = feedback.message;
    planner_feedback_pub.publish(msg);
  }

  void publishPlannerPath(
      const ayanami::actions::GoalResult<ayanami::actions::GetPathResult>&
          result) {
    if (result.status != ayanami::actions::GoalStatus::SUCCEEDED) {
      return;
    }
    g_planner_path_pub_count.fetch_add(1);
    planner_path_pub.publish(toRosPath(result.result.path));
  }

  void publishSmootherGoal(const ayanami::actions::SmoothPathGoal& goal) {
    g_smoother_goal_pub_count.fetch_add(1);
    smoother_input_path_pub.publish(toRosPath(goal.path));
  }

  void publishSmootherFeedback(
      const ayanami::actions::SmoothPathFeedback& feedback) {
    g_smoother_feedback_pub_count.fetch_add(1);
    std_msgs::String msg;
    msg.data = feedback.message;
    smoother_feedback_pub.publish(msg);
  }

  void publishSmootherPath(
      const ayanami::actions::GoalResult<ayanami::actions::SmoothPathResult>&
          result) {
    if (result.status != ayanami::actions::GoalStatus::SUCCEEDED) {
      return;
    }
    g_smoother_path_pub_count.fetch_add(1);
    smoother_path_pub.publish(toRosPath(result.result.smoothed_path));
  }

  void publishExePathGoal(const ayanami::actions::ExePathGoal& goal) {
    g_exe_path_goal_pub_count.fetch_add(1);
    controller_path_pub.publish(toRosPath(goal.path));
  }

  void publishExePathFeedback(
      const ayanami::actions::ExePathFeedback& feedback) {
    g_exe_path_feedback_pub_count.fetch_add(1);
    std_msgs::String msg;
    msg.data = feedback.message;
    controller_feedback_pub.publish(msg);
  }

  void publishExePathResult(
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

  void publishRecoveryGoal(const ayanami::actions::RecoveryGoal& goal) {
    g_recovery_goal_pub_count.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(last_recovery_mutex);
      last_recovery_id = goal.recovery_id;
    }
    std_msgs::String msg;
    msg.data = "recovery=" + goal.recovery_id;
    recovery_goal_pub.publish(msg);
  }

  void publishRecoveryFeedback(
      const ayanami::actions::RecoveryFeedback& feedback) {
    g_recovery_feedback_pub_count.fetch_add(1);
    std_msgs::String msg;
    msg.data = feedback.message;
    recovery_feedback_pub.publish(msg);
  }

  void publishRecoveryResult(
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

  void publishVelocityCommand(
      const ayanami::geometry_msgs::TwistStamped& velocity) {
    g_velocity_pub_count.fetch_add(1);
    // Task 12 stage C: gazebo_ros_diff_drive (noetic 2.9.3) 只订阅
    // geometry_msgs/Twist (无 TwistStamped/legacyMode), 故 /cmd_vel 发 Twist。
    geometry_msgs::Twist twist;
    twist.linear.x = velocity.twist.linear.x();
    twist.linear.y = velocity.twist.linear.y();
    twist.linear.z = velocity.twist.linear.z();
    twist.angular.x = velocity.twist.angular.x();
    twist.angular.y = velocity.twist.angular.y();
    twist.angular.z = velocity.twist.angular.z();
    velocity_pub.publish(twist);
    // Twin TwistStamped for RViz's CmdVel display (rviz 1.14 has no plain
    // geometry_msgs/Twist display).
    geometry_msgs::TwistStamped twist_stamped;
    twist_stamped.header.frame_id = "base_footprint";
    twist_stamped.twist = twist;
    velocity_stamped_pub.publish(twist_stamped);
  }

  void publishCostmap(const std::string& name,
                      const ayanami::nav_msgs::OccupancyGrid& map) {
    nav_msgs::OccupancyGrid grid = toRosOccupancyGrid(map);
    if (grid.header.frame_id.empty()) {
      grid.header.frame_id =
          name == "local_costmap"
              ? (odom_frame.empty() ? "odom" : odom_frame)
              : (map_frame.empty() ? "map" : map_frame);
    }
    if (name == "local_costmap") {
      g_local_costmap_pub_count.fetch_add(1);
      local_costmap_pub.publish(grid);
    } else {
      g_global_costmap_pub_count.fetch_add(1);
      global_costmap_pub.publish(grid);
    }
  }

  ros::NodeHandle nh;
  ros::Publisher planner_path_pub, smoother_path_pub, velocity_pub,
      global_costmap_pub, local_costmap_pub, planner_goal_pub,
      planner_feedback_pub, smoother_input_path_pub, smoother_feedback_pub,
      controller_path_pub, controller_feedback_pub, controller_status_pub,
      recovery_goal_pub, recovery_feedback_pub, recovery_status_pub,
      velocity_stamped_pub;
  std::string planner_path_topic = "/ayanami/planner/path";
  std::string smoother_path_topic = "/ayanami/smoother/path";
  std::string velocity_topic = "/cmd_vel";
  std::string velocity_stamped_topic = "/cmd_vel_stamped";
  std::string global_costmap_topic = "/ayanami/global_costmap";
  std::string local_costmap_topic = "/ayanami/local_costmap";
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
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::mutex last_recovery_mutex;
  std::string last_recovery_id;
  ayanami::CallbackSubscription cb_costmap_viz, cb_get_path, cb_smooth_path,
      cb_velocity_cmd, cb_get_path_goal, cb_get_path_feedback, cb_smooth_goal,
      cb_smooth_feedback, cb_exe_goal, cb_exe_feedback, cb_exe_result,
      cb_recovery_goal, cb_recovery_feedback, cb_recovery_result;
};

}  // namespace ayanami_ros

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami_ros::RvizViewer();
}

// ---- TDD 测试 hook (rvizViewerTest dlopen 后经 dlsym 读) ----
extern "C" int rvizViewerPlannerPathPubCount() {
  return ayanami_ros::g_planner_path_pub_count.load();
}
extern "C" int rvizViewerSmootherPathPubCount() {
  return ayanami_ros::g_smoother_path_pub_count.load();
}
extern "C" int rvizViewerVelocityPubCount() {
  return ayanami_ros::g_velocity_pub_count.load();
}
extern "C" int rvizViewerGlobalCostmapPubCount() {
  return ayanami_ros::g_global_costmap_pub_count.load();
}
extern "C" int rvizViewerLocalCostmapPubCount() {
  return ayanami_ros::g_local_costmap_pub_count.load();
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
extern "C" const char* rvizViewerPlannerPathTopic() {
  return ayanami_ros::g_planner_path_topic.c_str();
}
