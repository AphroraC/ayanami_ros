#include <ayanami_ros/extension_module_ros.hpp>
#include <ayanami_ros/ros_msg_converter.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/config.hpp>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <atomic>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace ayanami_ros {

namespace {

// 测试计数 hook (rviz_viewer_test 经 dlsym 读): 各 publish handler 被调次数。
std::atomic<int> g_planner_path_pub_count{0};
std::atomic<int> g_smoother_path_pub_count{0};
std::atomic<int> g_velocity_pub_count{0};
std::atomic<int> g_global_costmap_pub_count{0};
std::atomic<int> g_local_costmap_pub_count{0};

// 测试话题 hook: create_subscriptions 读 config_ros.json 后的最终 topic。
std::string g_planner_path_topic;

}  // namespace

// spec §4.5 核心 -> ROS: 构造期 add NavigationCallbacks 订阅 + advertiseTopics
// (经 create_subscriptions(nh) 在 AyanamiROS::loadExtensionModules 内调用, 先于
// navigation_plugin->start(), 满足 M2-P1 顺序); 析构期 remove callback ids。
// 不 include costmap_2d/* (spec §4.6 红线): costmap 用 core OccupancyGrid 转发为
// nav_msgs::OccupancyGrid。
class RvizViewer final : public ExtensionModuleROS {
public:
  RvizViewer() = default;

  ~RvizViewer() override { unregisterCallbacks(); }

  std::vector<GenericTopicSubscription> create_subscriptions(
      ros::NodeHandle& nh) override {
    nh_ = nh;
    readConfig();
    advertiseTopics();
    registerCallbacks();
    // 本 ext 不主动 ros subscribe (spec §4.5: 可留空)。
    return {};
  }

private:
  void readConfig() {
    // config_ros.json 缺失/不可读时 Config 为空文档 → 全回退默认 (spec §8.1)。
    const ayanami::Config config_ros(
        ayanami::GlobalConfig::get_config_path("config_ros"));
    planner_path_topic_ = config_ros.param<std::string>(
        "ayanami_ros", "planner_path_topic", "/ayanami/planner/path");
    smoother_path_topic_ = config_ros.param<std::string>(
        "ayanami_ros", "smoother_path_topic", "/ayanami/smoother/path");
    velocity_topic_ = config_ros.param<std::string>(
        "ayanami_ros", "cmd_vel_topic", "/cmd_vel");
    static_map_topic_ = config_ros.param<std::string>(
        "ayanami_ros", "static_map_topic", "/ayanami/map");
    global_costmap_topic_ = config_ros.param<std::string>(
        "ayanami_ros", "global_costmap_topic", "/ayanami/global_costmap");
    local_costmap_topic_ = config_ros.param<std::string>(
        "ayanami_ros", "local_costmap_topic", "/ayanami/local_costmap");
    map_frame_ = config_ros.param<std::string>("ayanami_ros", "map_frame", "map");
    odom_frame_ =
        config_ros.param<std::string>("ayanami_ros", "odom_frame", "odom");
    g_planner_path_topic = planner_path_topic_;
  }

  void advertiseTopics() {
    // 路径与地图 latched (latched=true), 新订阅的 rviz 立即拿到最新一帧。
    planner_path_pub_ =
        nh_.advertise<nav_msgs::Path>(planner_path_topic_, 1, true);
    smoother_path_pub_ =
        nh_.advertise<nav_msgs::Path>(smoother_path_topic_, 1, true);
    velocity_pub_ =
        nh_.advertise<geometry_msgs::TwistStamped>(velocity_topic_, 1);
    static_map_pub_ =
        nh_.advertise<nav_msgs::OccupancyGrid>(static_map_topic_, 1, true);
    global_costmap_pub_ =
        nh_.advertise<nav_msgs::OccupancyGrid>(global_costmap_topic_, 1, true);
    local_costmap_pub_ =
        nh_.advertise<nav_msgs::OccupancyGrid>(local_costmap_topic_, 1, true);
    spdlog::info(
        "rviz_viewer publishers: planner_path={} smoother_path={} cmd_vel={} "
        "static_map={} global_costmap={} local_costmap={}",
        planner_path_topic_, smoother_path_topic_, velocity_topic_,
        static_map_topic_, global_costmap_topic_, local_costmap_topic_);
  }

  void registerCallbacks() {
    cb_get_path_ =
        ayanami::interfaces::NavigationCallbacks::on_get_path_result.add(
            [this](
                const ayanami::actions::GoalResult<ayanami::actions::GetPathResult>&
                    result) { publishPlannerPath(result); });
    cb_smooth_path_ =
        ayanami::interfaces::NavigationCallbacks::on_smooth_path_result.add(
            [this](const ayanami::actions::GoalResult<
                       ayanami::actions::SmoothPathResult>& result) {
              publishSmootherPath(result);
            });
    cb_velocity_cmd_ =
        ayanami::interfaces::NavigationCallbacks::on_velocity_command.add(
            [this](const ayanami::geometry_msgs::TwistStamped& velocity) {
              publishVelocityCommand(velocity);
            });
    cb_costmap_viz_ =
        ayanami::interfaces::NavigationCallbacks::on_costmap_visualization.add(
            [this](const std::string& name,
                   const ayanami::nav_msgs::OccupancyGrid& map) {
              publishCostmap(name, map);
            });
  }

  void unregisterCallbacks() {
    if (cb_get_path_ >= 0) {
      ayanami::interfaces::NavigationCallbacks::on_get_path_result.remove(
          cb_get_path_);
      cb_get_path_ = -1;
    }
    if (cb_smooth_path_ >= 0) {
      ayanami::interfaces::NavigationCallbacks::on_smooth_path_result.remove(
          cb_smooth_path_);
      cb_smooth_path_ = -1;
    }
    if (cb_velocity_cmd_ >= 0) {
      ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(
          cb_velocity_cmd_);
      cb_velocity_cmd_ = -1;
    }
    if (cb_costmap_viz_ >= 0) {
      ayanami::interfaces::NavigationCallbacks::on_costmap_visualization.remove(
          cb_costmap_viz_);
      cb_costmap_viz_ = -1;
    }
  }

  void publishPlannerPath(
      const ayanami::actions::GoalResult<ayanami::actions::GetPathResult>&
          result) {
    if (result.status != ayanami::actions::GoalStatus::SUCCEEDED) {
      return;
    }
    g_planner_path_pub_count.fetch_add(1);
    planner_path_pub_.publish(toRosPath(result.result.path));
  }

  void publishSmootherPath(
      const ayanami::actions::GoalResult<ayanami::actions::SmoothPathResult>&
          result) {
    if (result.status != ayanami::actions::GoalStatus::SUCCEEDED) {
      return;
    }
    g_smoother_path_pub_count.fetch_add(1);
    smoother_path_pub_.publish(toRosPath(result.result.smoothed_path));
  }

  void publishVelocityCommand(
      const ayanami::geometry_msgs::TwistStamped& velocity) {
    g_velocity_pub_count.fetch_add(1);
    velocity_pub_.publish(toRosTwist(velocity));
  }

  void publishCostmap(const std::string& name,
                      const ayanami::nav_msgs::OccupancyGrid& map) {
    nav_msgs::OccupancyGrid grid = toRosOccupancyGrid(map);
    if (grid.header.frame_id.empty()) {
      grid.header.frame_id =
          name == "local_costmap"
              ? (odom_frame_.empty() ? "odom" : odom_frame_)
              : (map_frame_.empty() ? "map" : map_frame_);
    }
    if (name == "local_costmap") {
      g_local_costmap_pub_count.fetch_add(1);
      local_costmap_pub_.publish(grid);
    } else {
      g_global_costmap_pub_count.fetch_add(1);
      global_costmap_pub_.publish(grid);
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher planner_path_pub_, smoother_path_pub_, velocity_pub_,
      global_costmap_pub_, local_costmap_pub_, static_map_pub_;
  std::string planner_path_topic_ = "/ayanami/planner/path";
  std::string smoother_path_topic_ = "/ayanami/smoother/path";
  std::string velocity_topic_ = "/cmd_vel";
  std::string static_map_topic_ = "/ayanami/map";
  std::string global_costmap_topic_ = "/ayanami/global_costmap";
  std::string local_costmap_topic_ = "/ayanami/local_costmap";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  int cb_costmap_viz_ = -1, cb_get_path_ = -1, cb_smooth_path_ = -1,
      cb_velocity_cmd_ = -1;
};

}  // namespace ayanami_ros

extern "C" ayanami_ros::ExtensionModuleROS* create_extension_module() {
  return new ayanami_ros::RvizViewer();
}

// ---- TDD 测试 hook (rviz_viewer_test dlopen 后经 dlsym 读) ----
extern "C" int rviz_viewer_planner_path_pub_count() {
  return ayanami_ros::g_planner_path_pub_count.load();
}
extern "C" int rviz_viewer_smoother_path_pub_count() {
  return ayanami_ros::g_smoother_path_pub_count.load();
}
extern "C" int rviz_viewer_velocity_pub_count() {
  return ayanami_ros::g_velocity_pub_count.load();
}
extern "C" int rviz_viewer_global_costmap_pub_count() {
  return ayanami_ros::g_global_costmap_pub_count.load();
}
extern "C" int rviz_viewer_local_costmap_pub_count() {
  return ayanami_ros::g_local_costmap_pub_count.load();
}
extern "C" const char* rviz_viewer_planner_path_topic() {
  return ayanami_ros::g_planner_path_topic.c_str();
}
