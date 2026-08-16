// Task 9: rviz_viewer ROS extension .so (dlopen → create_extension_module →
// RvizViewer : ExtensionModuleROS)。
//
// 无 roscore 环境 (headless)。用临时 config 目录 (含 config_ros.json, 覆盖 rviz
// 话题名) + dlopen librviz_viewer.so, 验证:
//   1. dlopen + create_extension_module + dynamic_cast<ExtensionModuleROS>。
//   2. create_subscriptions(nh) → advertiseTopics (6 publisher) + 注册 4 个
//      NavigationCallbacks 订阅, 返回空 sub 列表 (本 ext 不主动 ros subscribe)。
//   3. 主动 emit NavigationCallbacks 事件 → 对应 publish handler 被调 (经 .so 导出
//      的计数 hook 证明), 无订阅者时 publish 为 no-op, 无崩。
//   4. 话题名从 config_ros.json::ayanami_ros 读入 (planner_path_topic 覆盖验证)。
//   5. ~RvizViewer → 4 个 callback id 全部 remove → 再 emit 计数不再增。

#include <dlfcn.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>
#include <ayanami_ros/extension_module_ros.hpp>

namespace {

#ifndef RVIZ_VIEWER_SO
#error "RVIZ_VIEWER_SO must be defined by CMake"
#endif

const char* const g_rviz_viewer_so = RVIZ_VIEWER_SO;

void write_file(const std::filesystem::path& path,
                const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream ofs(path);
  ASSERT_TRUE(ofs.good()) << "cannot write " << path;
  ofs << content;
}

std::filesystem::path make_config_dir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_ros_rviz_viewer_test";
  std::filesystem::remove_all(dir);
  write_file(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_ros\": \"config_ros.json\"\n"
             "  }\n}\n");
  write_file(dir / "config_ros.json",
             "{\n  \"ayanami_ros\": {\n"
             "    \"planner_path_topic\": \"/test/rviz/planner/path\",\n"
             "    \"smoother_path_topic\": \"/test/rviz/smoother/path\",\n"
             "    \"cmd_vel_topic\": \"/test/rviz/cmd_vel\",\n"
             "    \"static_map_topic\": \"/test/rviz/static_map\",\n"
             "    \"global_costmap_topic\": \"/test/rviz/global_costmap\",\n"
             "    \"local_costmap_topic\": \"/test/rviz/local_costmap\",\n"
             "    \"map_frame\": \"map\",\n"
             "    \"odom_frame\": \"odom\"\n"
             "  }\n}\n");
  return dir;
}

void init_global_config(const std::filesystem::path& dir) {
  setenv("AYANAMI_CONFIG_PATH", dir.string().c_str(), 1);
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);
}

// 从 librviz_viewer.so 读 publish 计数 / 话题名 (dlopen 由 load_module RTLD_GLOBAL)。
int pub_count(const char* symbol) {
  using Getter = int (*)();
  auto getter = reinterpret_cast<Getter>(dlsym(RTLD_DEFAULT, symbol));
  if (getter == nullptr) {
    return -1;
  }
  return getter();
}

const char* topic_string(const char* symbol) {
  using Getter = const char* (*)();
  auto getter = reinterpret_cast<Getter>(dlsym(RTLD_DEFAULT, symbol));
  if (getter == nullptr) {
    return nullptr;
  }
  return getter();
}

ayanami::nav_msgs::Path make_path() {
  ayanami::nav_msgs::Path path;
  path.header.frame_id = "map";
  ayanami::geometry_msgs::PoseStamped pose =
      ayanami::geometry_msgs::PoseStamped::fromIsometry(
          "map", Eigen::Isometry3d::Identity());
  pose.pose.translation() = Eigen::Vector3d(1.0, 2.0, 0.0);
  path.poses.push_back(pose);
  return path;
}

TEST(RvizViewer, DlopenCreateAndRegisterCallbacks) {
  init_global_config(make_config_dir());

  auto ext = ayanami::ExtensionModule::load_module(g_rviz_viewer_so);
  ASSERT_NE(ext, nullptr);
  auto ros_ext = std::dynamic_pointer_cast<ayanami_ros::ExtensionModuleROS>(ext);
  ASSERT_NE(ros_ext, nullptr);

  ros::NodeHandle nh("~");
  auto subs = ros_ext->create_subscriptions(nh);
  EXPECT_TRUE(subs.empty());

  // 6 publisher advertiseTopics 完成: 计数 hook 均可达 (dlsym 找到, 初值 0)。
  EXPECT_GE(pub_count("rviz_viewer_planner_path_pub_count"), 0);
  EXPECT_GE(pub_count("rviz_viewer_smoother_path_pub_count"), 0);
  EXPECT_GE(pub_count("rviz_viewer_velocity_pub_count"), 0);
  EXPECT_GE(pub_count("rviz_viewer_global_costmap_pub_count"), 0);
  EXPECT_GE(pub_count("rviz_viewer_local_costmap_pub_count"), 0);

  // 话题名从 config_ros.json::ayanami_ros 读入 (覆盖默认值)。
  const char* topic = topic_string("rviz_viewer_planner_path_topic");
  ASSERT_NE(topic, nullptr);
  EXPECT_STREQ(topic, "/test/rviz/planner/path");
}

TEST(RvizViewer, EmitCallbacksPublishRos) {
  init_global_config(make_config_dir());

  auto ext = ayanami::ExtensionModule::load_module(g_rviz_viewer_so);
  ASSERT_NE(ext, nullptr);
  auto ros_ext = std::dynamic_pointer_cast<ayanami_ros::ExtensionModuleROS>(ext);
  ASSERT_NE(ros_ext, nullptr);

  ros::NodeHandle nh("~");
  auto subs = ros_ext->create_subscriptions(nh);
  (void)subs;

  // 主动 emit 4 组 NavigationCallbacks → 对应 publish handler 被调 ≥1。
  ayanami::actions::GoalResult<ayanami::actions::GetPathResult> get_path;
  get_path.status = ayanami::actions::GoalStatus::SUCCEEDED;
  get_path.result.path = make_path();
  const int planner_before = pub_count("rviz_viewer_planner_path_pub_count");
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(get_path);
  EXPECT_GE(pub_count("rviz_viewer_planner_path_pub_count"),
            planner_before + 1);

  ayanami::actions::GoalResult<ayanami::actions::SmoothPathResult> smooth;
  smooth.status = ayanami::actions::GoalStatus::SUCCEEDED;
  smooth.result.smoothed_path = make_path();
  const int smoother_before = pub_count("rviz_viewer_smoother_path_pub_count");
  ayanami::interfaces::NavigationCallbacks::on_smooth_path_result(smooth);
  EXPECT_GE(pub_count("rviz_viewer_smoother_path_pub_count"),
            smoother_before + 1);

  ayanami::geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "odom";
  twist.twist.linear = Eigen::Vector3d(1.0, 0.0, 0.0);
  const int velocity_before = pub_count("rviz_viewer_velocity_pub_count");
  ayanami::interfaces::NavigationCallbacks::on_velocity_command(twist);
  EXPECT_GE(pub_count("rviz_viewer_velocity_pub_count"), velocity_before + 1);

  ayanami::nav_msgs::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.5;
  grid.info.width = 4;
  grid.info.height = 4;
  grid.data.assign(16, 0);
  const int global_before = pub_count("rviz_viewer_global_costmap_pub_count");
  ayanami::interfaces::NavigationCallbacks::on_costmap_visualization(
      "global_costmap", grid);
  EXPECT_GE(pub_count("rviz_viewer_global_costmap_pub_count"),
            global_before + 1);
  const int local_before = pub_count("rviz_viewer_local_costmap_pub_count");
  ayanami::interfaces::NavigationCallbacks::on_costmap_visualization(
      "local_costmap", grid);
  EXPECT_GE(pub_count("rviz_viewer_local_costmap_pub_count"),
            local_before + 1);

  // 非 SUCCEEDED 的 path 不 publish。
  ayanami::actions::GoalResult<ayanami::actions::GetPathResult> failed;
  failed.status = ayanami::actions::GoalStatus::ABORTED;
  failed.result.path = make_path();
  const int planner_after = pub_count("rviz_viewer_planner_path_pub_count");
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(failed);
  EXPECT_EQ(pub_count("rviz_viewer_planner_path_pub_count"), planner_after);
}

TEST(RvizViewer, DestructorRemovesCallbacks) {
  init_global_config(make_config_dir());

  auto ext = ayanami::ExtensionModule::load_module(g_rviz_viewer_so);
  ASSERT_NE(ext, nullptr);
  auto ros_ext = std::dynamic_pointer_cast<ayanami_ros::ExtensionModuleROS>(ext);
  ASSERT_NE(ros_ext, nullptr);

  ros::NodeHandle nh("~");
  auto subs = ros_ext->create_subscriptions(nh);
  (void)subs;

  ayanami::actions::GoalResult<ayanami::actions::GetPathResult> get_path;
  get_path.status = ayanami::actions::GoalStatus::SUCCEEDED;
  get_path.result.path = make_path();
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(get_path);
  ASSERT_GE(pub_count("rviz_viewer_planner_path_pub_count"), 1);

  // ~RvizViewer → 4 个 callback id 全部 remove。注意 ext 与 ros_ext 是同一对象
  // 的两个 alias 引用, 需全部释放才触发析构。
  ext.reset();
  ros_ext.reset();
  ASSERT_EQ(ext, nullptr);
  ASSERT_EQ(ros_ext, nullptr);

  const int after_dtor = pub_count("rviz_viewer_planner_path_pub_count");
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(get_path);
  EXPECT_EQ(pub_count("rviz_viewer_planner_path_pub_count"), after_dtor);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rviz_viewer_test",
            ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
  ros::master::setRetryTimeout(ros::WallDuration(0.3));
  return RUN_ALL_TESTS();
}
