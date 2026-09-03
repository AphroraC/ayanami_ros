// Task 9: rviz_viewer ROS extension .so (dlopen → create_extension_module →
// RvizViewer : ayanami::ExtensionModule)。
//
// 无 roscore 环境 (headless)。用临时 config 目录 (含 config_ros.json, 覆盖 rviz
// 话题名) + dlopen librviz_viewer.so, 验证:
//   1. dlopen + create_extension_module + dynamic_cast<ayanami::ExtensionModule>。
//   2. 构造期 (dlopen 时) 自建 ros::NodeHandle → advertiseTopics (16 publisher)
//      + 注册 15 个 NavigationCallbacks 订阅 (ayanami 全部输出槽位); 扩展自身
//      不 ros subscribe (输入职责归 AyanamiROS)。
//   3. 主动 emit NavigationCallbacks 事件 → 对应 publish handler 被调 (经 .so 导出
//      的计数 hook 证明), 无订阅者时 publish 为 no-op, 无崩。
//   4. 话题名从 config_ros.json::ayanami_ros 读入 (planner_path_topic 覆盖验证)。
//   5. ~RvizViewer → 15 个 callback id 全部 remove → 再 emit 计数不再增。

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

namespace {

#ifndef RVIZ_VIEWER_SO
#error "RVIZ_VIEWER_SO must be defined by CMake"
#endif

const char* const g_rviz_viewer_so = RVIZ_VIEWER_SO;

void writeFile(const std::filesystem::path& path, const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream ofs(path);
  ASSERT_TRUE(ofs.good()) << "cannot write " << path;
  ofs << content;
}

std::filesystem::path makeConfigDir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_ros_rviz_viewer_test";
  std::filesystem::remove_all(dir);
  writeFile(dir / "config.json",
            "{\n  \"global\": {\n"
            "    \"config_ros\": \"config_ros.json\"\n"
            "  }\n}\n");
  writeFile(dir / "config_ros.json",
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

void initGlobalConfig(const std::filesystem::path& dir) {
  setenv("AYANAMI_CONFIG_PATH", dir.string().c_str(), 1);
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);
}

// 从 librviz_viewer.so 读 publish 计数 / 话题名 (dlopen 由 load_module RTLD_GLOBAL)。
int pubCount(const char* symbol) {
  using Getter = int (*)();
  auto getter = reinterpret_cast<Getter>(dlsym(RTLD_DEFAULT, symbol));
  if (getter == nullptr) {
    return -1;
  }
  return getter();
}

const char* topicString(const char* symbol) {
  using Getter = const char* (*)();
  auto getter = reinterpret_cast<Getter>(dlsym(RTLD_DEFAULT, symbol));
  if (getter == nullptr) {
    return nullptr;
  }
  return getter();
}

ayanami::nav_msgs::Path makePath() {
  ayanami::nav_msgs::Path path;
  path.header.frame_id = "map";
  ayanami::geometry_msgs::PoseStamped pose =
      ayanami::geometry_msgs::PoseStamped::fromIsometry(
          "map", Eigen::Isometry3d::Identity());
  pose.pose.position = Eigen::Vector3d(1.0, 2.0, 0.0);
  path.poses.push_back(pose);
  return path;
}

TEST(RvizViewer, dlopenCreateAndRegisterCallbacks) {
  initGlobalConfig(makeConfigDir());

  auto ext = ayanami::ExtensionModule::load_module(g_rviz_viewer_so);
  ASSERT_NE(ext, nullptr);
  auto viewer = std::dynamic_pointer_cast<ayanami::ExtensionModule>(ext);
  ASSERT_NE(viewer, nullptr);

  // 构造期 advertiseTopics 完成: 计数 hook 均可达 (dlsym 找到, 初值 0)。
  EXPECT_GE(pubCount("rvizViewerPlannerPathPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerSmootherPathPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerVelocityPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerGlobalCostmapPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerLocalCostmapPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerPlannerGoalPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerPlannerFeedbackPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerSmootherGoalPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerSmootherFeedbackPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerExePathGoalPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerExePathFeedbackPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerExePathResultPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerRecoveryGoalPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerRecoveryFeedbackPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerRecoveryResultPubCount"), 0);

  // 话题名从 config_ros.json::ayanami_ros 读入 (覆盖默认值)。
  const char* topic = topicString("rvizViewerPlannerPathTopic");
  ASSERT_NE(topic, nullptr);
  EXPECT_STREQ(topic, "/test/rviz/planner/path");
}

TEST(RvizViewer, emitCallbacksPublishRos) {
  initGlobalConfig(makeConfigDir());

  auto ext = ayanami::ExtensionModule::load_module(g_rviz_viewer_so);
  ASSERT_NE(ext, nullptr);

  // 主动 emit NavigationCallbacks → 对应 publish handler 被调 ≥1。
  ayanami::actions::GoalResult<ayanami::actions::GetPathResult> get_path;
  get_path.status = ayanami::actions::GoalStatus::SUCCEEDED;
  get_path.result.path = makePath();
  const int planner_before = pubCount("rvizViewerPlannerPathPubCount");
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(get_path);
  EXPECT_GE(pubCount("rvizViewerPlannerPathPubCount"), planner_before + 1);

  ayanami::actions::GoalResult<ayanami::actions::SmoothPathResult> smooth;
  smooth.status = ayanami::actions::GoalStatus::SUCCEEDED;
  smooth.result.smoothed_path = makePath();
  const int smoother_before = pubCount("rvizViewerSmootherPathPubCount");
  ayanami::interfaces::NavigationCallbacks::on_smooth_path_result(smooth);
  EXPECT_GE(pubCount("rvizViewerSmootherPathPubCount"), smoother_before + 1);

  ayanami::geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "odom";
  twist.twist.linear = Eigen::Vector3d(1.0, 0.0, 0.0);
  const int velocity_before = pubCount("rvizViewerVelocityPubCount");
  ayanami::interfaces::NavigationCallbacks::on_velocity_command(twist);
  EXPECT_GE(pubCount("rvizViewerVelocityPubCount"), velocity_before + 1);

  ayanami::nav_msgs::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.5;
  grid.info.width = 4;
  grid.info.height = 4;
  grid.data.assign(16, 0);
  const int global_before = pubCount("rvizViewerGlobalCostmapPubCount");
  ayanami::interfaces::NavigationCallbacks::on_costmap_visualization(
      "global_costmap", grid);
  EXPECT_GE(pubCount("rvizViewerGlobalCostmapPubCount"), global_before + 1);
  const int local_before = pubCount("rvizViewerLocalCostmapPubCount");
  ayanami::interfaces::NavigationCallbacks::on_costmap_visualization(
      "local_costmap", grid);
  EXPECT_GE(pubCount("rvizViewerLocalCostmapPubCount"), local_before + 1);

  // 非 SUCCEEDED 的 path 不 publish。
  ayanami::actions::GoalResult<ayanami::actions::GetPathResult> failed;
  failed.status = ayanami::actions::GoalStatus::ABORTED;
  failed.result.path = makePath();
  const int planner_after = pubCount("rvizViewerPlannerPathPubCount");
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(failed);
  EXPECT_EQ(pubCount("rvizViewerPlannerPathPubCount"), planner_after);

  // ---- 扩展输出槽位 (spec §4.5 扩展): goal/feedback/result 全覆盖 ----
  ayanami::geometry_msgs::PoseStamped goal_pose =
      ayanami::geometry_msgs::PoseStamped::fromIsometry(
          "map", Eigen::Isometry3d::Identity());
  ayanami::actions::GetPathGoal planner_goal;
  planner_goal.goal = goal_pose;
  const int pg_before = pubCount("rvizViewerPlannerGoalPubCount");
  ayanami::interfaces::NavigationCallbacks::on_get_path_goal(planner_goal);
  EXPECT_GE(pubCount("rvizViewerPlannerGoalPubCount"), pg_before + 1);

  ayanami::actions::GetPathFeedback planner_fb;
  planner_fb.message = "planning";
  const int pf_before = pubCount("rvizViewerPlannerFeedbackPubCount");
  ayanami::interfaces::NavigationCallbacks::on_get_path_feedback(planner_fb);
  EXPECT_GE(pubCount("rvizViewerPlannerFeedbackPubCount"), pf_before + 1);

  ayanami::actions::SmoothPathGoal smoother_goal;
  smoother_goal.path = makePath();
  const int sg_before = pubCount("rvizViewerSmootherGoalPubCount");
  ayanami::interfaces::NavigationCallbacks::on_smooth_path_goal(smoother_goal);
  EXPECT_GE(pubCount("rvizViewerSmootherGoalPubCount"), sg_before + 1);

  ayanami::actions::SmoothPathFeedback smoother_fb;
  smoother_fb.message = "smoothing";
  const int sf_before = pubCount("rvizViewerSmootherFeedbackPubCount");
  ayanami::interfaces::NavigationCallbacks::on_smooth_path_feedback(smoother_fb);
  EXPECT_GE(pubCount("rvizViewerSmootherFeedbackPubCount"), sf_before + 1);

  ayanami::actions::ExePathGoal exe_goal;
  exe_goal.path = makePath();
  const int eg_before = pubCount("rvizViewerExePathGoalPubCount");
  ayanami::interfaces::NavigationCallbacks::on_exe_path_goal(exe_goal);
  EXPECT_GE(pubCount("rvizViewerExePathGoalPubCount"), eg_before + 1);

  ayanami::actions::ExePathFeedback exe_fb;
  exe_fb.message = "tracking";
  const int ef_before = pubCount("rvizViewerExePathFeedbackPubCount");
  ayanami::interfaces::NavigationCallbacks::on_exe_path_feedback(exe_fb);
  EXPECT_GE(pubCount("rvizViewerExePathFeedbackPubCount"), ef_before + 1);

  ayanami::actions::GoalResult<ayanami::actions::ExePathResult> exe_result;
  exe_result.status = ayanami::actions::GoalStatus::SUCCEEDED;
  exe_result.result.failure = ayanami::FailureCode::None;
  exe_result.result.reached = true;
  exe_result.result.message = "reached";
  const int er_before = pubCount("rvizViewerExePathResultPubCount");
  ayanami::interfaces::NavigationCallbacks::on_exe_path_result(exe_result);
  EXPECT_GE(pubCount("rvizViewerExePathResultPubCount"), er_before + 1);

  ayanami::actions::RecoveryGoal recovery_goal;
  recovery_goal.recovery_id = "spin";
  const int rg_before = pubCount("rvizViewerRecoveryGoalPubCount");
  ayanami::interfaces::NavigationCallbacks::on_recovery_goal(recovery_goal);
  EXPECT_GE(pubCount("rvizViewerRecoveryGoalPubCount"), rg_before + 1);

  ayanami::actions::RecoveryFeedback recovery_fb;
  recovery_fb.message = "spinning";
  const int rf_before = pubCount("rvizViewerRecoveryFeedbackPubCount");
  ayanami::interfaces::NavigationCallbacks::on_recovery_feedback(recovery_fb);
  EXPECT_GE(pubCount("rvizViewerRecoveryFeedbackPubCount"), rf_before + 1);

  ayanami::actions::GoalResult<ayanami::actions::RecoveryResult> recovery_result;
  recovery_result.status = ayanami::actions::GoalStatus::SUCCEEDED;
  recovery_result.result.failure = ayanami::FailureCode::None;
  const int rr_before = pubCount("rvizViewerRecoveryResultPubCount");
  ayanami::interfaces::NavigationCallbacks::on_recovery_result(recovery_result);
  EXPECT_GE(pubCount("rvizViewerRecoveryResultPubCount"), rr_before + 1);

}

TEST(RvizViewer, destructorRemovesCallbacks) {
  initGlobalConfig(makeConfigDir());

  auto ext = ayanami::ExtensionModule::load_module(g_rviz_viewer_so);
  ASSERT_NE(ext, nullptr);

  ayanami::actions::GoalResult<ayanami::actions::GetPathResult> get_path;
  get_path.status = ayanami::actions::GoalStatus::SUCCEEDED;
  get_path.result.path = makePath();
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(get_path);
  ASSERT_GE(pubCount("rvizViewerPlannerPathPubCount"), 1);

  // ~RvizViewer → 15 个 callback id 全部 remove。释放全部 alias 引用触发析构。
  ext.reset();
  ASSERT_EQ(ext, nullptr);

  const int after_dtor = pubCount("rvizViewerPlannerPathPubCount");
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(get_path);
  EXPECT_EQ(pubCount("rvizViewerPlannerPathPubCount"), after_dtor);

  // 扩展槽位同样全部退订: 析构后再 emit on_exe_path_result 计数不再增。
  ayanami::actions::GoalResult<ayanami::actions::ExePathResult> exe_result;
  exe_result.status = ayanami::actions::GoalStatus::SUCCEEDED;
  exe_result.result.reached = true;
  const int after_dtor_exe = pubCount("rvizViewerExePathResultPubCount");
  ayanami::interfaces::NavigationCallbacks::on_exe_path_result(exe_result);
  EXPECT_EQ(pubCount("rvizViewerExePathResultPubCount"), after_dtor_exe);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rviz_viewer_test",
            ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
  ros::master::setRetryTimeout(ros::WallDuration(0.3));
  return RUN_ALL_TESTS();
}
