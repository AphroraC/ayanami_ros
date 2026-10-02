// rviz_viewer ROS extension .so (dlopen -> create_extension_module ->
// RvizViewer : ayanami::ExtensionModule).
//
// roscore-less environment (headless). Uses a temporary config directory
// (containing config_extensions.json, overriding the rviz topic names) +
// dlopen librviz_viewer.so to verify:
// 1. dlopen + create_extension_module + dynamic_cast<ayanami::ExtensionModule>.
// 2. At construction time (during dlopen) it creates its own ros::NodeHandle ->
// advertiseTopics (16 publishers) + registers 15 NavigationCallbacks
// subscriptions (all of ayanami's output slots); the extension itself does no
// ros subscribe (input duty belongs to AyanamiROS).
// 3. Actively emitting NavigationCallbacks events -> the corresponding publish
// handlers are invoked (proven by the counting hooks exported from the .so);
// publish is a no-op with no subscribers, without crashing.
// 4. Topic names are read from config_extensions.json::rviz_viewer
// (planner_path_topic override verified); config_ros.json no longer carries
// extension output topics.
// 5. ~RvizViewer -> all 15 callback ids removed -> further emits no longer
// increase the counts.

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

/// @brief Load the module the way the production loader does (context injected
/// after a successful load): parameters arrive through configureParameters, so
/// the 1-arg load would leave the module on its built-in defaults.
ayanami::ExtensionModule::Ptr loadRvizViewer() {
  return ayanami::ExtensionModule::loadModule(
      g_rviz_viewer_so,
      ayanami::ModuleContext{
          "rviz_viewer",
          ayanami::Config(ayanami::GlobalConfig::getConfigPath("config_extensions"))});
}

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
            "    \"config_extensions\": \"config_extensions.json\"\n"
            "  }\n}\n");
  // Single source for extension output topics =
  // config_extensions.json::rviz_viewer (asuka mode).
  writeFile(dir / "config_extensions.json",
            "{\n  \"extensions\": {\n"
            "    \"loading_extension_modules\": []\n"
            "  },\n"
            "  \"rviz_viewer\": {\n"
            "    \"planner_path_topic\": \"/test/rviz/planner/path\",\n"
            "    \"smoother_path_topic\": \"/test/rviz/smoother/path\",\n"
            "    \"cmd_vel_topic\": \"/test/rviz/cmd_vel\"\n"
            "  }\n}\n");
  return dir;
}

void initGlobalConfig(const std::filesystem::path& dir) {
  setenv("AYANAMI_CONFIG_PATH", dir.string().c_str(), 1);
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);
}

// Reads publish counts / topic names from librviz_viewer.so (dlopen done by
// loadModule with RTLD_GLOBAL).
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

  auto ext = loadRvizViewer();
  ASSERT_NE(ext, nullptr);
  auto viewer = std::dynamic_pointer_cast<ayanami::ExtensionModule>(ext);
  ASSERT_NE(viewer, nullptr);

  // advertiseTopics completed at construction: all counting hooks reachable
  // (found via dlsym, initial value 0).
  EXPECT_GE(pubCount("rvizViewerPlannerPathPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerSmootherPathPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerPlannerGoalPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerPlannerFeedbackPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerSmootherFeedbackPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerExePathFeedbackPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerExePathResultPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerRecoveryGoalPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerRecoveryFeedbackPubCount"), 0);
  EXPECT_GE(pubCount("rvizViewerRecoveryResultPubCount"), 0);

  // Topic names read from config_extensions.json::rviz_viewer (overriding the
  // defaults).
  const char* topic = topicString("rvizViewerPlannerPathTopic");
  ASSERT_NE(topic, nullptr);
  EXPECT_STREQ(topic, "/test/rviz/planner/path");
}

TEST(RvizViewer, emitCallbacksPublishRos) {
  initGlobalConfig(makeConfigDir());

  auto ext = loadRvizViewer();
  ASSERT_NE(ext, nullptr);

  // Actively emit NavigationCallbacks -> the corresponding publish handler is
  // invoked at least once.
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

  // Non-SUCCEEDED paths are not published.
  ayanami::actions::GoalResult<ayanami::actions::GetPathResult> failed;
  failed.status = ayanami::actions::GoalStatus::ABORTED;
  failed.result.path = makePath();
  const int planner_after = pubCount("rvizViewerPlannerPathPubCount");
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(failed);
  EXPECT_EQ(pubCount("rvizViewerPlannerPathPubCount"), planner_after);

  // ---- extension output slots (extension): full goal/feedback/result
  // coverage ----
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

  ayanami::actions::SmoothPathFeedback smoother_fb;
  smoother_fb.message = "smoothing";
  const int sf_before = pubCount("rvizViewerSmootherFeedbackPubCount");
  ayanami::interfaces::NavigationCallbacks::on_smooth_path_feedback(smoother_fb);
  EXPECT_GE(pubCount("rvizViewerSmootherFeedbackPubCount"), sf_before + 1);

  ayanami::actions::ExePathFeedback exe_fb;
  exe_fb.message = "tracking";
  const int ef_before = pubCount("rvizViewerExePathFeedbackPubCount");
  ayanami::interfaces::NavigationCallbacks::on_exe_path_feedback(exe_fb);
  EXPECT_GE(pubCount("rvizViewerExePathFeedbackPubCount"), ef_before + 1);

  ayanami::actions::GoalResult<ayanami::actions::ExePathResult> exe_result;
  exe_result.status = ayanami::actions::GoalStatus::SUCCEEDED;
  exe_result.result.error = ayanami::FailureCode::NONE;
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
  recovery_result.result.error = ayanami::FailureCode::NONE;
  const int rr_before = pubCount("rvizViewerRecoveryResultPubCount");
  ayanami::interfaces::NavigationCallbacks::on_recovery_result(recovery_result);
  EXPECT_GE(pubCount("rvizViewerRecoveryResultPubCount"), rr_before + 1);

}

TEST(RvizViewer, destructorRemovesCallbacks) {
  initGlobalConfig(makeConfigDir());

  auto ext = loadRvizViewer();
  ASSERT_NE(ext, nullptr);

  ayanami::actions::GoalResult<ayanami::actions::GetPathResult> get_path;
  get_path.status = ayanami::actions::GoalStatus::SUCCEEDED;
  get_path.result.path = makePath();
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(get_path);
  ASSERT_GE(pubCount("rvizViewerPlannerPathPubCount"), 1);

  // ~RvizViewer -> all 15 callback ids removed. Releasing all alias references
  // triggers the destructor.
  ext.reset();
  ASSERT_EQ(ext, nullptr);

  const int after_dtor = pubCount("rvizViewerPlannerPathPubCount");
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(get_path);
  EXPECT_EQ(pubCount("rvizViewerPlannerPathPubCount"), after_dtor);

  // Extension slots are fully unsubscribed as well: emitting
  // on_exe_path_result after destruction no longer increases the count.
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
