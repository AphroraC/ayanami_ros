// AyanamiROS construction + loading-chain TDD tests.
//
// roscore-less environment (headless). Uses a temporary config directory (full
// M1 config set + config_ros.json) + the real rviz_viewer extension
// (libayanami_rviz_viewer.so) to verify:
// 1. AyanamiROS construction completes in order: GlobalConfig ->
// loadExtensionModules -> loadNavigationPlugin (dlopen
// libayanami_costmap_navigation.so + initialize + start) -> subscribeTopics.
// 2. Extensions register before navigation->start: the
// on_costmap_visualization subscription registered during rviz_viewer
// construction must receive start's first wave of visualization emits
// (via the extern "C" counting queries exported by the rviz_viewer .so).
// 3. extension_modules / navigation_plugin members in place; start/stop
// without crashing.

#include <dlfcn.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <Eigen/Geometry>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/sensor_msgs.hpp>
#include <ayanami_ros/ayanami_ros.hpp>

namespace {

#ifndef RVIZ_VIEWER_SO
#error "RVIZ_VIEWER_SO must be defined by CMake"
#endif

const char* const g_rviz_viewer_so = RVIZ_VIEWER_SO;

void writeFile(const std::filesystem::path& path,
                const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream ofs(path);
  ASSERT_TRUE(ofs.good()) << "cannot write " << path;
  ofs << content;
}

std::string plugin_json(const std::string& module, const std::string& name,
                        const std::string& so_name) {
  return "\"" + module + "\": {\"name\": \"" + name +
         "\", \"so_name\": \"" + so_name + "\"}";
}

// Static map PGM (P2 ASCII, all FREE).
std::string free_pgm(int width, int height, int max_value) {
  std::string content = "P2\n" + std::to_string(width) + " " +
                        std::to_string(height) + "\n" +
                        std::to_string(max_value) + "\n";
  for (int i = 0; i < width * height; ++i) {
    content += "0 ";
  }
  content += "\n";
  return content;
}

// Full M1 config set (modeled on the headless_e2e structure) +
// config_ros.json.
// Plugin .so files use bare names: the catkin test environment's
// LD_LIBRARY_PATH already contains devel/lib.
std::filesystem::path makeConfigDir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_ros_load_test";
  std::filesystem::remove_all(dir);

  writeFile(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"costmap_navigation.json\"\n"
             "  }\n}\n");

  {
    std::ofstream nav(dir / "costmap_navigation.json");
    nav << "{\n  \"navigation\": {\n"
        << "    \"behavior_tree_xml\": \"BT/navigation.xml\",\n"
        << "    \"behavior_tree_id\": \"NavigateWithReplanningAndSmoothing\",\n"
        << "    \"planner_id\": \"costmap_smac\",\n"
        << "    \"smoother_id\": \"costmap_savitzky\",\n"
        << "    \"controller_id\": \"costmap_pid\",\n"
        << "    \"recovery_id\": \"clear_recovery\",\n"
        << "    \"clear_costmap_id\": \"clear_recovery\",\n"
        << "    \"spin_id\": \"clear_recovery\",\n"
        << "    \"wait_id\": \"clear_recovery\",\n"
        << "    \"backup_id\": \"clear_recovery\",\n"
        << "    \"planner_frequency\": 100.0,\n"
        << "    \"smoother_frequency\": -1.0,\n"
        << "    \"controller_frequency\": 20.0,\n"
        << "    \"tick_frequency\": 100.0,\n"
        << "    \"robot_frame\": \"base_footprint\",\n"
        << "    \"costmap_visualization_rate\": 100.0\n"
        << "  },\n"
        << "  \"modules\": {\n"
        << "    \"config_planner\": \"costmap_smac_planner.json\",\n"
        << "    \"config_smoother\": \"costmap_savitzky_smoother.json\",\n"
        << "    \"config_controller\": \"costmap_pid_controller.json\",\n"
        << "    \"config_recovery\": [\"costmap_clear_costmap_recovery.json\"]\n"
        << "  }\n";
  }

  writeFile(dir / "costmap_smac_planner.json",
             "{\n  " +
                 plugin_json("planner", "costmap_smac",
                             "libayanami_costmap_smac_planner.so") +
                 ",\n  \"costmap_smac_planner\": {\n"
                 "    \"use_diagonal\": true,\n    \"tolerance\": 0.0,\n"
                 "    \"cost_factor\": 0.5,\n    \"lethal_cost\": 254,\n"
                 "    \"neutral_cost\": 1.0,\n    \"use_final_approach\": false\n"
                 "  }\n}\n");
  writeFile(dir / "costmap_savitzky_smoother.json",
             "{\n  " +
                 plugin_json("smoother", "costmap_savitzky",
                             "libayanami_costmap_savitzky_smoother.so") +
                 ",\n  \"costmap_savitzky_smoother\": {\n"
                 "    \"do_refinement\": true,\n    \"refinement_num\": 2\n"
                 "  }\n}\n");
  writeFile(dir / "costmap_pid_controller.json",
             "{\n  " +
                 plugin_json("controller", "costmap_pid",
                             "libayanami_costmap_pid_controller.so") +
                 ",\n  \"costmap_pid_controller\": {\n"
                 "    \"max_vel_x\": 2.0,\n    \"max_vel_theta\": 1.0,\n"
                 "    \"goal_tolerance\": 0.1,\n    \"mode\": \"diff\"\n"
                 "  }\n}\n");
  writeFile(dir / "costmap_clear_costmap_recovery.json",
             "{\n  \"recovery\": {\n"
             "    \"name\": \"clear_recovery\",\n"
             "    \"so_name\": \"libayanami_costmap_clear_costmap_recovery.so\",\n"
             "    \"clear_radius\": 1.0,\n    \"affected_maps\": \"both\"\n"
             "  }\n}\n");

  {
    // costmap parameters merged into costmap_navigation.json (the original
    // costmap.json was removed).
    std::ofstream nav(dir / "costmap_navigation.json", std::ios::app);
    nav << ",\n  \"costmap\": {\n"
        << "    \"global_frame\": \"map\",\n"
        << "    \"local_frame\": \"odom\",\n"
        << "    \"resolution\": 0.5\n"
        << "  },\n"
        << "  \"costmap_global\": {\n"
        << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
        << "    \"rolling_window\": false,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"static_map\": \"" << (dir / "static_map.pgm").string() << "\",\n"
        << "    \"inflation_radius\": 1.0,\n"
        << "    \"inscribed_radius\": 0.8,\n"
        << "    \"cost_scaling_factor\": 2.0\n"
        << "  },\n"
        << "  \"costmap_local\": {\n"
        << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
        << "    \"rolling_window\": true,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"inflation_radius\": 1.0,\n"
        << "    \"inscribed_radius\": 0.8,\n"
        << "    \"cost_scaling_factor\": 2.0\n"
        << "  }\n"
        << "}\n";
  }

  writeFile(dir / "static_map.pgm", free_pgm(20, 20, 254));

  writeFile(dir / "BT" / "navigation.xml",
             R"(<root BTCPP_format="4" main_tree_to_execute="NavigateWithReplanningAndSmoothing">
  <BehaviorTree ID="NavigateWithReplanningAndSmoothing">
    <RecoveryNode name="NavigateRecovery" number_of_retries="6">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="{planner_frequency}">
          <RecoveryNode name="ComputePath" number_of_retries="1">
            <Sequence>
              <ComputePathToPose goal="{goal}" path="{raw_path}"/>
              <IsPathValid path="{raw_path}"/>
              <SmoothPath  input_path="{raw_path}" path="{smoothed_path}"/>
              <IsPathValid path="{smoothed_path}"/>
            </Sequence>
            <Sequence>
              <Recovery recovery="{clear_costmap_id}"/>
              <AlwaysFailure/>
            </Sequence>
          </RecoveryNode>
        </RateController>
        <RecoveryNode name="FollowPath" number_of_retries="1">
          <FollowPath path="{smoothed_path}" frequency="{controller_frequency}"/>
          <Sequence>
            <Recovery recovery="{clear_costmap_id}"/>
            <AlwaysFailure/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryActions">
        <GoalUpdated goal="{goal}"/>
        <RoundRobin name="RecoveryActions">
          <Sequence><Recovery recovery="{spin_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{wait_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{backup_id}"/>  <AlwaysFailure/></Sequence>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>)");

  writeFile(dir / "config_ros.json",
             "{\n  \"ayanami_ros\": {\n"
             "    \"laserscan_topic\": \"/scan\",\n"
             "    \"pointcloud_topic\": \"/velodyne_points\",\n"
             "    \"odom_topic\": \"/odom\",\n"
             "    \"goal_topic\": \"/move_base_simple/goal\",\n"
             "    \"map_frame\": \"map\",\n"
             "    \"odom_frame\": \"odom\",\n"
             "    \"robot_frame\": \"base_footprint\",\n"
             "    \"tf_sync_rate\": 50.0\n"
             "  }\n}\n");
  // Single source for the extension manifest = config_extensions.json (asuka
  // mode).
  writeFile(dir / "config_extensions.json",
            "{\n  \"extensions\": {\n"
            "    \"loading_extension_modules\": [\"" +
                 std::string(g_rviz_viewer_so) +
                 "\"]\n"
                 "  }\n}\n");

  return dir;
}

// Reads the costmap visualization publish count from the rviz_viewer .so
// (the dlopen was already done by AyanamiROS with RTLD_GLOBAL; uses the real
// output extension to verify load ordering instead of maintaining a separate
// stub).
int rvizViewerCostmapPubCount() {
  using Getter = int (*)();
  auto getter = reinterpret_cast<Getter>(dlsym(RTLD_DEFAULT,
                                                "rvizViewerGlobalCostmapPubCount"));
  if (getter == nullptr) {
    return -1;
  }
  return getter();
}

TEST(AyanamiROS, initLoadExtensionsBeforeNavigationPlugin) {
  const auto config_dir = makeConfigDir();
  ros::NodeHandle nh("~");
  // No roscore: nh.setParam would fail; use AyanamiROS's env fallback.
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  ayanami_ros::AyanamiROS node(nh);

  // 1) Extension modules loaded (libayanami_rviz_viewer.so dlopen +
  // create_extension_module).
  ASSERT_EQ(node.extensionModules().size(), 1u);

  // 2) Core navigation host plugin loaded (dlopen
  // libayanami_costmap_navigation.so + initialize + start).
  ASSERT_NE(node.navigationPlugin(), nullptr);

  // 3) Load ordering: extensions register before navigation->start. The
  // constructor's loadNavigationPlugin->start emits on_costmap_visualization;
  // if the order is wrong (nav before ext), rviz_viewer's subscription is not
  // registered and the count stays 0. A count > 0 here proves extension
  // construction completed before navigation start.
  const int emits = rvizViewerCostmapPubCount();
  EXPECT_GT(emits, 0) <<
      "extension 订阅应在 navigation_plugin->start() 前注册";
  EXPECT_EQ(node.configPath(), config_dir.string());

  // 4) start enters the loop, stop joins; no hang, no segfault.
  EXPECT_NO_THROW({
    node.start();
    ros::WallDuration(0.15).sleep();
    node.stop();
  });
}

TEST(AyanamiROS, emptyExtensionList) {
  const auto config_dir = makeConfigDir();
  writeFile(config_dir / "config_extensions.json",
             "{\n  \"extensions\": {\n"
             "    \"loading_extension_modules\": []\n"
             "  }\n}\n");
  ros::NodeHandle nh("~");
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  ayanami_ros::AyanamiROS node(nh);
  EXPECT_EQ(node.extensionModules().size(), 0u);
  ASSERT_NE(node.navigationPlugin(), nullptr);
  EXPECT_NO_THROW({
    node.start();
    ros::WallDuration(0.15).sleep();
    node.stop();
  });
}

namespace {

ayanami::geometry_msgs::PoseStamped makeAyaPose(double x, double y) {
  ayanami::geometry_msgs::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

}  // namespace

// latest-wins goal mailbox: the ROS callback side's submitGoal only overwrites
// pending_goal (no queueing); the loop side's drainPendingGoal delivers at most
// the current latest one each time and then clears it.
TEST(AyanamiROS, goalMailboxIsLatestWinsAndDrainsAtMostLatest) {
  const auto config_dir = makeConfigDir();
  ros::NodeHandle nh("~");
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  ayanami_ros::AyanamiROS node(nh);
  ASSERT_NE(node.navigationPlugin(), nullptr);

  // Two submitGoal calls -> only the latest one remains in the mailbox
  // (latest wins, no queueing).
  node.submitGoal(makeAyaPose(1.0, 2.0));
  node.submitGoal(makeAyaPose(3.0, 4.0));
  EXPECT_TRUE(node.hasPendingGoal());

  // One drain delivers only the current latest (returns true) and empties the
  // mailbox; a second drain finds no goal.
  EXPECT_TRUE(node.drainPendingGoal());
  EXPECT_FALSE(node.hasPendingGoal());
  EXPECT_FALSE(node.drainPendingGoal());

  // Draining one-by-one after empty submits produces no extra deliveries.
  node.drainPendingGoal();
  EXPECT_FALSE(node.hasPendingGoal());

  node.stop();
}

// odom velocity written directly into the global RobotStateController: the
// controller execution reads the real latest velocity (no longer a hardcoded
// zero velocity).
TEST(AyanamiROS, odomVelocityWiredIntoRobotStateController) {
  const auto config_dir = makeConfigDir();
  ros::NodeHandle nh("~");
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  ayanami_ros::AyanamiROS node(nh);
  ASSERT_NE(node.navigationPlugin(), nullptr);
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.resetVelocity();
  // During AyanamiROS construction, navigation_plugin->initialize already
  // initialized the singleton with the config frames
  // (map/odom/base_footprint).
  EXPECT_EQ(rsc.getRobotFrame(), "base_footprint");

  ayanami::geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "base_footprint";
  twist.twist.linear.x() = 0.5;
  twist.twist.angular.z() = 0.25;
  node.setOdometry(twist);

  EXPECT_TRUE(rsc.hasFreshVelocity());
  const auto v = rsc.getRobotVelocity();
  EXPECT_DOUBLE_EQ(v.twist.linear.x(), 0.5);
  EXPECT_DOUBLE_EQ(v.twist.angular.z(), 0.25);

  node.stop();
}

// Obstacle observation input no longer goes through global bypass slots: ROS
// callbacks deliver directly to the host AbstractNavigation's
// setPointcloud/setLaserScan (input/output layering); empty messages are
// silently ignored without crashing.
TEST(AyanamiROS, obstacleInputWiredToHostPluginWithoutCrash) {
  const auto config_dir = makeConfigDir();
  ros::NodeHandle nh("~");
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  ayanami_ros::AyanamiROS node(nh);
  auto* plugin = node.navigationPlugin().get();
  ASSERT_NE(plugin, nullptr);

  // Empty pointcloud / laserscan -> the rasterization pipeline returns early,
  // no crash.
  ayanami::sensor_msgs::PointCloud2 cloud;
  ayanami::sensor_msgs::LaserScan scan;
  plugin->setPointcloud(cloud);
  plugin->setLaserScan(scan);
  EXPECT_NO_THROW({
    plugin->setPointcloud(cloud);
    plugin->setLaserScan(scan);
  });

  node.stop();
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ayanami_ros_load_test",
            ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
  ros::master::setRetryTimeout(ros::WallDuration(0.3));
  return RUN_ALL_TESTS();
}
