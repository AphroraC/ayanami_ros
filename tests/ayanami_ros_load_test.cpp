// AyanamiROS construction + loading-chain TDD tests.
//
// roscore-less environment (headless). Uses a temporary config directory (full
// M1 config set + config_ros.json) + the real rviz_viewer extension
// (libayanami_rviz_viewer.so) to verify:
// 1. AyanamiROS construction completes in order: GlobalConfig ->
// loadExtensionModules -> loadNavigationModule (dlopen
// libayanami_rose_navigation_fsm.so + initialize + start) -> subscribeTopics.
// 2. Extensions register before navigation->start: the per-layer costmap
// slot subscriptions registered during rose_viewer construction must
// receive initialize's configure-time artifact emission (via the extern "C"
// counting queries exported by the rose_viewer .so).
// 3. extension_modules / navigation_module members in place; start/stop
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
const char* const g_rose_viewer_so = ROSE_VIEWER_SO;

void writeFile(const std::filesystem::path& path,
                const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream ofs(path);
  ASSERT_TRUE(ofs.good()) << "cannot write " << path;
  ofs << content;
}

std::string module_json(const std::string& module, const std::string& name,
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

// FSM-era config set (modeled on the fsm e2e structure) + config_ros.json.
// Module .so files use bare names: the catkin test environment's
// LD_LIBRARY_PATH already contains devel/lib.
std::filesystem::path makeConfigDir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_ros_load_test";
  std::filesystem::remove_all(dir);

  writeFile(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"fsm_navigation.json\"\n"
             "  }\n}\n");

  {
    std::ofstream nav(dir / "fsm_navigation.json");
    nav << "{\n  \"navigation\": {\n"
        << "    \"so_name\": \"libayanami_rose_navigation_fsm.so\",\n"
        << "    \"config_planner\": \"planner.json\",\n"
        << "    \"config_smoother\": \"smoother.json\",\n"
        << "    \"config_controller\": \"controller.json\",\n"
        << "    \"config_recovery\": [\"recovery.json\"],\n"
        << "    \"config_goal_checker\": \"goal_checker.json\",\n"
        << "    \"config_progress_checker\": \"progress_checker.json\",\n"
        << "    \"fsm_frequency\": 50.0,\n"
        << "    \"planner_frequency\": -1.0,\n"
        << "    \"smoother_frequency\": -1.0,\n"
        << "    \"smoother_timeout\": 2.0,\n"
        << "    \"controller_frequency\": 20.0,\n"
        << "    \"max_nav_retries\": 6,\n"
        << "    \"oscillation_distance\": 0.1,\n"
        << "    \"oscillation_timeout\": 3600.0,\n"
        << "    \"controller_patience\": -1.0,\n"
        << "    \"recovery_patience\": 0.0,\n"
        << "    \"max_vel_x\": 2.0,\n    \"max_vel_y\": 2.0,\n"
        << "    \"max_vel_theta\": 2.0,\n"
        << "    \"robot_frame\": \"base_footprint\",\n"
        << "    \"velocity_timeout\": 0.25\n"
        << "  },\n"
        << "  \"esdfmap\": {\n"
        << "    \"max_distance\": 3.0,\n"
        << "    \"update_rate\": 50.0,\n"
        << "    \"stale_timeout\": 0.0\n"
        << "  },\n"
        << "  \"costmap\": {\n"
        << "    \"global_frame\": \"map\",\n"
        << "    \"local_frame\": \"odom\",\n"
        << "    \"robot_frame\": \"base_footprint\",\n"
        << "    \"resolution\": 0.5\n"
        << "  },\n"
        << "  \"costmap_global\": {\n"
        << "    \"rolling_window\": false,\n    \"track_unknown\": false,\n"
        << "    \"static_layer\": {\n"
        << "      \"static_map\": \"" << (dir / "static_map.pgm").string()
        << "\"\n"
        << "    },\n"
        << "    \"inflation_layer\": {\n"
        << "      \"inflation_radius\": 1.0, \"inscribed_radius\": 0.8, "
           "\"cost_scaling_factor\": 2.0\n"
        << "    }\n"
        << "  },\n"
        << "  \"costmap_local\": {\n"
        << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
        << "    \"rolling_window\": true,\n    \"track_unknown\": false,\n"
        << "    \"obstacle_layer\": {\n"
        << "      \"combination\": \"overwrite\", "
           "\"footprint_clearing\": true,\n"
        << "      \"observation_sources\": [\"scan\"],\n"
        << "      \"scan\": {\"type\": \"laserscan\"}\n"
        << "    },\n"
        << "    \"inflation_layer\": {\n"
        << "      \"inflation_radius\": 1.0, \"inscribed_radius\": 0.8, "
           "\"cost_scaling_factor\": 2.0\n"
        << "    }\n"
        << "  }\n"
        << "}\n";
  }

  writeFile(dir / "planner.json",
             "{\n  " +
                 module_json("planner", "costmap_astar_planner",
                             "libayanami_costmap_astar_planner.so") +
                 ",\n  \"costmap_astar_planner\": {}\n}\n");
  writeFile(dir / "smoother.json",
             "{\n  " +
                 module_json("smoother", "esdfmap_minco_smoother",
                             "libayanami_esdfmap_minco_smoother.so") +
                 ",\n  \"esdfmap_minco_smoother\": {}\n}\n");
  writeFile(dir / "controller.json",
             "{\n  " +
                 module_json("controller", "abstract_pid_controller",
                             "libayanami_abstract_pid_controller.so") +
                 ",\n  \"abstract_pid_controller\": {\n"
                 "    \"max_vel_x\": 2.0,\n    \"max_vel_theta\": 1.0,\n"
                 "    \"goal_tolerance\": 0.1,\n    \"mode\": \"diff\"\n"
                 "  }\n}\n");
  writeFile(dir / "recovery.json",
             "{\n  " +
                 module_json("recovery", "abstract_nothing_recovery",
                             "libayanami_abstract_nothing_recovery.so") +
                 ",\n  \"abstract_nothing_recovery\": {}\n}\n");

  writeFile(dir / "static_map.pgm", free_pgm(20, 20, 254));
  // Core contract: a static map is a PGM + JSON metadata PAIR (nav.json
  // semantics: resolution + origin); without the metadata the
  // configureCoreMaps static-layer load fails.
  writeFile(dir / "static_map.json",
            R"({"map": {"resolution": 0.5, "origin": [0.0, 0.0, 0.0]}})");

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
                 "\", \"" +
                 std::string(g_rose_viewer_so) +
                 "\"]\n"
                 "  }\n}\n");

  return dir;
}

// Reads the global-inflation publish count from the rose_viewer .so
// (the dlopen was already done by AyanamiROS with RTLD_GLOBAL; uses the real
// output extension to verify load ordering instead of maintaining a separate
// stub).
int roseViewerGlobalInflationPubCount() {
  using Getter = int (*)();
  auto getter = reinterpret_cast<Getter>(dlsym(RTLD_DEFAULT,
                                                "roseViewerGlobalInflationPubCount"));
  if (getter == nullptr) {
    return -1;
  }
  return getter();
}

TEST(AyanamiROS, initLoadExtensionsBeforeNavigationModule) {
  const auto config_dir = makeConfigDir();
  ros::NodeHandle nh("~");
  // No roscore: nh.setParam would fail; use AyanamiROS's env fallback.
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  ayanami_ros::AyanamiROS node(nh);

  // 1) Extension modules loaded (libayanami_rviz_viewer.so +
  // libayanami_rose_viewer.so dlopen + create_extension_module).
  ASSERT_EQ(node.extensionModules().size(), 2u);

  // 2) Core navigation host module loaded (dlopen
  // libayanami_rose_navigation_fsm.so + initialize + start).
  ASSERT_NE(node.navigationModule(), nullptr);

  // 3) Load ordering: extensions register before navigation->start.
  // initialize's configureCoreMaps emits the per-layer artifacts (a static
  // global may fire only once); if the order is wrong (nav before ext),
  // rose_viewer's subscription is not registered and the count stays 0.
  // A count > 0 here proves extension construction completed before
  // navigation start.
  // The emission->publish chain is asynchronous (the viewer's own publisher
  // thread; the core emits on its writer thread), so poll with a deadline
  // instead of asserting synchronously. If the order were wrong (nav before
  // ext), the subscription would miss the configure-time emission entirely
  // and the count would stay 0 -- the timeout then fails the test, keeping
  // the ordering property locked.
  int emits = roseViewerGlobalInflationPubCount();
  {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(2);
    while (emits <= 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      emits = roseViewerGlobalInflationPubCount();
    }
  }
  EXPECT_GT(emits, 0) <<
      "extension subscriptions must register before navigation_module->start()";
  EXPECT_EQ(node.configPath(), config_dir.string());

  // 4) start enters the loop, stop joins; no hang, no segfault.
  EXPECT_NO_THROW({
    node.start();
    ros::WallDuration(0.15).sleep();
    node.stop();
  });
}

TEST(AyanamiROS, robotFrameMismatchRefusesToStart) {
  // robot_frame is owned by the navigation config; config_ros only mirrors it.
  // The check used to live in the core's CostmapNavigationServer as a
  // spdlog::error + assert -- and the assert was compiled out in Release, so a
  // mismatch silently built TF edges around two different robot frames.
  const auto config_dir = makeConfigDir();
  ros::NodeHandle nh("~");
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  // Re-point config_ros at a different robot frame than costmap_navigation.json.
  writeFile(config_dir / "config_ros.json",
            "{\n  \"ayanami_ros\": {\n"
            "    \"laserscan_topic\": \"/scan\",\n"
            "    \"pointcloud_topic\": \"/velodyne_points\",\n"
            "    \"odom_topic\": \"/odom\",\n"
            "    \"goal_topic\": \"/move_base_simple/goal\",\n"
            "    \"map_frame\": \"map\",\n"
            "    \"odom_frame\": \"odom\",\n"
            "    \"robot_frame\": \"base_link_mismatch\",\n"
            "    \"tf_sync_rate\": 50.0\n"
            "  }\n}\n");

  EXPECT_THROW(ayanami_ros::AyanamiROS node(nh), std::runtime_error);
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
  ASSERT_NE(node.navigationModule(), nullptr);
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
  ASSERT_NE(node.navigationModule(), nullptr);

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
  ASSERT_NE(node.navigationModule(), nullptr);
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.resetVelocity();
  // During AyanamiROS construction, navigation_module->initialize already
  // initialized the singleton with the config frames
  // (map/odom/base_footprint).
  EXPECT_EQ(rsc.robotFrame(), "base_footprint");

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
TEST(AyanamiROS, obstacleInputWiredToHostModuleWithoutCrash) {
  const auto config_dir = makeConfigDir();
  ros::NodeHandle nh("~");
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  ayanami_ros::AyanamiROS node(nh);
  auto* module = node.navigationModule().get();
  ASSERT_NE(module, nullptr);

  // Empty pointcloud / laserscan -> the rasterization pipeline returns early,
  // no crash.
  ayanami::sensor_msgs::PointCloud2 cloud;
  ayanami::sensor_msgs::LaserScan scan;
  module->setPointcloud(cloud);
  module->setLaserScan(scan);
  EXPECT_NO_THROW({
    module->setPointcloud(cloud);
    module->setLaserScan(scan);
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
