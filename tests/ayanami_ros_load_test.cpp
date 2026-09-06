// AyanamiROS 构造 + 加载链 TDD 测试。
//
// 无 roscore 环境 (headless)。用临时 config 目录 (全套 M1 配置 + config_ros.json)
// + 真实 rviz_viewer 扩展 (libayanami_rviz_viewer.so), 验证:
// 1. AyanamiROS 构造按 顺序完成: GlobalConfig → loadExtensionModules
// → loadNavigationPlugin (dlopen libayanami_costmap_navigation.so +
// initialize + start) → subscribeTopics。
// 2. 扩展先于 navigation->start 注册 : rviz_viewer 构造期注册的
// on_costmap_visualization 订阅必须收到 start 的首波可视化 emit
// (通过 rviz_viewer .so 导出的 extern "C" 计数查询)。
// 3. extension_modules / navigation_plugin 成员就位; start/stop 无崩。

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

// 静态地图 PGM (P2 ASCII, 全 FREE)。
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

// 全套 M1 配置 (对标 , headless_e2e 结构) + config_ros.json。
// 插件 so 用裸名: catkin 测试环境 LD_LIBRARY_PATH 已含 devel/lib。
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
    // costmap 参数并入 costmap_navigation.json (原 costmap.json 移除)。
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
             "    \"tf_sync_rate\": 50.0,\n"
              "    \"extension_modules\": [\"" +
                 std::string(g_rviz_viewer_so) +
                 "\"]\n"
                 "  }\n}\n");

  return dir;
}

// 从 rviz_viewer .so 读 costmap 可视化 publish 计数 (dlopen 已由 AyanamiROS
// 以 RTLD_GLOBAL 完成; 用真实输出扩展验证加载顺序, 不再单独维护 stub)。
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
  // 无 roscore: nh.setParam 会失败; 走 AyanamiROS 的 env fallback。
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  ayanami_ros::AyanamiROS node(nh);

  // 1) 扩展模块加载成功 (libayanami_rviz_viewer.so dlopen + create_extension_module)。
  ASSERT_EQ(node.extensionModules().size(), 1u);

  // 2) 核心导航宿主插件加载成功 (dlopen libayanami_costmap_navigation.so +
  // initialize + start)。
  ASSERT_NE(node.navigationPlugin(), nullptr);

  // 3) 加载顺序 : 扩展在 navigation->start 前注册。构造期
  // loadNavigationPlugin→start 会 emit on_costmap_visualization; 若顺序
  // 错误 (nav 先于 ext), rviz_viewer 订阅未注册, 计数恒 0。此处计数 > 0 即证明
  // 扩展构造先于 navigation start 完成。
  const int emits = rvizViewerCostmapPubCount();
  EXPECT_GT(emits, 0) <<
      "extension 订阅应在 navigation_plugin->start() 前注册";
  EXPECT_EQ(node.configPath(), config_dir.string());

  // 4) start 进 loop, stop join; 不挂, 无 segfault。
  EXPECT_NO_THROW({
    node.start();
    ros::WallDuration(0.15).sleep();
    node.stop();
  });
}

TEST(AyanamiROS, emptyExtensionList) {
  const auto config_dir = makeConfigDir();
  writeFile(config_dir / "config_ros.json",
             "{\n  \"ayanami_ros\": {\n"
             "    \"extension_modules\": []\n"
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

// latest-wins goal 邮箱 : ROS 回调侧 submitGoal 只覆写 pending_goal
// (不排队), loop 侧 drainPendingGoal 每次最多投递当前最新一条再置空。
TEST(AyanamiROS, goalMailboxIsLatestWinsAndDrainsAtMostLatest) {
  const auto config_dir = makeConfigDir();
  ros::NodeHandle nh("~");
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  ayanami_ros::AyanamiROS node(nh);
  ASSERT_NE(node.navigationPlugin(), nullptr);

  // 两次 submitGoal -> 邮箱只剩最新一条 (latest wins, 不排队)。
  node.submitGoal(makeAyaPose(1.0, 2.0));
  node.submitGoal(makeAyaPose(3.0, 4.0));
  EXPECT_TRUE(node.hasPendingGoal());

  // drain 一次只投递当前最新 (返回 true), 邮箱随即清空; 再 drain 无 goal。
  EXPECT_TRUE(node.drainPendingGoal());
  EXPECT_FALSE(node.hasPendingGoal());
  EXPECT_FALSE(node.drainPendingGoal());

  // 空提交逐个 drain 不产生额外投递。
  node.drainPendingGoal();
  EXPECT_FALSE(node.hasPendingGoal());

  node.stop();
}

// odom 速度直写全局 RobotStateController: 控制器 execution 读到真实最新速度
// (不再是硬编码零速度)。
TEST(AyanamiROS, odomVelocityWiredIntoRobotStateController) {
  const auto config_dir = makeConfigDir();
  ros::NodeHandle nh("~");
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  ayanami_ros::AyanamiROS node(nh);
  ASSERT_NE(node.navigationPlugin(), nullptr);
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.resetVelocity();
  // AyanamiROS 构造期间 navigation_plugin->initialize 已用 config frame
  // (map/odom/base_footprint) 初始化 singleton。
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

// 障碍观测输入不再走全局旁路槽: ROS 回调直递宿主 AbstractNavigation 的
// setPointcloud/setLaserScan (输入/输出分层), 空消息静默不崩。
TEST(AyanamiROS, obstacleInputWiredToHostPluginWithoutCrash) {
  const auto config_dir = makeConfigDir();
  ros::NodeHandle nh("~");
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  ayanami_ros::AyanamiROS node(nh);
  auto* plugin = node.navigationPlugin().get();
  ASSERT_NE(plugin, nullptr);

  // 空 pointcloud / laserscan -> 落图管线提前返回, 不崩。
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
