// Task 6: AyanamiROS 构造 + 加载链 TDD 测试。
//
// 无 roscore 环境 (headless)。用临时 config 目录 (全套 M1 配置 + config_ros.json)
// + T5 stub extension .so (代替 rviz_viewer), 验证:
//   1. AyanamiROS 构造按 spec §4.2 顺序完成: GlobalConfig → loadExtensionModules
//      → loadNavigationPlugin (dlopen libayanami_costmap_navigation.so +
//      initialize + start) → subscribeTopics。
//   2. 扩展先于 navigation->start() 注册 (M2-P1): stub 构造期注册的
//      on_costmap_visualization 订阅必须收到 start() 的首波可视化 emit
//      (通过 stub .so 导出的 extern "C" 计数查询)。
//   3. extension_modules_ / navigation_plugin_ 成员就位; start()/stop() 无崩。

#include <dlfcn.h>

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami_ros/ayanami_ros.hpp>

namespace {

#ifndef STUB_EXTENSION_SO
#error "STUB_EXTENSION_SO must be defined by CMake"
#endif

const char* const g_stub_so = STUB_EXTENSION_SO;

void write_file(const std::filesystem::path& path,
                const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream ofs(path);
  ASSERT_TRUE(ofs.good()) << "cannot write " << path;
  ofs << content;
}

std::string plugin_json(const std::string& module, const std::string& name,
                        const std::string& so_name,
                        const std::string& symbol) {
  return "\"" + module + "\": {\"name\": \"" + name +
         "\", \"so_name\": \"" + so_name + "\", \"create_symbol\": \"" +
         symbol + "\"}";
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

// 全套 M1 配置 (对标 spec §9, headless_e2e 结构) + config_ros.json。
// 插件 so 用裸名: catkin 测试环境 LD_LIBRARY_PATH 已含 devel/lib。
std::filesystem::path make_config_dir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_ros_load_test";
  std::filesystem::remove_all(dir);

  write_file(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"costmap_navigation.json\",\n"
             "    \"config_costmap\": \"costmap.json\",\n"
             "    \"config_planner\": \"costmap_smac_planner.json\",\n"
             "    \"config_smoother\": \"costmap_savitzky_smoother.json\",\n"
             "    \"config_controller\": \"costmap_pid_controller.json\",\n"
             "    \"config_recovery\": [\"costmap_clear_costmap_recovery.json\"]\n"
             "  }\n}\n");

  write_file(dir / "costmap_navigation.json",
             "{\n  \"navigation\": {\n"
             "    \"behavior_tree_xml\": \"BT/navigation.xml\",\n"
             "    \"behavior_tree_id\": \"NavigateWithReplanningAndSmoothing\",\n"
             "    \"planner_id\": \"costmap_smac\",\n"
             "    \"smoother_id\": \"costmap_savitzky\",\n"
             "    \"controller_id\": \"costmap_pid\",\n"
             "    \"recovery_id\": \"clear_recovery\",\n"
             "    \"clear_costmap_id\": \"clear_recovery\",\n"
             "    \"spin_id\": \"clear_recovery\",\n"
             "    \"wait_id\": \"clear_recovery\",\n"
             "    \"backup_id\": \"clear_recovery\",\n"
             "    \"planner_frequency\": 100.0,\n"
             "    \"smoother_frequency\": -1.0,\n"
             "    \"controller_frequency\": 20.0,\n"
             "    \"tick_frequency\": 100.0,\n"
             "    \"robot_frame\": \"base_footprint\",\n"
             "    \"costmap_visualization_rate\": 100.0\n"
             "  }\n}\n");

  write_file(dir / "costmap_smac_planner.json",
             "{\n  " +
                 plugin_json("planner", "costmap_smac",
                             "libayanami_costmap_smac_planner.so",
                             "create_costmap_planner_module") +
                 ",\n  \"costmap_smac_planner\": {\n"
                 "    \"use_diagonal\": true,\n    \"tolerance\": 0.0,\n"
                 "    \"cost_factor\": 0.5,\n    \"lethal_cost\": 254,\n"
                 "    \"neutral_cost\": 1.0,\n    \"use_final_approach\": false\n"
                 "  }\n}\n");
  write_file(dir / "costmap_savitzky_smoother.json",
             "{\n  " +
                 plugin_json("smoother", "costmap_savitzky",
                             "libayanami_costmap_savitzky_smoother.so",
                             "create_costmap_smoother_module") +
                 ",\n  \"costmap_savitzky_smoother\": {\n"
                 "    \"do_refinement\": true,\n    \"refinement_num\": 2\n"
                 "  }\n}\n");
  write_file(dir / "costmap_pid_controller.json",
             "{\n  " +
                 plugin_json("controller", "costmap_pid",
                             "libayanami_costmap_pid_controller.so",
                             "create_controller_module") +
                 ",\n  \"costmap_pid_controller\": {\n"
                 "    \"max_v\": 2.0,\n    \"max_w\": 1.0,\n"
                 "    \"goal_tolerance\": 0.1,\n    \"mode\": \"diff\"\n"
                 "  }\n}\n");
  write_file(dir / "costmap_clear_costmap_recovery.json",
             "{\n  \"recovery\": {\n"
             "    \"name\": \"clear_recovery\",\n"
             "    \"so_name\": \"libayanami_costmap_clear_costmap_recovery.so\",\n"
             "    \"create_symbol\": \"create_costmap_recovery_module\",\n"
             "    \"clear_radius\": 1.0,\n    \"affected_maps\": \"both\"\n"
             "  }\n}\n");

  write_file(dir / "costmap.json",
             "{\n  \"costmap\": {\n"
             "    \"global_frame\": \"map\",\n"
             "    \"local_frame\": \"odom\",\n"
             "    \"resolution\": 0.5\n"
             "  },\n"
             "  \"costmap_global\": {\n"
             "    \"size_x\": 20,\n    \"size_y\": 20,\n"
             "    \"rolling_window\": false,\n"
             "    \"track_unknown\": false,\n"
             "    \"static_map\": \"" +
                 (dir / "static_map.pgm").string() +
                 "\",\n"
                 "    \"inflation_radius\": 1.0,\n"
                 "    \"inscribed_radius\": 0.8,\n"
                 "    \"cost_scaling_factor\": 2.0\n"
                 "  },\n"
                 "  \"costmap_local\": {\n"
                 "    \"size_x\": 20,\n    \"size_y\": 20,\n"
                 "    \"rolling_window\": true,\n"
                 "    \"track_unknown\": false,\n"
                 "    \"inflation_radius\": 1.0,\n"
                 "    \"inscribed_radius\": 0.8,\n"
                 "    \"cost_scaling_factor\": 2.0\n"
                 "  }\n}\n");

  write_file(dir / "static_map.pgm", free_pgm(20, 20, 254));

  write_file(dir / "BT" / "navigation.xml",
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

  write_file(dir / "config_ros.json",
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
                 std::string(g_stub_so) +
                 "\"]\n"
                 "  }\n}\n");

  return dir;
}

// 从 stub .so 读可视化 emit 计数 (dlopen 已由 AyanamiROS 以 RTLD_GLOBAL 完成)。
int stub_viz_emit_count() {
  using Getter = int (*)();
  auto getter = reinterpret_cast<Getter>(dlsym(RTLD_DEFAULT,
                                               "stub_extension_viz_emit_count"));
  if (getter == nullptr) {
    return -1;
  }
  return getter();
}

TEST(AyanamiROS, InitLoadExtensionsBeforeNavigationPlugin) {
  const auto config_dir = make_config_dir();
  ros::NodeHandle nh("~");
  // 无 roscore: nh.setParam 会失败; 走 AyanamiROS 的 env fallback。
  setenv("AYANAMI_CONFIG_PATH", config_dir.string().c_str(), 1);

  ayanami_ros::AyanamiROS node(nh);

  // 1) 扩展模块加载成功 (stub .so dlopen + create_extension_module)。
  ASSERT_EQ(node.extensionModules().size(), 1u);

  // 2) 核心导航宿主插件加载成功 (dlopen libayanami_costmap_navigation.so +
  //    initialize + start)。
  ASSERT_NE(node.navigationPlugin(), nullptr);

  // 3) 加载顺序 (M2-P1): 扩展在 navigation->start() 前注册。构造期
  //    loadNavigationPlugin→start() 会 emit on_costmap_visualization; 若顺序
  //    错误 (nav 先于 ext), stub 订阅未注册, 计数恒 0。此处计数 > 0 即证明
  //    扩展构造先于 navigation start 完成。
  const int emits = stub_viz_emit_count();
  EXPECT_GT(emits, 0) <<
      "extension 订阅应在 navigation_plugin->start() 前注册 (M2-P1)";
  EXPECT_EQ(node.configPath(), config_dir.string());

  // 4) start() 进 loop, stop() join; 不挂, 无 segfault。
  EXPECT_NO_THROW({
    node.start();
    ros::WallDuration(0.15).sleep();
    node.stop();
  });
}

TEST(AyanamiROS, EmptyExtensionList) {
  const auto config_dir = make_config_dir();
  write_file(config_dir / "config_ros.json",
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

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ayanami_ros_load_test",
            ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
  ros::master::setRetryTimeout(ros::WallDuration(0.3));
  return RUN_ALL_TESTS();
}
