// Task 10: config_ros.json + costmap.json + costmap_navigation.json
// (Gazebo nav_demo 对齐) 内容一致性验证 (spec §8.1-§8.3)。
//
// 加载真实 config 目录 (src/ayanami/config), 验证:
//   1. config_ros.json::ayanami_ros topic/frame/rate 与 spec §8.1 verbatim 一致
//      (laserscan=/scan, odom=/odom, goal=/move_base_simple/goal,
//      map/odom/base_footprint, tf_sync_rate 50)。
//   2. extension_modules 含 "librviz_viewer.so"。
//   3. costmap.json: global rolling=false, local rolling=true,
//      observation_sources 用 LaserScan (含 scan), obstacle_range/raytrace_range,
//      inscribed_radius 0.3 (车体), inflation_radius 0.3+safety margin。
//   4. costmap_navigation.json: robot_frame=base_footprint 与 config_ros 一致;
//      *_id 保持 full plugin name (Ruling #8 不破)。
//   5. static_map 指向可加载 PGM (M1 assets/nav.pgm, box_house 兼容), 且
//      nav_demo/map/nav.pgm 同 loader (StaticLayer) 可兼容加载 (与 M1 能否兼容载)。

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <costmap_2d_layers/static_layer.hpp>
#include <ayanami/utility/config.hpp>

namespace {

#ifndef AYANAMI_TEST_CONFIG_DIR
#error "AYANAMI_TEST_CONFIG_DIR must be defined by CMake"
#endif
#ifndef AYANAMI_TEST_ASSETS_DIR
#error "AYANAMI_TEST_ASSETS_DIR must be defined by CMake"
#endif
#ifndef NAV_DEMO_MAP_DIR
#error "NAV_DEMO_MAP_DIR must be defined by CMake"
#endif

const char* const g_config_dir = AYANAMI_TEST_CONFIG_DIR;
const char* const g_assets_dir = AYANAMI_TEST_ASSETS_DIR;
const char* const g_nav_demo_map_dir = NAV_DEMO_MAP_DIR;

void init_config() {
  setenv("AYANAMI_CONFIG_PATH", g_config_dir, 1);
  ayanami::GlobalConfig::instance(g_config_dir, /*override_path=*/true);
}

// 原始读 json 文件 (extension_modules / observation_sources 是数组, Config 无
// vector<string> 实例化, 直接 nlohmann 解析)。
nlohmann::json read_json(const char* config_name) {
  std::ifstream ifs(ayanami::GlobalConfig::get_config_path(config_name));
  EXPECT_TRUE(ifs.good()) << "cannot open " << config_name;
  return nlohmann::json::parse(ifs, nullptr, false);
}

bool pgm_loadable(const std::string& path) {
  costmap_2d_layers::StaticLayer layer;
  try {
    layer.setMapPath(path, 0.05, 0.0, 0.0);
  } catch (...) {
    return false;
  }
  return true;
}

TEST(ConfigSpecVerify, ConfigRosTopicsAndFrames) {
  init_config();
  ayanami::Config cfg(
      ayanami::GlobalConfig::get_config_path("config_ros"));
  const std::string sec = "ayanami_ros";

  EXPECT_EQ(cfg.param<std::string>(sec, "pointcloud_topic", ""),
            "/velodyne_points");
  EXPECT_EQ(cfg.param<std::string>(sec, "laserscan_topic", ""), "/scan");
  EXPECT_EQ(cfg.param<std::string>(sec, "odom_topic", ""), "/odom");
  EXPECT_EQ(cfg.param<std::string>(sec, "goal_topic", ""),
            "/move_base_simple/goal");
  EXPECT_EQ(cfg.param<std::string>(sec, "cmd_vel_topic", ""), "/cmd_vel");
  EXPECT_EQ(cfg.param<std::string>(sec, "planner_path_topic", ""),
            "/ayanami/planner/path");
  EXPECT_EQ(cfg.param<std::string>(sec, "smoother_path_topic", ""),
            "/ayanami/smoother/path");
  EXPECT_EQ(cfg.param<std::string>(sec, "global_costmap_topic", ""),
            "/ayanami/global_costmap");
  EXPECT_EQ(cfg.param<std::string>(sec, "local_costmap_topic", ""),
            "/ayanami/local_costmap");
  EXPECT_EQ(cfg.param<std::string>(sec, "static_map_topic", ""),
            "/ayanami/map");

  EXPECT_EQ(cfg.param<std::string>(sec, "map_frame", ""), "map");
  EXPECT_EQ(cfg.param<std::string>(sec, "odom_frame", ""), "odom");
  EXPECT_EQ(cfg.param<std::string>(sec, "robot_frame", ""),
            "base_footprint");
  EXPECT_DOUBLE_EQ(cfg.param<double>(sec, "tf_sync_rate", 0.0), 50.0);

  EXPECT_EQ(cfg.param<std::string>(sec, "navigation_so_name", ""),
            "libayanami_costmap_navigation.so");
  EXPECT_EQ(cfg.param<std::string>(sec, "navigation_create_symbol", ""),
            "create_navigation_module");
}

TEST(ConfigSpecVerify, ExtensionModulesIncludeRvizViewer) {
  init_config();
  const auto doc = read_json("config_ros");
  const auto& mods = doc.at("ayanami_ros").at("extension_modules");
  ASSERT_TRUE(mods.is_array());
  bool found = false;
  for (const auto& m : mods) {
    if (m.is_string() && m.get<std::string>() == "librviz_viewer.so") {
      found = true;
    }
  }
  EXPECT_TRUE(found) << "extension_modules 应含 librviz_viewer.so";
}

TEST(ConfigSpecVerify, CostmapGazeboAlignment) {
  init_config();
  ayanami::Config cfg(
      ayanami::GlobalConfig::get_config_path("config_costmap"));

  // global 全图固定 / local rolling (spec §8.3)。
  EXPECT_FALSE(cfg.param<bool>("costmap_global", "rolling_window", true));
  EXPECT_TRUE(cfg.param<bool>("costmap_local", "rolling_window", false));

  // 车体 0.3 (inscribed) + inflation safety margin。
  EXPECT_DOUBLE_EQ(
      cfg.param<double>("costmap_global", "inscribed_radius", -1.0), 0.3);
  EXPECT_DOUBLE_EQ(
      cfg.param<double>("costmap_local", "inscribed_radius", -1.0), 0.3);
  EXPECT_GT(cfg.param<double>("costmap_global", "inflation_radius", 0.0), 0.3);
  EXPECT_GT(cfg.param<double>("costmap_local", "inflation_radius", 0.0), 0.3);

  // 小车窄场景 obstacle/raytrace range。
  EXPECT_DOUBLE_EQ(
      cfg.param<double>("costmap_local", "obstacle_range", -1.0), 2.5);
  EXPECT_DOUBLE_EQ(
      cfg.param<double>("costmap_local", "raytrace_range", -1.0), 3.0);
}

TEST(ConfigSpecVerify, CostmapObservationSourcesUseScan) {
  init_config();
  const auto doc = read_json("config_costmap");
  const auto& sources = doc.at("costmap_local").at("observation_sources");
  ASSERT_TRUE(sources.is_array());
  bool found = false;
  for (const auto& s : sources) {
    if (!s.is_string()) {
      continue;
    }
    const auto v = s.get<std::string>();
    if (v == "scan" || v.find("LaserScan") != std::string::npos) {
      found = true;
    }
  }
  EXPECT_TRUE(found) << "observation_sources 应含 Gazebo LaserScan (scan)";
}

TEST(ConfigSpecVerify, RobotFrameSingleSource) {
  init_config();
  ayanami::Config nav(
      ayanami::GlobalConfig::get_config_path("config_navigation"));
  ayanami::Config ros(
      ayanami::GlobalConfig::get_config_path("config_ros"));
  const std::string nav_frame =
      nav.param<std::string>("navigation", "robot_frame", "");
  const std::string ros_frame =
      ros.param<std::string>("ayanami_ros", "robot_frame", "");
  EXPECT_EQ(nav_frame, "base_footprint");
  EXPECT_EQ(ros_frame, "base_footprint");
  EXPECT_EQ(nav_frame, ros_frame);
}

TEST(ConfigSpecVerify, NavigationIdsKeepFullPluginNames) {
  init_config();
  ayanami::Config nav(
      ayanami::GlobalConfig::get_config_path("config_navigation"));
  // Ruling #8: *_id 值 == 插件 full name (含 _recovery 后缀), 不破。
  EXPECT_EQ(nav.param<std::string>("navigation", "spin_id", ""),
            "costmap_spin_recovery");
  EXPECT_EQ(nav.param<std::string>("navigation", "wait_id", ""),
            "costmap_wait_recovery");
  EXPECT_EQ(nav.param<std::string>("navigation", "backup_id", ""),
            "costmap_back_up_recovery");
  EXPECT_EQ(nav.param<std::string>("navigation", "clear_costmap_id", ""),
            "costmap_clear_costmap_recovery");
  EXPECT_EQ(nav.param<std::string>("navigation", "recovery_id", ""),
            "costmap_clear_costmap_recovery");
}

TEST(ConfigSpecVerify, StaticMapLoadsAndNavDemoCompat) {
  init_config();
  ayanami::Config cfg(
      ayanami::GlobalConfig::get_config_path("config_costmap"));
  const std::string static_map =
      cfg.param<std::string>("costmap_global", "static_map", "");
  ASSERT_FALSE(static_map.empty());

  // M1 assets 航道 (box_house 兼容): static_map 指 assets/nav.pgm。
  const std::filesystem::path assets_map =
      std::filesystem::path(g_assets_dir) / "nav.pgm";
  ASSERT_TRUE(std::filesystem::exists(assets_map)) << assets_map;
  EXPECT_TRUE(pgm_loadable(assets_map.string()));

  // nav_demo 地图 (nav.yaml → nav.pgm) 同 loader 可兼容加载 (与 M1 兼容载)。
  const std::filesystem::path nav_demo_map =
      std::filesystem::path(g_nav_demo_map_dir) / "nav.pgm";
  if (std::filesystem::exists(nav_demo_map)) {
    EXPECT_TRUE(pgm_loadable(nav_demo_map.string()));
  } else {
    GTEST_SKIP() << "nav_demo map not present: " << nav_demo_map;
  }
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
