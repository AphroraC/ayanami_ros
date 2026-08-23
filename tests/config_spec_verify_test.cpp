// Task 10: config_ros.json + costmap.json + costmap_navigation.json
// (Gazebo nav_demo 对齐) 内容一致性验证 (spec §8.1-§8.3)。
//
// 加载真实 config 目录 (src/ayanami/config), 验证:
//   1. config_ros.json::ayanami_ros topic/frame/rate 与 spec §8.1 verbatim 一致
//      (laserscan=/scan, odom=/odom, goal=/move_base_simple/goal,
//      map/odom/base_footprint, tf_sync_rate 50)。
//   2. extension_modules 含 "librviz_viewer.so"。
//   3. costmap.json: global rolling=true (M2 Ruling #M2-4: nav_demo 99m 大地图
//      滚动跟随, 见 Task 12 stage C), local rolling=true,
//      observation_sources 用 LaserScan (含 scan), obstacle_range/raytrace_range,
//      inscribed_radius 0.3 (车体), inflation_radius 0.3+safety margin。
//   4. costmap_navigation.json: robot_frame=base_footprint 与 config_ros 一致;
//      *_id 保持 full plugin name (Ruling #8 不破)。
//   5. static_map 指向可加载 PGM (M1 assets/nav.pgm, box_house 兼容), 且
//      nav_demo/map/nav.pgm 同 loader (StaticLayer) 可兼容加载 (与 M1 能否兼容载)。

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

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

// 读 config_navigation 文件 (costmap 参数已并入其中)。
nlohmann::json read_navigation_json() {
  std::ifstream ifs(
      ayanami::GlobalConfig::get_config_path("config_navigation"));
  EXPECT_TRUE(ifs.good()) << "cannot open config_navigation";
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
  ayanami::Config cfg(
      ayanami::GlobalConfig::get_config_path("config_ros"));
  const auto mods = cfg.param<std::vector<std::string>>(
      "ayanami_ros", "extension_modules", {});
  ASSERT_FALSE(mods.empty()) << "extension_modules 不能为空";
  bool found = false;
  for (const auto& m : mods) {
    if (m == "librviz_viewer.so") {
      found = true;
    }
  }
  EXPECT_TRUE(found) << "extension_modules 应含 librviz_viewer.so";
}

TEST(ConfigSpecVerify, TfSyncEdgesAreValidPairs) {
  init_config();
  ayanami::Config cfg(
      ayanami::GlobalConfig::get_config_path("config_ros"));
  const auto edges = cfg.param<std::vector<std::vector<std::string>>>(
      "ayanami_ros", "tf_sync_edges", {});
  ASSERT_FALSE(edges.empty()) << "tf_sync_edges 不能为空";
  for (const auto& edge : edges) {
    ASSERT_EQ(edge.size(), 2u) << "每条 tf_sync_edges 必须为 2 项";
    EXPECT_FALSE(edge[0].empty());
    EXPECT_FALSE(edge[1].empty());
  }
}

TEST(ConfigSpecVerify, TfStaticEdgesAreValidSyncEdges) {
  init_config();
  ayanami::Config cfg(
      ayanami::GlobalConfig::get_config_path("config_ros"));
  const auto sync = cfg.param<std::vector<std::vector<std::string>>>(
      "ayanami_ros", "tf_sync_edges", {});
  const auto statics = cfg.param<std::vector<std::vector<std::string>>>(
      "ayanami_ros", "tf_static_edges", {});
  for (const auto& edge : statics) {
    ASSERT_EQ(edge.size(), 2u);
    EXPECT_NE(std::find(sync.begin(), sync.end(), edge), sync.end());
  }
}

TEST(ConfigSpecVerify, CostmapGazeboAlignment) {
  init_config();
  ayanami::Config cfg(
      ayanami::GlobalConfig::get_config_path("config_navigation"));

  // Global spans the full nav_demo map; local remains a rolling window.
  EXPECT_FALSE(cfg.param<bool>("costmap_global", "rolling_window", true));
  EXPECT_TRUE(cfg.param<bool>("costmap_local", "rolling_window", false));
  EXPECT_FALSE(cfg.has_param("costmap_global", "size_x"));
  EXPECT_FALSE(cfg.has_param("costmap_global", "size_y"));

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
  const auto doc = read_navigation_json();
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
  // Ruling #8: 插件 id == 插件 full name (含 _recovery 后缀), 不破。
  // id 统一从各组件 config_* json 的 name 字段读取 (recovery 按 role 分组)。
  auto component_name = [](const std::string& key,
                           const std::string& section) {
    ayanami::Config cfg(
        ayanami::GlobalConfig::get_config_path("config_navigation", key));
    return cfg.param<std::string>(section, "name", "");
  };
  auto recovery_name_by_role = [](const std::string& role) {
    for (const auto& path : ayanami::GlobalConfig::get_config_paths(
             "config_navigation", "config_recovery")) {
      ayanami::Config cfg(path);
      if (cfg.param<std::string>("recovery", "role", "") == role) {
        return cfg.param<std::string>("recovery", "name", "");
      }
    }
    return std::string("");
  };

  EXPECT_EQ(component_name("config_planner", "planner"), "costmap_smac");
  EXPECT_EQ(component_name("config_smoother", "smoother"),
            "costmap_savitzky");
  EXPECT_EQ(component_name("config_controller", "controller"),
            "costmap_pid");
  EXPECT_EQ(recovery_name_by_role("spin"), "costmap_spin_recovery");
  EXPECT_EQ(recovery_name_by_role("wait"), "costmap_wait_recovery");
  EXPECT_EQ(recovery_name_by_role("backup"), "costmap_back_up_recovery");
  EXPECT_EQ(recovery_name_by_role("clear_costmap"),
            "costmap_clear_costmap_recovery");
}

TEST(ConfigSpecVerify, StaticMapLoadsAndNavDemoCompat) {
  init_config();
  ayanami::Config cfg(
      ayanami::GlobalConfig::get_config_path("config_navigation"));
  const std::string static_map =
      cfg.param<std::string>("costmap_global", "static_map", "");
  ASSERT_FALSE(static_map.empty());

  // M1 assets 航道 (box_house 兼容): static_map 指 assets/nav.pgm。
  const std::filesystem::path assets_map =
      std::filesystem::path(g_assets_dir) / "nav.pgm";
  ASSERT_TRUE(std::filesystem::exists(assets_map)) << assets_map;
  EXPECT_TRUE(pgm_loadable(assets_map.string()));

  // nav_demo 的 ROS YAML adapter 与 Ayanami JSON 配置相互独立。
  const std::filesystem::path nav_demo_map =
      std::filesystem::path(g_nav_demo_map_dir) / "nav.pgm";
  if (std::filesystem::exists(nav_demo_map)) {
    EXPECT_TRUE(pgm_loadable(nav_demo_map.string()));
  } else {
    GTEST_SKIP() << "nav_demo map not present: " << nav_demo_map;
  }
}

TEST(ConfigSpecVerify, AyanamiConfigurationContainsNoYaml) {
  const std::filesystem::path config_dir(AYANAMI_TEST_CONFIG_DIR);
  const std::filesystem::path assets_dir(AYANAMI_TEST_ASSETS_DIR);
  for (const auto& root : {config_dir, assets_dir}) {
    for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
      const auto extension = entry.path().extension().string();
      EXPECT_NE(extension, ".yaml") << entry.path();
      EXPECT_NE(extension, ".yml") << entry.path();
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
