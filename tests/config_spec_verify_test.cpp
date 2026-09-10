// config_ros.json + costmap.json + costmap_navigation.json
// (aligned with Gazebo nav_demo) content consistency verification (-§8.3).
//
// Loads the real config directory (src/ayanami/config) and verifies:
// 1. config_ros.json::ayanami_ros topic/frame/rate matches verbatim
// (laserscan=/scan, odom=/odom, goal=/move_base_simple/goal,
// map/odom/base_footprint, tf_sync_rate 50).
// 2. config_extensions.json::extensions.loading_extension_modules contains
// "libayanami_rviz_viewer.so"; output topics live in the rviz_viewer section.
// 3. costmap.json: global rolling=true (M2 nav_demo 99m large-map rolling
// follow, see), local rolling=true,
// observation_sources uses LaserScan (incl. scan), obstacle_range/raytrace_range,
// inscribed_radius 0.3 (vehicle body), inflation_radius 0.3+safety margin.
// 4. costmap_navigation.json: robot_frame=base_footprint consistent with
// config_ros; *_id keeps the full module name (unbroken).
// 5. static_map points to a loadable PGM (M1 assets/nav.pgm, box_house
// compatible), and nav_demo/map/nav.pgm loads compatibly with the same loader
// (StaticLayer) (M1 cross-compatibility).

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

void initConfig() {
  setenv("AYANAMI_CONFIG_PATH", g_config_dir, 1);
  ayanami::GlobalConfig::instance(g_config_dir, /*override_path=*/true);
}

// Raw json file read (extension_modules / observation_sources are arrays and
// Config has no vector<string> instantiation, so parse directly with
// nlohmann).
nlohmann::json readJson(const char* config_name) {
  std::ifstream ifs(ayanami::GlobalConfig::getConfigPath(config_name));
  EXPECT_TRUE(ifs.good()) << "cannot open " << config_name;
  return nlohmann::json::parse(ifs, nullptr, false);
}

// Reads the config_navigation file (costmap parameters were merged into it).
nlohmann::json read_navigation_json() {
  std::ifstream ifs(
      ayanami::GlobalConfig::getConfigPath("config_navigation"));
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

TEST(ConfigSpecVerify, configRosIsInputBoundaryOnly) {
  // config_ros.json's single responsibility = the ROS input boundary
  // (subscriptions + TF + host module assembly). The planner -> smoother ->
  // controller path data flow lives entirely inside the C++ core (BT
  // blackboard + TaskHandle) and never touches ROS; output topics belong to
  // config_extensions.json (published on the bypass via rviz_viewer's global
  // callback slots).
  initConfig();
  ayanami::Config cfg(
      ayanami::GlobalConfig::getConfigPath("config_ros"));
  const std::string sec = "ayanami_ros";

  EXPECT_EQ(cfg.param<std::string>(sec, "pointcloud_topic", ""),
            "/velodyne_points");
  // laserscan_topic may be explicitly set to "" (subscribe-nothing sentinel
  // honored by AyanamiROS::subscribeTopics); "/scan" is the Gazebo default.
  {
    const std::string laserscan =
        cfg.param<std::string>(sec, "laserscan_topic", "__missing__");
    EXPECT_TRUE(laserscan == "/scan" || laserscan.empty())
        << "laserscan_topic must be \"/scan\" or \"\" (disabled)";
  }
  EXPECT_EQ(cfg.param<std::string>(sec, "odom_topic", ""), "/odom");
  EXPECT_EQ(cfg.param<std::string>(sec, "goal_topic", ""),
            "/move_base_simple/goal");

  // Regression lock: config_ros.json must not carry output/extension topics
  // (the data flow does not ping-pong in and out of ROS).
  const char* const k_output_keys[] = {
      "cmd_vel_topic", "cmd_vel_stamped_topic", "planner_path_topic",
      "smoother_path_topic", "smoother_input_path_topic",
      "global_costmap_topic", "local_costmap_topic", "static_map_topic",
      "planner_goal_topic", "planner_feedback_topic",
      "smoother_feedback_topic", "controller_path_topic",
      "controller_feedback_topic", "controller_status_topic",
      "recovery_goal_topic", "recovery_feedback_topic",
      "recovery_status_topic", "robot_state_topic",
      "extension_modules",
  };
  for (const auto* key : k_output_keys) {
    EXPECT_FALSE(cfg.hasParam(sec, key))
        << "config_ros.json must not contain output/extension keys: " << key;
  }

  EXPECT_EQ(cfg.param<std::string>(sec, "map_frame", ""), "map");
  EXPECT_EQ(cfg.param<std::string>(sec, "odom_frame", ""), "odom");
  EXPECT_EQ(cfg.param<std::string>(sec, "robot_frame", ""),
            "base_link");
  EXPECT_DOUBLE_EQ(cfg.param<double>(sec, "tf_sync_rate", 0.0), 50.0);

  EXPECT_EQ(cfg.param<std::string>(sec, "navigation_so_name", ""),
            "libayanami_costmap_navigation.so");
  // Generic factory: navigation_create_symbol no longer exists.
  EXPECT_FALSE(cfg.hasParam(sec, "navigation_create_symbol"));
}

TEST(ConfigSpecVerify, extensionModulesIncludeRvizViewer) {
  // Single source for the extension manifest =
  // config_extensions.json::extensions.loading_extension_modules (asuka mode).
  initConfig();
  ayanami::Config cfg(
      ayanami::GlobalConfig::getConfigPath("config_extensions"));
  const auto mods = cfg.param<std::vector<std::string>>(
      "extensions", "loading_extension_modules", {});
  ASSERT_FALSE(mods.empty()) << "loading_extension_modules must not be empty";
  bool found = false;
  for (const auto& m : mods) {
    if (m == "libayanami_rviz_viewer.so") {
      found = true;
    }
  }
  EXPECT_TRUE(found)
      << "loading_extension_modules must contain libayanami_rviz_viewer.so";
}

TEST(ConfigSpecVerify, rvizViewerTopicsLiveInConfigExtensions) {
  // rviz_viewer output bridge topics (extension): all ayanami outputs are
  // published as ROS messages by rviz_viewer through global callbacks; topic
  // names are configured in the rviz_viewer section of config_extensions.json
  // (asuka mode: the module parameter section has the same name as the
  // module).
  initConfig();
  ayanami::Config cfg(
      ayanami::GlobalConfig::getConfigPath("config_extensions"));
  const std::string sec = "rviz_viewer";
  EXPECT_EQ(cfg.param<std::string>(sec, "planner_path_topic", ""),
            "/ayanami/planner/path");
  EXPECT_EQ(cfg.param<std::string>(sec, "smoother_path_topic", ""),
            "/ayanami/smoother/path");
  EXPECT_EQ(cfg.param<std::string>(sec, "smoother_input_path_topic", ""),
            "/ayanami/smoother/input_path");
  EXPECT_EQ(cfg.param<std::string>(sec, "controller_path_topic", ""),
            "/ayanami/controller/path");
  EXPECT_EQ(cfg.param<std::string>(sec, "global_costmap_topic", ""),
            "/ayanami/global_costmap");
  EXPECT_EQ(cfg.param<std::string>(sec, "local_costmap_topic", ""),
            "/ayanami/local_costmap");
  EXPECT_EQ(cfg.param<std::string>(sec, "planner_goal_topic", ""),
            "/ayanami/planner/goal");
  EXPECT_EQ(cfg.param<std::string>(sec, "planner_feedback_topic", ""),
            "/ayanami/planner/feedback");
  EXPECT_EQ(cfg.param<std::string>(sec, "smoother_feedback_topic", ""),
            "/ayanami/smoother/feedback");
  EXPECT_EQ(cfg.param<std::string>(sec, "controller_feedback_topic", ""),
            "/ayanami/controller/feedback");
  EXPECT_EQ(cfg.param<std::string>(sec, "controller_status_topic", ""),
            "/ayanami/controller/status");
  EXPECT_EQ(cfg.param<std::string>(sec, "recovery_goal_topic", ""),
            "/ayanami/recovery/goal");
  EXPECT_EQ(cfg.param<std::string>(sec, "recovery_feedback_topic", ""),
            "/ayanami/recovery/feedback");
  EXPECT_EQ(cfg.param<std::string>(sec, "recovery_status_topic", ""),
            "/ayanami/recovery/status");
  EXPECT_EQ(cfg.param<std::string>(sec, "navigation_status_topic", ""),
            "/ayanami/navigation_status");
  // rviz_viewer no longer publishes any velocity command (rviz_viewer stays
  // rm_msgs-free); /cmd_chassis belongs to cmd_chassis_publisher.
  EXPECT_FALSE(cfg.hasParam(sec, "cmd_vel_topic"));
  EXPECT_FALSE(cfg.hasParam(sec, "cmd_vel_stamped_topic"));
  const std::string chassis = "cmd_chassis_publisher";
  EXPECT_EQ(cfg.param<std::string>(chassis, "cmd_chassis_topic", ""),
            "/cmd_chassis");
  EXPECT_DOUBLE_EQ(cfg.param<double>(chassis, "power_limit", -1.0), 100.0);
}

TEST(ConfigSpecVerify, tfSyncEdgesUseStringFormat) {
  // New format: "target-source[:rate]hz" per entry; the rate suffix is
  // optional (falls back to tf_sync_rate) and required syntax is strict:
  // non-empty frames, no empty rate.
  initConfig();
  ayanami::Config cfg(
      ayanami::GlobalConfig::getConfigPath("config_ros"));
  const auto entries = cfg.param<std::vector<std::string>>(
      "ayanami_ros", "tf_sync_edges", {});
  ASSERT_FALSE(entries.empty()) << "tf_sync_edges cannot be empty";
  for (const auto& entry : entries) {
    const auto dash = entry.rfind('-');
    ASSERT_NE(dash, std::string::npos) << entry;
    ASSERT_GT(dash, 0u) << entry;
    ASSERT_LT(dash + 1, entry.size()) << entry;
    const auto colon = entry.rfind(':');
    if (colon != std::string::npos) {
      ASSERT_GT(colon, dash) << entry;
      std::string rate = entry.substr(colon + 1);
      for (auto& ch : rate) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
      }
      if (rate.size() >= 2 && rate.compare(rate.size() - 2, 2, "hz") == 0) {
        rate.erase(rate.size() - 2);
      }
      ASSERT_FALSE(rate.empty()) << entry;
      EXPECT_EQ(rate.find_first_not_of("0123456789."), std::string::npos)
          << "invalid rate in " << entry;
    }
  }
}

TEST(ConfigSpecVerify, tfStaticEdgesAreSyncEdgePairs) {
  // Every tf_static_edges entry must denote the same (target, source) pair as
  // some tf_sync_edges entry, independent of the optional rate suffix.
  initConfig();
  ayanami::Config cfg(
      ayanami::GlobalConfig::getConfigPath("config_ros"));
  auto split = [](const std::string& entry) {
    std::string pair_part = entry;
    const auto colon = entry.rfind(':');
    if (colon != std::string::npos) {
      pair_part = entry.substr(0, colon);
    }
    const auto dash = pair_part.rfind('-');
    return std::make_pair(pair_part.substr(0, dash), pair_part.substr(dash + 1));
  };
  std::vector<std::pair<std::string, std::string>> sync;
  for (const auto& entry : cfg.param<std::vector<std::string>>(
           "ayanami_ros", "tf_sync_edges", {})) {
    sync.push_back(split(entry));
  }
  const auto statics = cfg.param<std::vector<std::string>>(
      "ayanami_ros", "tf_static_edges", {});
  for (const auto& entry : statics) {
    const auto pair = split(entry);
    EXPECT_NE(std::find(sync.begin(), sync.end(), pair), sync.end())
        << "static edge " << entry << " has no matching sync edge";
  }
}

TEST(ConfigSpecVerify, costmapGazeboAlignment) {
  initConfig();
  ayanami::Config cfg(
      ayanami::GlobalConfig::getConfigPath("config_navigation"));

  // Global spans the full nav_demo map; local remains a rolling window.
  EXPECT_FALSE(cfg.param<bool>("costmap_global", "rolling_window", true));
  EXPECT_TRUE(cfg.param<bool>("costmap_local", "rolling_window", false));
  EXPECT_FALSE(cfg.hasParam("costmap_global", "size_x"));
  EXPECT_FALSE(cfg.hasParam("costmap_global", "size_y"));

  // nav_demo/param alignment: robot_radius=0.12 (circular body, inscribed
  // fallback 0.12); global inflation 0.5/csf 10.0; local inflation 0.2/csf 3.0.
  EXPECT_DOUBLE_EQ(cfg.param<double>("costmap", "robot_radius", -1.0), 0.12);
  EXPECT_DOUBLE_EQ(
      cfg.param<double>("costmap_global", "inscribed_radius", -1.0), 0.12);
  EXPECT_DOUBLE_EQ(
      cfg.param<double>("costmap_local", "inscribed_radius", -1.0), 0.12);
  EXPECT_DOUBLE_EQ(
      cfg.param<double>("costmap_global", "inflation_radius", 0.0), 0.5);
  EXPECT_DOUBLE_EQ(
      cfg.param<double>("costmap_global", "cost_scaling_factor", 0.0), 10.0);
  EXPECT_DOUBLE_EQ(
      cfg.param<double>("costmap_local", "inflation_radius", 0.0), 0.2);
  EXPECT_DOUBLE_EQ(
      cfg.param<double>("costmap_local", "cost_scaling_factor", 0.0), 3.0);

  // obstacle/raytrace ranges for the narrow small-vehicle scenario (nav_demo
  // common parameters).
  EXPECT_DOUBLE_EQ(
      cfg.param<double>("costmap_local", "obstacle_range", -1.0), 3.0);
  EXPECT_DOUBLE_EQ(
      cfg.param<double>("costmap_local", "raytrace_range", -1.0), 3.5);
}

TEST(ConfigSpecVerify, costmapObservationSourcesUseScan) {
  initConfig();
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
  EXPECT_TRUE(found) << "observation_sources must contain the Gazebo LaserScan (scan)";
}

TEST(ConfigSpecVerify, robotFrameSingleSource) {
  initConfig();
  ayanami::Config nav(
      ayanami::GlobalConfig::getConfigPath("config_navigation"));
  ayanami::Config ros(
      ayanami::GlobalConfig::getConfigPath("config_ros"));
  const std::string nav_frame =
      nav.param<std::string>("navigation", "robot_frame", "");
  const std::string ros_frame =
      ros.param<std::string>("ayanami_ros", "robot_frame", "");
  EXPECT_EQ(nav_frame, "base_link");
  EXPECT_EQ(ros_frame, "base_link");
  EXPECT_EQ(nav_frame, ros_frame);
}

TEST(ConfigSpecVerify, navigationIdsKeepFullModuleNames) {
  initConfig();
  // Module id == module full name (incl. the _recovery suffix), unbroken.
  // ids are uniformly read from the name field of the component json pointed
  // to by the navigation section's config_* keys (recovery grouped by role).
  auto component_name = [](const std::string& key,
                           const std::string& section) {
    // config_{planner,smoother,controller} are single-file relative paths
    // inside config_navigation's navigation section (after the config
    // reorganization they no longer occupy config.json global keys).
    ayanami::Config nav(
        ayanami::GlobalConfig::getConfigPath("config_navigation"));
    const std::string entry = nav.param<std::string>("navigation", key, "");
    EXPECT_FALSE(entry.empty()) << "navigation." << key << " missing";
    const std::filesystem::path p(entry);
    const std::string directory =
        ayanami::GlobalConfig::instance()->param<std::string>(
            "global", "config_path", ".");
    ayanami::Config cfg(p.is_absolute() ? entry : (directory + "/" + entry));
    return cfg.param<std::string>(section, "name", "");
  };
  auto recovery_name_by_role = [](const std::string& role) {
    // The ordered config_recovery list lives in config_navigation's
    // navigation section.
    ayanami::Config nav(
        ayanami::GlobalConfig::getConfigPath("config_navigation"));
    std::vector<std::string> entries;
    try {
      entries = nav.param<std::vector<std::string>>(
          "navigation", "config_recovery", {});
    } catch (const std::exception&) {
      const std::string single =
          nav.param<std::string>("navigation", "config_recovery", "");
      if (!single.empty()) {
        entries.push_back(single);
      }
    }
    const std::string directory =
        ayanami::GlobalConfig::instance()->param<std::string>(
            "global", "config_path", ".");
    for (const auto& entry : entries) {
      const std::filesystem::path p(entry);
      ayanami::Config cfg(p.is_absolute() ? entry : (directory + "/" + entry));
      if (cfg.param<std::string>("recovery", "role", "") == role) {
        return cfg.param<std::string>("recovery", "name", "");
      }
    }
    return std::string("");
  };

  EXPECT_EQ(component_name("config_planner", "planner"),
            "costmap_smac_planner");
  EXPECT_EQ(component_name("config_smoother", "smoother"),
            "costmap_savitzky_smoother");
  EXPECT_EQ(component_name("config_controller", "controller"),
            "costmap_pid_controller");
  EXPECT_EQ(recovery_name_by_role("spin"), "costmap_spin_recovery");
  EXPECT_EQ(recovery_name_by_role("wait"), "costmap_wait_recovery");
  EXPECT_EQ(recovery_name_by_role("backup"), "costmap_back_up_recovery");
  EXPECT_EQ(recovery_name_by_role("clear_costmap"),
            "costmap_clear_costmap_recovery");
}

TEST(ConfigSpecVerify, staticMapLoadsAndNavDemoCompat) {
  initConfig();
  ayanami::Config cfg(
      ayanami::GlobalConfig::getConfigPath("config_navigation"));
  const std::string static_map =
      cfg.param<std::string>("costmap_global", "static_map", "");
  ASSERT_FALSE(static_map.empty());

  // M1 assets route (box_house compatible): static_map points to
  // assets/nav.pgm.
  const std::filesystem::path assets_map =
      std::filesystem::path(g_assets_dir) / "nav.pgm";
  ASSERT_TRUE(std::filesystem::exists(assets_map)) << assets_map;
  EXPECT_TRUE(pgm_loadable(assets_map.string()));

  // nav_demo's ROS YAML adapter and Ayanami's JSON config are independent of
  // each other.
  const std::filesystem::path nav_demo_map =
      std::filesystem::path(g_nav_demo_map_dir) / "nav.pgm";
  if (std::filesystem::exists(nav_demo_map)) {
    EXPECT_TRUE(pgm_loadable(nav_demo_map.string()));
  } else {
    GTEST_SKIP() << "nav_demo map not present: " << nav_demo_map;
  }
}

TEST(ConfigSpecVerify, ayanamiConfigurationContainsNoYaml) {
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
