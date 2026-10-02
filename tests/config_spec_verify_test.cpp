// config_ros.json + routes/*.json + config_extensions.json
// (aligned with Gazebo nav_demo) content consistency verification (-§8.3).
//
// Loads the real config directory (src/ayanami/config) and verifies:
// 1. config_ros.json::ayanami_ros topic/frame matches verbatim
// (pointcloud=/nagisa/obstacle, odom=/asuka/odometry,
// goal=/move_base_simple/goal, map/odom/base_link). The host navigation
// module is NOT here: each route config declares its own navigation.so_name,
// so route switching stays a one-file edit (config.json::config_navigation).
// 2. config_extensions.json::extensions.loading_extension_modules contains
// "libayanami_rviz_viewer.so"; output topics live in the rviz_viewer section.
// 3. routes/costmap_navigation.json (pinned, NOT the active route): global
// static+inflation composition, local rolling obstacle+inflation,
// robot_radius 0.3 as the single body-size source.
// 4. Route ids keep the full module name (unbroken).
// 5. static_map points to a loadable PGM (M1 assets/nav.pgm, box_house
// compatible), and nav_demo/map/nav.pgm loads compatibly with the same loader
// (StaticLayer) (M1 cross-compatibility).

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <costmap/static_layer.hpp>
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

// Reads a route file DIRECTLY, independent of which route
// config.json::global.config_navigation currently activates: route-spec tests
// below pin their route so the suite stays green for either active route.
nlohmann::json read_route_json(const char* relative) {
  const std::filesystem::path path =
      std::filesystem::path(AYANAMI_TEST_CONFIG_DIR) / relative;
  std::ifstream ifs(path);
  EXPECT_TRUE(ifs.good()) << "cannot open " << path;
  return nlohmann::json::parse(ifs, nullptr, false);
}

std::string route_path(const char* relative) {
  return (std::filesystem::path(AYANAMI_TEST_CONFIG_DIR) / relative).string();
}

const char* const k_costmap_route = "routes/fsm_costmap_navigation.json";
const char* const k_rosemap_route = "routes/fsm_rose_navigation.json";
const char* const k_neupan_route = "routes/fsm_neupan_navigation.json";

bool pgm_loadable(const std::string& path) {
  costmap_2d::StaticLayer layer;
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
            "/nagisa/obstacle");
  // laserscan_topic may be explicitly set to "" (subscribe-nothing sentinel
  // honored by AyanamiROS::subscribeTopics); "/scan" is the Gazebo default.
  {
    const std::string laserscan =
        cfg.param<std::string>(sec, "laserscan_topic", "__missing__");
    EXPECT_TRUE(laserscan == "/scan" || laserscan.empty())
        << "laserscan_topic must be \"/scan\" or \"\" (disabled)";
  }
  EXPECT_EQ(cfg.param<std::string>(sec, "odom_topic", ""),
            "/asuka/odometry");
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
  // Host module selection moved to the route config
  // (routes/*.json::navigation.so_name): config_ros carrying a second copy
  // made route switching a two-file edit, and a one-file switch booted the
  // costmap host against a rosemap route (uncaught "costmap_global.layers is
  // required" -> SIGABRT / "died of signal 6"). Regression lock: the key must
  // not come back here.
  EXPECT_FALSE(cfg.hasParam(sec, "navigation_so_name"));
  // Generic factory: navigation_create_symbol no longer exists.
  EXPECT_FALSE(cfg.hasParam(sec, "navigation_create_symbol"));
}

TEST(ConfigSpecVerify, routeConfigsDeclareHostModule) {
  // Each route file must declare the host navigation module it boots
  // (navigation.so_name), next to the family resources it owns. This is the
  // structural regression lock for the signal-6 crash: AyanamiROS refuses to
  // start when the key is missing, and the declared family must match the
  // route's map sections.
  {
    const auto doc = read_route_json(k_costmap_route);
    ASSERT_TRUE(doc.at("navigation").contains("so_name"));
    EXPECT_EQ(doc.at("navigation").at("so_name").get<std::string>(),
              "libayanami_costmap_navigation_fsm.so");
    EXPECT_TRUE(doc.contains("costmap_global"));
    EXPECT_TRUE(doc.contains("costmap_local"));
  }
  {
    const auto doc = read_route_json(k_rosemap_route);
    ASSERT_TRUE(doc.at("navigation").contains("so_name"));
    EXPECT_EQ(doc.at("navigation").at("so_name").get<std::string>(),
              "libayanami_rose_navigation_fsm.so");
    EXPECT_TRUE(doc.contains("costmap_global"));
    EXPECT_TRUE(doc.contains("costmap_local"));
    EXPECT_TRUE(doc.contains("esdfmap"));
  }
  {
    // The neupan controller route: costmap host + the neupan controller
    // module (obstacle feed = the costmap presence mask).
    const auto doc = read_route_json(k_neupan_route);
    ASSERT_TRUE(doc.at("navigation").contains("so_name"));
    EXPECT_EQ(doc.at("navigation").at("so_name").get<std::string>(),
              "libayanami_costmap_navigation_fsm.so");
    EXPECT_EQ(doc.at("navigation").at("config_controller").get<std::string>(),
              "modules/costmap_neupan_controller.json");
    EXPECT_TRUE(doc.contains("costmap_global"));
    EXPECT_TRUE(doc.contains("costmap_local"));
  }
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
  // The smoother input path and the controller follow path are no longer
  // published by rviz_viewer (the smoother path result and the controller
  // status carry the visualization-relevant state).
  EXPECT_EQ(cfg.param<std::string>(sec, "smoother_input_path_topic", "__absent__"),
            "__absent__");
  EXPECT_EQ(cfg.param<std::string>(sec, "controller_path_topic", "__absent__"),
            "__absent__");
  // Costmap topics moved to the dedicated costmap_viewer section (rviz_viewer
  // no longer publishes any map).
  EXPECT_EQ(cfg.param<std::string>(sec, "global_costmap_topic", "__absent__"),
            "__absent__");
  EXPECT_EQ(cfg.param<std::string>(sec, "local_costmap_topic", "__absent__"),
            "__absent__");
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

  // costmap_viewer: dedicated per-layer costmap output; four slots
  // (global static/inflation, local obstacle/inflation), each cached
  // long-term and republished at ITS OWN rate (the global layers are
  // static -- 0.1 Hz default; the local layers follow the robot).
  const std::string cv_sec = "rose_viewer";
  EXPECT_EQ(cfg.param<std::string>(cv_sec, "global_static_topic", ""),
            "/ayanami/global_costmap/static");
  EXPECT_EQ(cfg.param<std::string>(cv_sec, "global_inflation_topic", ""),
            "/ayanami/global_costmap/inflation");
  EXPECT_EQ(cfg.param<std::string>(cv_sec, "local_obstacle_topic", ""),
            "/ayanami/local_costmap/obstacle");
  EXPECT_EQ(cfg.param<std::string>(cv_sec, "local_inflation_topic", ""),
            "/ayanami/local_costmap/inflation");
  EXPECT_DOUBLE_EQ(cfg.param<double>(cv_sec, "global_static_rate", 0.0), 0.1);
  EXPECT_DOUBLE_EQ(cfg.param<double>(cv_sec, "global_inflation_rate", 0.0), 0.1);
  EXPECT_DOUBLE_EQ(cfg.param<double>(cv_sec, "local_obstacle_rate", 0.0), 5.0);
  EXPECT_DOUBLE_EQ(cfg.param<double>(cv_sec, "local_inflation_rate", 0.0), 5.0);

  // rosemap_viewer is gone with the rosemap family (the FSM era has no
  // field-output slot); the section must not come back.
  EXPECT_FALSE(cfg.hasSection("rosemap_viewer"));
  // rviz_viewer no longer publishes any velocity command (rviz_viewer stays
  // rm_msgs-free); /cmd_chassis belongs to cmd_chassis_publisher.
  EXPECT_FALSE(cfg.hasParam(sec, "cmd_vel_topic"));
  EXPECT_FALSE(cfg.hasParam(sec, "cmd_vel_stamped_topic"));
  const std::string chassis = "cmd_chassis_publisher";
  EXPECT_EQ(cfg.param<std::string>(chassis, "cmd_chassis_topic", ""),
            "/cmd_chassis");
  EXPECT_DOUBLE_EQ(cfg.param<double>(chassis, "power_limit", -1.0), 100.0);
}

TEST(ConfigSpecVerify, configRosCarriesNoTfSamplingConfig) {
  // TF moved to an on-demand bridge (the core queries the injected tf2
  // source directly): the periodic-sampling configuration keys are gone.
  initConfig();
  ayanami::Config cfg(
      ayanami::GlobalConfig::getConfigPath("config_ros"));
  EXPECT_FALSE(cfg.hasParam("ayanami_ros", "tf_sync_edges"));
  EXPECT_FALSE(cfg.hasParam("ayanami_ros", "tf_static_edges"));
  EXPECT_FALSE(cfg.hasParam("ayanami_ros", "tf_sync_rate"));
  EXPECT_FALSE(cfg.hasParam("ayanami_ros", "tf_cache_duration"));
  EXPECT_EQ(ayanami::GlobalConfig::getConfigPath("config_ros"),
            std::string(AYANAMI_TEST_CONFIG_DIR) + "/config_ros.json");
}

nlohmann::json read_module_json(const std::string& rel) {
  std::filesystem::path path =
      std::filesystem::path(AYANAMI_TEST_CONFIG_DIR) / rel;
  std::ifstream ifs(path);
  EXPECT_TRUE(ifs.good()) << "cannot open " << path;
  return nlohmann::json::parse(ifs, nullptr, false);
}

TEST(ConfigSpecVerify, costmapGazeboAlignment) {
  initConfig();
  // Pinned to the costmap route file: this test documents the costmap stack's
  // Gazebo nav_demo alignment, not whichever route is currently active.
  ayanami::Config cfg(route_path(k_costmap_route));

  // Global spans the full nav_demo map; local remains a rolling window.
  EXPECT_FALSE(cfg.param<bool>("costmap_global", "rolling_window", true));
  EXPECT_TRUE(cfg.param<bool>("costmap_local", "rolling_window", false));
  EXPECT_FALSE(cfg.hasParam("costmap_global", "size_x"));
  EXPECT_FALSE(cfg.hasParam("costmap_global", "size_y"));

  // robot_radius=0.3 (config_sentry alignment) is the SINGLE body-size
  // source: the per-map inscribed_radius keys are gone (the footprint
  // derives from the radius).
  EXPECT_DOUBLE_EQ(cfg.param<double>("costmap", "robot_radius", -1.0), 0.3);
  EXPECT_FALSE(cfg.hasParam("costmap_global", "inscribed_radius"));
  EXPECT_FALSE(cfg.hasParam("costmap_local", "inscribed_radius"));

  // Layer composition is INLINE (config_sentry-consolidated): the layers
  // list carries bare layer names, each layer's so_name + parameters live in
  // the same map section of this file.
  // NOTE: hold the parsed doc in a named variable -- binding a reference
  // into a temporary would dangle (lifetime extension does not chain).
  const auto nav_doc = read_route_json(k_costmap_route);
  // Static-global policy: the global grid carries static+inflation ONLY --
  // fresh dynamic obstacle marks live exclusively in the local rolling map,
  // and the global map changes only through configure/clear commands (no
  // periodic update_frequency needed).
  ASSERT_TRUE(nav_doc.at("costmap_global").contains("static_layer"));
  ASSERT_TRUE(nav_doc.at("costmap_global").contains("inflation_layer"));
  EXPECT_FALSE(nav_doc.at("costmap_global").contains("obstacle_layer"));
  EXPECT_FALSE(nav_doc.at("costmap_global").contains("update_frequency"));
  EXPECT_FALSE(nav_doc.at("costmap_global").contains("layers"));
  ASSERT_TRUE(nav_doc.at("costmap_local").contains("obstacle_layer"));
  ASSERT_TRUE(nav_doc.at("costmap_local").contains("inflation_layer"));
  EXPECT_FALSE(nav_doc.at("costmap_local").contains("layers"));

  // Inflation policy lives in the inline layer sections (global 0.5/csf 3.0;
  // local 0.55/csf 3.0 -- deliberately larger than nav_demo's 0.2 baseline:
  // robot_radius 0.3 + safety margin, same order as the global 0.5).
  const auto& global_inflation =
      nav_doc.at("costmap_global").at("inflation_layer");
  EXPECT_DOUBLE_EQ(global_inflation.at("inflation_radius").get<double>(), 0.5);
  EXPECT_DOUBLE_EQ(global_inflation.at("cost_scaling_factor").get<double>(),
                   3.0);
  const auto& local_inflation =
      nav_doc.at("costmap_local").at("inflation_layer");
  EXPECT_DOUBLE_EQ(local_inflation.at("inflation_radius").get<double>(), 0.55);
  EXPECT_DOUBLE_EQ(local_inflation.at("cost_scaling_factor").get<double>(),
                   3.0);

  // Observation filtering lives in the local obstacle layer section (nav_demo
  // scenario ranges; source = the nagisa obstacle point cloud, published in
  // the sensor frame so ranges are sensor-relative; obstacle_max_range
  // matches the 6x6 m rolling window and raytrace_max_range).
  const auto& local_obstacle = nav_doc.at("costmap_local").at("obstacle_layer");
  EXPECT_DOUBLE_EQ(
      local_obstacle.at("nagisa_obstacle").at("obstacle_max_range").get<double>(),
      3.0);
  EXPECT_DOUBLE_EQ(
      local_obstacle.at("nagisa_obstacle").at("raytrace_max_range").get<double>(),
      3.5);
  // Marks retire via decay (occlusion tolerance), not wholesale window
  // wipes.
  EXPECT_DOUBLE_EQ(local_obstacle.at("decay_time").get<double>(), 1.0);
}

TEST(ConfigSpecVerify, costmapObservationSourcesUseNagisaPointcloud) {
  // The observation_sources declaration lives in the consuming obstacle
  // layer's inline section (filtering parameters are layer-owned); the input
  // is the nagisa obstacle point cloud. Pinned to the costmap route.
  const auto nav_doc = read_route_json(k_costmap_route);
  const auto& sources = nav_doc.at("costmap_local")
                             .at("obstacle_layer")
                             .at("observation_sources");
  ASSERT_TRUE(sources.is_array());
  bool found = false;
  for (const auto& s : sources) {
    if (!s.is_string()) {
      continue;
    }
    if (s.get<std::string>() == "nagisa_obstacle") {
      found = true;
    }
  }
  EXPECT_TRUE(found) << "observation_sources must contain the nagisa "
                       "obstacle point cloud (nagisa_obstacle)";
  EXPECT_EQ(nav_doc.at("costmap_local")
                .at("obstacle_layer")
                .at("nagisa_obstacle")
                .at("type")
                .get<std::string>(),
            "pointcloud");
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
  // Pinned to the costmap route file (route-switch independent).
  auto component_name = [](const std::string& key,
                           const std::string& section) {
    // config_{planner,smoother,controller} are single-file relative paths
    // inside the route file's navigation section (after the config
    // reorganization they no longer occupy config.json global keys).
    ayanami::Config nav(route_path(k_costmap_route));
    const std::string entry = nav.param<std::string>("navigation", key, "");
    EXPECT_FALSE(entry.empty()) << "navigation." << key << " missing";
    const std::filesystem::path p(entry);
    const std::string directory =
        ayanami::GlobalConfig::instance()->param<std::string>(
            "global", "config_path", ".");
    ayanami::Config cfg(p.is_absolute() ? entry : (directory + "/" + entry));
    return cfg.param<std::string>(section, "name", "");
  };
  // The recovery list is an ordered plain list; the test phase ships the
  // placeholder only (it runs the full start/tick/finish protocol and takes
  // no action).
  ayanami::Config nav(route_path(k_costmap_route));
  const auto recovery_entries = nav.param<std::vector<std::string>>(
      "navigation", "config_recovery", {});
  ASSERT_FALSE(recovery_entries.empty());
  const std::string directory =
      ayanami::GlobalConfig::instance()->param<std::string>(
          "global", "config_path", ".");
  const std::filesystem::path rp(recovery_entries.front());
  ayanami::Config recovery_cfg(rp.is_absolute() ? recovery_entries.front()
                                                : (directory + "/" +
                                                   recovery_entries.front()));
  EXPECT_EQ(recovery_cfg.param<std::string>("recovery", "name", ""),
            "abstract_nothing_recovery");

  EXPECT_EQ(component_name("config_planner", "planner"),
            "costmap_astar_planner");
  EXPECT_EQ(component_name("config_smoother", "smoother"),
            "costmap_minsnap_smoother");
  EXPECT_EQ(component_name("config_controller", "controller"),
            "costmap_pid_controller");
}

TEST(ConfigSpecVerify, staticMapLoadsAndNavDemoCompat) {
  initConfig();
  // Pinned to the costmap route (the static map is a costmap-family asset).
  const auto nav_doc = read_route_json(k_costmap_route);
  const std::string static_map =
      nav_doc.at("costmap_global")
          .at("static_layer")
          .at("static_map")
          .get<std::string>();
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
