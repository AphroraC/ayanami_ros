// costmap_viewer ROS extension .so (dlopen -> create_extension_module ->
// configureParameters): subscribes the core's per-layer costmap artifact
// slots, holds the artifacts long-term, and republishes them from its own
// thread at the configured rates.
//
// dlopen libayanami_costmap_viewer.so to verify:
// 1. slot emissions land in the cache and are published on the latched topics;
// 2. the cache outlives the emission: with no further core emissions the
//    publisher thread keeps republishing at the configured rate.

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <dlfcn.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

namespace {

#ifndef COSTMAP_VIEWER_SO
#error "COSTMAP_VIEWER_SO must be defined by CMake"
#endif

const char* const g_costmap_viewer_so = COSTMAP_VIEWER_SO;

void writeFile(const std::filesystem::path& path, const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream ofs(path);
  ASSERT_TRUE(ofs.good()) << "cannot write " << path;
  ofs << content;
}

/// @brief Fast rates for deterministic testing (the production default for
/// the global artifacts is 0.1 Hz -- far too slow for a unit test).
std::filesystem::path makeConfigDir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_ros_costmap_viewer_test";
  std::filesystem::remove_all(dir);
  writeFile(dir / "config.json",
            "{\n  \"global\": {\n"
            "    \"config_extensions\": \"config_extensions.json\"\n"
            "  }\n}\n");
  writeFile(dir / "config_extensions.json",
            "{\n  \"extensions\": {\n"
            "    \"loading_extension_modules\": []\n"
            "  },\n"
            "  \"costmap_viewer\": {\n"
            "    \"global_static_topic\": \"/test/cv/global_static\",\n"
            "    \"global_inflation_topic\": \"/test/cv/global_inflation\",\n"
            "    \"local_obstacle_topic\": \"/test/cv/local_obstacle\",\n"
            "    \"local_inflation_topic\": \"/test/cv/local_inflation\",\n"
            "    \"global_static_rate\": 30.0,\n"
            "    \"global_inflation_rate\": 30.0,\n"
            "    \"local_obstacle_rate\": 30.0,\n"
            "    \"local_inflation_rate\": 30.0\n"
            "  }\n}\n");
  return dir;
}

void initGlobalConfig(const std::filesystem::path& dir) {
  setenv("AYANAMI_CONFIG_PATH", dir.string().c_str(), 1);
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);
}

int pubCount(const char* symbol) {
  using Getter = int (*)();
  auto getter = reinterpret_cast<Getter>(dlsym(RTLD_DEFAULT, symbol));
  if (getter == nullptr) {
    return -1;
  }
  return getter();
}

ayanami::nav_msgs::OccupancyGrid makeGrid() {
  ayanami::nav_msgs::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.5;
  grid.info.width = 4;
  grid.info.height = 4;
  grid.data.assign(16, 0);
  return grid;
}

/// @brief Load the module the way the production loader does (context
/// injected after a successful load).
ayanami::ExtensionModule::Ptr loadViewer() {
  ayanami::Config cfg((makeConfigDir() / "config_extensions.json").string());
  return ayanami::ExtensionModule::loadModule(
      g_costmap_viewer_so, ayanami::ModuleContext{"costmap_viewer", cfg});
}

template <typename Fn>
bool waitUntil(const Fn& fn, int timeout_ms = 3000) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (fn()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return fn();
}

}  // namespace

TEST(CostmapViewer, slotEmissionsArePublished) {
  initGlobalConfig(makeConfigDir());

  auto ext = loadViewer();
  ASSERT_NE(ext, nullptr);

  // Emit each artifact once; the publisher thread picks the cache up within
  // one rate period (33 ms at the test's 30 Hz).
  const ayanami::nav_msgs::OccupancyGrid grid = makeGrid();
  const int static_before = pubCount("costmapViewerGlobalStaticPubCount");
  ayanami::interfaces::NavigationCallbacks::on_global_costmap_static(grid);
  EXPECT_TRUE(waitUntil(
      [&] { return pubCount("costmapViewerGlobalStaticPubCount") >= static_before + 1; }));

  const int inflation_before = pubCount("costmapViewerGlobalInflationPubCount");
  ayanami::interfaces::NavigationCallbacks::on_global_costmap_inflation(grid);
  EXPECT_TRUE(waitUntil(
      [&] { return pubCount("costmapViewerGlobalInflationPubCount") >= inflation_before + 1; }));

  const int local_before = pubCount("costmapViewerLocalInflationPubCount");
  ayanami::interfaces::NavigationCallbacks::on_local_costmap_inflation(grid);
  EXPECT_TRUE(waitUntil(
      [&] { return pubCount("costmapViewerLocalInflationPubCount") >= local_before + 1; }));
}

TEST(CostmapViewer, cacheOutlivesEmissionAndRepublishes) {
  initGlobalConfig(makeConfigDir());

  auto ext = loadViewer();
  ASSERT_NE(ext, nullptr);

  // Exactly ONE core emission (the static-global "may fire only once"
  // contract), then silence from the core.
  const ayanami::nav_msgs::OccupancyGrid grid = makeGrid();
  ayanami::interfaces::NavigationCallbacks::on_global_costmap_static(grid);
  ASSERT_TRUE(waitUntil(
      [&] { return pubCount("costmapViewerGlobalStaticPubCount") >= 1; }));

  // No further emissions: the viewer keeps republishing the held cache at
  // the configured rate (30 Hz -> dozens of publishes within 300 ms).
  const int before = pubCount("costmapViewerGlobalStaticPubCount");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_GT(pubCount("costmapViewerGlobalStaticPubCount"), before + 1);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "costmap_viewer_test", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}
