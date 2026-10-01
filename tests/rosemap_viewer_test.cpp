// rosemap_viewer ROS extension .so (dlopen -> create_extension_module ->
// configureParameters): subscribes the core's on_rosemap_field slot, holds
// the field long-term, and republishes it from its own thread at the
// configured rate.
//
// dlopen libayanami_rosemap_viewer.so to verify:
// 1. slot emissions land in the cache and are published on the latched topic;
// 2. the cache outlives the emission: with no further core emissions the
//    publisher thread keeps republishing at the configured rate (and a
//    fresh emission reaches the topic immediately via the edge trigger).

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

#ifndef ROSEMAP_VIEWER_SO
#error "ROSEMAP_VIEWER_SO must be defined by CMake"
#endif

const char* const g_rosemap_viewer_so = ROSEMAP_VIEWER_SO;

void writeFile(const std::filesystem::path& path, const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream ofs(path);
  ASSERT_TRUE(ofs.good()) << "cannot write " << path;
  ofs << content;
}

/// @brief Fast rate for deterministic testing (the production default is
/// 5 Hz; 30 Hz here makes the republish cadence observable in milliseconds).
std::filesystem::path makeConfigDir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_ros_rosemap_viewer_test";
  std::filesystem::remove_all(dir);
  writeFile(dir / "config.json",
            "{\n  \"global\": {\n"
            "    \"config_extensions\": \"config_extensions.json\"\n"
            "  }\n}\n");
  writeFile(dir / "config_extensions.json",
            "{\n  \"extensions\": {\n"
            "    \"loading_extension_modules\": []\n"
            "  },\n"
            "  \"rosemap_viewer\": {\n"
            "    \"field_topic\": \"/test/rm/field\",\n"
            "    \"field_rate\": 30.0\n"
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
      g_rosemap_viewer_so, ayanami::ModuleContext{"rosemap_viewer", cfg});
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

TEST(RosemapViewer, slotEmissionsArePublished) {
  initGlobalConfig(makeConfigDir());

  auto ext = loadViewer();
  ASSERT_NE(ext, nullptr);

  // One slot emission; the publisher thread picks the cache up within one
  // tick (the edge trigger, not the rate).
  const ayanami::nav_msgs::OccupancyGrid grid = makeGrid();
  const int before = pubCount("rosemapViewerFieldPubCount");
  ayanami::interfaces::NavigationCallbacks::on_rosemap_field(grid);
  EXPECT_TRUE(waitUntil(
      [&] { return pubCount("rosemapViewerFieldPubCount") >= before + 1; }));
}

TEST(RosemapViewer, cacheOutlivesEmissionAndRepublishes) {
  initGlobalConfig(makeConfigDir());

  auto ext = loadViewer();
  ASSERT_NE(ext, nullptr);

  // Exactly ONE core emission, then silence from the core.
  const ayanami::nav_msgs::OccupancyGrid grid = makeGrid();
  ayanami::interfaces::NavigationCallbacks::on_rosemap_field(grid);
  ASSERT_TRUE(waitUntil([&] { return pubCount("rosemapViewerFieldPubCount") >= 1; }));

  // No further emissions: the viewer keeps republishing the held cache at
  // the configured rate (30 Hz -> dozens of publishes within 300 ms).
  const int before = pubCount("rosemapViewerFieldPubCount");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_GT(pubCount("rosemapViewerFieldPubCount"), before + 1);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rosemap_viewer_test", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}
