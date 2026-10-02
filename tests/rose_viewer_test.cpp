// rose_viewer ROS extension .so (dlopen -> create_extension_module ->
// configureParameters): subscribes the core's per-layer costmap artifact
// slots AND the on_esdfmap_field slot, holds the artifacts long-term, and
// republishes them from its own thread at the configured rates (costmap
// layers as OccupancyGrid; the ESDF as a PointCloud2 whose intensity is the
// signed distance, plus the occupied raster as an OccupancyGrid).
//
// dlopen libayanami_rose_viewer.so to verify:
// 1. slot emissions land in the cache and are published on the latched topics;
// 2. the cache outlives the emission (one-shot artifacts keep republishing);
// 3. the ESDF cloud content: strided cell centers with intensity = distance.

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <dlfcn.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

namespace {

#ifndef ROSE_VIEWER_SO
#error "ROSE_VIEWER_SO must be defined by CMake"
#endif

const char* const g_rose_viewer_so = ROSE_VIEWER_SO;

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
      std::filesystem::temp_directory_path() / "ayanami_ros_rose_viewer_test";
  std::filesystem::remove_all(dir);
  writeFile(dir / "config.json",
            "{\n  \"global\": {\n"
            "    \"config_extensions\": \"config_extensions.json\"\n"
            "  }\n}\n");
  writeFile(dir / "config_extensions.json",
            "{\n  \"extensions\": {\n"
            "    \"loading_extension_modules\": []\n"
            "  },\n"
            "  \"rose_viewer\": {\n"
            "    \"global_static_topic\": \"/test/rv/global_static\",\n"
            "    \"global_inflation_topic\": \"/test/rv/global_inflation\",\n"
            "    \"local_obstacle_topic\": \"/test/rv/local_obstacle\",\n"
            "    \"local_inflation_topic\": \"/test/rv/local_inflation\",\n"
            "    \"esdf_field_topic\": \"/test/rv/esdf_field\",\n"
            "    \"esdf_occupancy_topic\": \"/test/rv/esdf_occupancy\",\n"
            "    \"global_static_rate\": 30.0,\n"
            "    \"global_inflation_rate\": 30.0,\n"
            "    \"local_obstacle_rate\": 30.0,\n"
            "    \"local_inflation_rate\": 30.0,\n"
            "    \"esdf_rate\": 30.0,\n"
            "    \"esdf_stride\": 2\n"
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

ayanami::nav_msgs::EsdfGrid makeEsdf() {
  ayanami::nav_msgs::EsdfGrid grid;
  grid.header.frame_id = "odom";
  grid.resolution = 0.5;
  grid.size_x = 4;
  grid.size_y = 4;
  grid.origin_x = 1.0;
  grid.origin_y = -2.0;
  grid.max_distance = 3.0;
  grid.sequence = 7;
  // A signed field over 4x4: distance = x (so cell (0,*) has -0.5 etc. is
  // trivially checkable), obstacles on column 1.
  grid.distance.resize(16);
  grid.occupied.assign(16, 0);
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      grid.distance[static_cast<std::size_t>(y * 4 + x)] =
          static_cast<float>(x);
    }
    grid.occupied[static_cast<std::size_t>(y * 4 + 1)] = 1;
  }
  return grid;
}

/// @brief Load the module the way the production loader does (context
/// injected after a successful load).
ayanami::ExtensionModule::Ptr loadViewer() {
  ayanami::Config cfg((makeConfigDir() / "config_extensions.json").string());
  return ayanami::ExtensionModule::loadModule(
      g_rose_viewer_so, ayanami::ModuleContext{"rose_viewer", cfg});
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

TEST(RoseViewer, slotEmissionsArePublished) {
  initGlobalConfig(makeConfigDir());

  auto ext = loadViewer();
  ASSERT_NE(ext, nullptr);

  // Emit each artifact once; the publisher thread picks the cache up within
  // one rate period (33 ms at the test's 30 Hz).
  const ayanami::nav_msgs::OccupancyGrid grid = makeGrid();
  const int static_before = pubCount("roseViewerGlobalStaticPubCount");
  ayanami::interfaces::NavigationCallbacks::on_global_costmap_static(grid);
  EXPECT_TRUE(waitUntil(
      [&] { return pubCount("roseViewerGlobalStaticPubCount") >= static_before + 1; }));

  const int inflation_before = pubCount("roseViewerGlobalInflationPubCount");
  ayanami::interfaces::NavigationCallbacks::on_global_costmap_inflation(grid);
  EXPECT_TRUE(waitUntil(
      [&] { return pubCount("roseViewerGlobalInflationPubCount") >= inflation_before + 1; }));

  const int local_before = pubCount("roseViewerLocalInflationPubCount");
  ayanami::interfaces::NavigationCallbacks::on_local_costmap_inflation(grid);
  EXPECT_TRUE(waitUntil(
      [&] { return pubCount("roseViewerLocalInflationPubCount") >= local_before + 1; }));

  const int esdf_before = pubCount("roseViewerEsdfCloudPubCount");
  const ayanami::nav_msgs::EsdfGrid esdf_grid = makeEsdf();
  ayanami::interfaces::NavigationCallbacks::on_esdfmap_field(esdf_grid);
  EXPECT_TRUE(waitUntil(
      [&] { return pubCount("roseViewerEsdfCloudPubCount") >= esdf_before + 1; }));
  EXPECT_TRUE(waitUntil(
      [&] { return pubCount("roseViewerEsdfGridPubCount") >= 1; }));
}

TEST(RoseViewer, cacheOutlivesEmissionAndRepublishes) {
  initGlobalConfig(makeConfigDir());

  auto ext = loadViewer();
  ASSERT_NE(ext, nullptr);

  // Exactly ONE core emission (the static-global "may fire only once"
  // contract), then silence from the core.
  const ayanami::nav_msgs::OccupancyGrid grid = makeGrid();
  ayanami::interfaces::NavigationCallbacks::on_global_costmap_static(grid);
  ASSERT_TRUE(waitUntil(
      [&] { return pubCount("roseViewerGlobalStaticPubCount") >= 1; }));

  // No further emissions: the viewer keeps republishing the held cache at
  // the configured rate (30 Hz -> dozens of publishes within 300 ms).
  const int before = pubCount("roseViewerGlobalStaticPubCount");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_GT(pubCount("roseViewerGlobalStaticPubCount"), before + 1);
}

TEST(RoseViewer, esdfCloudCarriesSignedDistance) {
  initGlobalConfig(makeConfigDir());

  // Subscribe BEFORE the emission: the content check needs the actual
  // message, not just the publish counter.
  std::mutex msg_mutex;
  sensor_msgs::PointCloud2::ConstPtr received;
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/test/rv/esdf_field", 2,
      [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(msg_mutex);
        received = msg;
      });

  auto ext = loadViewer();
  ASSERT_NE(ext, nullptr);
  const ayanami::nav_msgs::EsdfGrid esdf_grid = makeEsdf();
  ayanami::interfaces::NavigationCallbacks::on_esdfmap_field(esdf_grid);
  // ros callbacks only run under spinOnce: wait for delivery by spinning.
  ASSERT_TRUE(waitUntil([&] {
    ros::spinOnce();
    std::lock_guard<std::mutex> lock(msg_mutex);
    return received != nullptr;
  }));

  sensor_msgs::PointCloud2::ConstPtr msg;
  {
    std::lock_guard<std::mutex> lock(msg_mutex);
    msg = received;
  }
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->header.frame_id, "odom");

  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*msg, cloud);
  // stride 2 over 4x4 -> 2x2 = 4 points.
  ASSERT_EQ(cloud.points.size(), 4u);
  // First strided cell (x=0, y=0): center at (1.0 + 0.5*0.5, -2.0 + 0.5*0.5).
  const pcl::PointXYZI& first = cloud.points.front();
  EXPECT_NEAR(first.x, 1.25, 1e-5);
  EXPECT_NEAR(first.y, -1.75, 1e-5);
  EXPECT_NEAR(first.z, 0.0, 1e-6);
  // intensity carries the signed distance of that cell (distance[x=0] = 0).
  EXPECT_NEAR(first.intensity, 0.0, 1e-6);
  // The strided cell (x=2, y=2) carries distance 2.0.
  const pcl::PointXYZI& last = cloud.points.back();
  EXPECT_NEAR(last.x, 1.0 + 2.5 * 0.5, 1e-5);
  EXPECT_NEAR(last.intensity, 2.0, 1e-6);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rose_viewer_test", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}
