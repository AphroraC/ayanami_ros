#include "costmap_viewer.hpp"

#include <ayanami_ros/ros_msg_converter.hpp>

#include <atomic>
#include <chrono>

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

namespace ayanami_ros {

namespace {

// Test counting hooks (read via dlsym after dlopen): ROS publish count per
// artifact.
std::atomic<int> g_global_static_pub_count{0};
std::atomic<int> g_global_inflation_pub_count{0};
std::atomic<int> g_local_obstacle_pub_count{0};
std::atomic<int> g_local_inflation_pub_count{0};

}  // namespace

CostmapViewer::CostmapViewer() : nh("~") {
  global_static.topic = "/ayanami/global_costmap/static";
  global_static.rate = 0.1;
  global_static.edge_triggered = true;
  global_inflation.topic = "/ayanami/global_costmap/inflation";
  global_inflation.rate = 0.1;
  global_inflation.edge_triggered = true;
  local_obstacle.topic = "/ayanami/local_costmap/obstacle";
  local_obstacle.rate = 5.0;
  local_inflation.topic = "/ayanami/local_costmap/inflation";
  local_inflation.rate = 5.0;
  registerCallbacks();
}

CostmapViewer::~CostmapViewer() {
  stop_flag.store(true);
  if (publish_thread.joinable()) {
    publish_thread.join();
  }
  unregisterCallbacks();
}

void CostmapViewer::configureParameters(const ayanami::ModuleContext& context) {
  readConfig(context);
  advertiseTopics();
  // The publisher thread starts only once topics/rates exist; the caches may
  // already hold artifacts emitted between construction and configure.
  stop_flag.store(false);
  publish_thread = std::thread([this] { publishLoop(); });
}

void CostmapViewer::readConfig(const ayanami::ModuleContext& context) {
  const ayanami::Config& extensions_config = context.config;
  const std::string& sec = context.name;
  global_static.topic = extensions_config.param<std::string>(
      sec, "global_static_topic", global_static.topic);
  global_inflation.topic = extensions_config.param<std::string>(
      sec, "global_inflation_topic", global_inflation.topic);
  local_obstacle.topic = extensions_config.param<std::string>(
      sec, "local_obstacle_topic", local_obstacle.topic);
  local_inflation.topic = extensions_config.param<std::string>(
      sec, "local_inflation_topic", local_inflation.topic);
  global_static.rate = extensions_config.param<double>(
      sec, "global_static_rate", global_static.rate);
  global_inflation.rate = extensions_config.param<double>(
      sec, "global_inflation_rate", global_inflation.rate);
  local_obstacle.rate = extensions_config.param<double>(
      sec, "local_obstacle_rate", local_obstacle.rate);
  local_inflation.rate = extensions_config.param<double>(
      sec, "local_inflation_rate", local_inflation.rate);
}

void CostmapViewer::advertiseTopics() {
  // Latched: a newly subscribed rviz immediately receives the latest frame
  // (on top of the periodic cache republish).
  global_static.pub =
      nh.advertise<nav_msgs::OccupancyGrid>(global_static.topic, 1, true);
  global_inflation.pub =
      nh.advertise<nav_msgs::OccupancyGrid>(global_inflation.topic, 1, true);
  local_obstacle.pub =
      nh.advertise<nav_msgs::OccupancyGrid>(local_obstacle.topic, 1, true);
  local_inflation.pub =
      nh.advertise<nav_msgs::OccupancyGrid>(local_inflation.topic, 1, true);
  SPDLOG_LOGGER_INFO(
    ayanami::rosLogger(),
    "costmap_viewer publishers: global_static={} ({:.3}Hz) "
    "global_inflation={} ({:.3}Hz) local_obstacle={} ({:.3}Hz) "
    "local_inflation={} ({:.3}Hz)",
    global_static.topic, global_static.rate,
    global_inflation.topic, global_inflation.rate,
    local_obstacle.topic, local_obstacle.rate,
    local_inflation.topic, local_inflation.rate);
}

void CostmapViewer::registerCallbacks() {
  // Long-term caching happens in the slot callbacks: whatever the core emits
  // is held from that moment on, independent of the publish rates.
  cb_global_static =
      ayanami::interfaces::NavigationCallbacks::on_global_costmap_static.subscribe(
          [this](const ayanami::nav_msgs::OccupancyGrid& grid) {
            std::lock_guard<std::mutex> lock(cache_mutex);
            global_static.cache = toRosOccupancyGrid(grid);
            global_static.have = true;
            ++global_static.version;
          });
  cb_global_inflation =
      ayanami::interfaces::NavigationCallbacks::on_global_costmap_inflation.subscribe(
          [this](const ayanami::nav_msgs::OccupancyGrid& grid) {
            std::lock_guard<std::mutex> lock(cache_mutex);
            global_inflation.cache = toRosOccupancyGrid(grid);
            global_inflation.have = true;
            ++global_inflation.version;
          });
  cb_local_obstacle =
      ayanami::interfaces::NavigationCallbacks::on_local_costmap_obstacle.subscribe(
          [this](const ayanami::nav_msgs::OccupancyGrid& grid) {
            std::lock_guard<std::mutex> lock(cache_mutex);
            local_obstacle.cache = toRosOccupancyGrid(grid);
            local_obstacle.have = true;
          });
  cb_local_inflation =
      ayanami::interfaces::NavigationCallbacks::on_local_costmap_inflation.subscribe(
          [this](const ayanami::nav_msgs::OccupancyGrid& grid) {
            std::lock_guard<std::mutex> lock(cache_mutex);
            local_inflation.cache = toRosOccupancyGrid(grid);
            local_inflation.have = true;
          });
}

void CostmapViewer::unregisterCallbacks() {
  cb_global_static.reset();
  cb_global_inflation.reset();
  cb_local_obstacle.reset();
  cb_local_inflation.reset();
}

void CostmapViewer::publishLayerIfDue(
    Layer& layer, const std::chrono::steady_clock::time_point& now,
    std::atomic<int>& pub_count) {
  if (!layer.have || layer.rate <= 0.0) {
    return;
  }
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / layer.rate));
  const bool due = now - layer.last_publish >= period;
  const bool fresh =
      layer.edge_triggered && layer.version != layer.sent_version;
  if (!due && !fresh) {
    return;
  }
  layer.last_publish = now;
  layer.sent_version = layer.version;
  layer.pub.publish(layer.cache);
  pub_count.fetch_add(1);
}

void CostmapViewer::publishLoop() {
  constexpr auto k_tick = std::chrono::milliseconds(10);
  while (!stop_flag.load()) {
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(cache_mutex);
      publishLayerIfDue(global_static, now, g_global_static_pub_count);
      publishLayerIfDue(global_inflation, now, g_global_inflation_pub_count);
      publishLayerIfDue(local_obstacle, now, g_local_obstacle_pub_count);
      publishLayerIfDue(local_inflation, now, g_local_inflation_pub_count);
    }
    std::this_thread::sleep_for(k_tick);
  }
}

}  // namespace ayanami_ros

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami_ros::CostmapViewer();
}

// ---- TDD test hooks (read via dlsym after dlopen) ----
extern "C" int costmapViewerGlobalStaticPubCount() {
  return ayanami_ros::g_global_static_pub_count.load();
}
extern "C" int costmapViewerGlobalInflationPubCount() {
  return ayanami_ros::g_global_inflation_pub_count.load();
}
extern "C" int costmapViewerLocalObstaclePubCount() {
  return ayanami_ros::g_local_obstacle_pub_count.load();
}
extern "C" int costmapViewerLocalInflationPubCount() {
  return ayanami_ros::g_local_inflation_pub_count.load();
}
