#include "rose_viewer.hpp"

#include <ayanami_ros/ros_msg_converter.hpp>

#include <pcl/point_types.h>

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
std::atomic<int> g_esdf_cloud_pub_count{0};
std::atomic<int> g_esdf_grid_pub_count{0};

}  // namespace

RoseViewer::RoseViewer() : nh("~") {
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
  esdf.cloud_topic = "/ayanami/esdf/field";
  esdf.grid_topic = "/ayanami/esdf/occupancy";
  registerCallbacks();
}

RoseViewer::~RoseViewer() {
  stop_flag.store(true);
  if (publish_thread.joinable()) {
    publish_thread.join();
  }
  unregisterCallbacks();
}

void RoseViewer::configureParameters(const ayanami::ModuleContext& context) {
  readConfig(context);
  advertiseTopics();
  // The publisher thread starts only once topics/rates exist; the caches may
  // already hold artifacts emitted between construction and configure.
  stop_flag.store(false);
  publish_thread = std::thread([this] { publishLoop(); });
}

void RoseViewer::readConfig(const ayanami::ModuleContext& context) {
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
  esdf.cloud_topic = extensions_config.param<std::string>(
      sec, "esdf_field_topic", esdf.cloud_topic);
  esdf.grid_topic = extensions_config.param<std::string>(
      sec, "esdf_occupancy_topic", esdf.grid_topic);
  esdf.rate = extensions_config.param<double>(sec, "esdf_rate", esdf.rate);
  const int stride = extensions_config.param<int>(sec, "esdf_stride", esdf.stride);
  esdf.stride = stride >= 1 ? stride : 1;
}

void RoseViewer::advertiseTopics() {
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
  esdf.cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>(esdf.cloud_topic, 1, true);
  esdf.grid_pub =
      nh.advertise<nav_msgs::OccupancyGrid>(esdf.grid_topic, 1, true);
  SPDLOG_LOGGER_INFO(
    ayanami::rosLogger(),
    "rose_viewer publishers: global_static={} ({:.3}Hz) "
    "global_inflation={} ({:.3}Hz) local_obstacle={} ({:.3}Hz) "
    "local_inflation={} ({:.3}Hz) esdf_field={} esdf_occupancy={} "
    "({:.3}Hz, stride {})",
    global_static.topic, global_static.rate,
    global_inflation.topic, global_inflation.rate,
    local_obstacle.topic, local_obstacle.rate,
    local_inflation.topic, local_inflation.rate,
    esdf.cloud_topic, esdf.grid_topic, esdf.rate, esdf.stride);
}

void RoseViewer::registerCallbacks() {
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
  cb_esdfmap_field =
      ayanami::interfaces::NavigationCallbacks::on_esdfmap_field.subscribe(
          [this](const ayanami::nav_msgs::EsdfGrid& grid) {
            std::lock_guard<std::mutex> lock(cache_mutex);
            esdf.cache = grid;
            esdf.have = true;
          });
}

void RoseViewer::unregisterCallbacks() {
  cb_global_static.reset();
  cb_global_inflation.reset();
  cb_local_obstacle.reset();
  cb_local_inflation.reset();
  cb_esdfmap_field.reset();
}

void RoseViewer::publishLayerIfDue(
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

void RoseViewer::publishEsdfIfDue(
    const std::chrono::steady_clock::time_point& now) {
  const std::size_t grid_cells = static_cast<std::size_t>(esdf.cache.size_x) *
                                 static_cast<std::size_t>(esdf.cache.size_y);
  if (!esdf.have || esdf.rate <= 0.0 || grid_cells == 0 ||
      grid_cells > esdf.cache.distance.size() ||
      (esdf.cache.occupied.size() != grid_cells && !esdf.cache.occupied.empty())) {
    return;
  }
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / esdf.rate));
  if (now - esdf.last_publish < period) {
    return;
  }
  esdf.last_publish = now;

  const ayanami::nav_msgs::EsdfGrid& grid = esdf.cache;

  // Distance-field point cloud (rose_navigation esdf_out style): one point
  // per strided cell center, z = 0, intensity = SIGNED distance in meters.
  // rviz's PointCloud (color transformer Intensity) renders the field.
  {
    ayanami::sensor_msgs::PointCloud2 cloud;
    cloud.header = grid.header;
    const int stride = esdf.stride >= 1 ? esdf.stride : 1;
    const auto cells = static_cast<std::size_t>(
        ((grid.size_x + stride - 1) / stride) *
        ((grid.size_y + stride - 1) / stride));
    cloud.cloud.reserve(cells);
    for (int y = 0; y < grid.size_y; y += stride) {
      for (int x = 0; x < grid.size_x; x += stride) {
        const std::size_t index =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(
                                               grid.size_x) +
            static_cast<std::size_t>(x);
        if (index >= grid.distance.size()) {
          break;
        }
        pcl::PointXYZI point;
        point.x = static_cast<float>(grid.origin_x +
                                     (x + 0.5) * grid.resolution);
        point.y = static_cast<float>(grid.origin_y +
                                     (y + 0.5) * grid.resolution);
        point.z = 0.0f;
        point.intensity = grid.distance[index];
        cloud.cloud.push_back(point);
      }
    }
    esdf.cloud_pub.publish(toRosPointCloud2(cloud));
    g_esdf_cloud_pub_count.fetch_add(1);
  }

  // Occupied raster as a classic occupancy grid (100 = obstacle, 0 = free).
  ayanami::nav_msgs::OccupancyGrid core;
  core.header = grid.header;
  core.info.resolution = grid.resolution;
  core.info.width = static_cast<std::uint32_t>(grid.size_x);
  core.info.height = static_cast<std::uint32_t>(grid.size_y);
  core.info.origin.position.x() = grid.origin_x;
  core.info.origin.position.y() = grid.origin_y;
  core.info.origin.position.z() = 0.0;
  core.info.origin.orientation.w() = 1.0;
  core.data.assign(core.info.width * core.info.height, 0);
  for (std::size_t i = 0;
       i < core.data.size() && i < grid.occupied.size(); ++i) {
    core.data[i] = grid.occupied[i] ? 100 : 0;
  }
  esdf.grid_pub.publish(toRosOccupancyGrid(core));
  g_esdf_grid_pub_count.fetch_add(1);
}

void RoseViewer::publishLoop() {
  constexpr auto k_tick = std::chrono::milliseconds(10);
  while (!stop_flag.load()) {
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(cache_mutex);
      publishLayerIfDue(global_static, now, g_global_static_pub_count);
      publishLayerIfDue(global_inflation, now, g_global_inflation_pub_count);
      publishLayerIfDue(local_obstacle, now, g_local_obstacle_pub_count);
      publishLayerIfDue(local_inflation, now, g_local_inflation_pub_count);
      publishEsdfIfDue(now);
    }
    std::this_thread::sleep_for(k_tick);
  }
}

}  // namespace ayanami_ros

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami_ros::RoseViewer();
}

// ---- TDD test hooks (read via dlsym after dlopen) ----
extern "C" int roseViewerGlobalStaticPubCount() {
  return ayanami_ros::g_global_static_pub_count.load();
}
extern "C" int roseViewerGlobalInflationPubCount() {
  return ayanami_ros::g_global_inflation_pub_count.load();
}
extern "C" int roseViewerLocalObstaclePubCount() {
  return ayanami_ros::g_local_obstacle_pub_count.load();
}
extern "C" int roseViewerLocalInflationPubCount() {
  return ayanami_ros::g_local_inflation_pub_count.load();
}
extern "C" int roseViewerEsdfCloudPubCount() {
  return ayanami_ros::g_esdf_cloud_pub_count.load();
}
extern "C" int roseViewerEsdfGridPubCount() {
  return ayanami_ros::g_esdf_grid_pub_count.load();
}
