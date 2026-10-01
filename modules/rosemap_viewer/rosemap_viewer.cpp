#include "rosemap_viewer.hpp"

#include <ayanami_ros/ros_msg_converter.hpp>

#include <atomic>
#include <chrono>

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

namespace ayanami_ros {

namespace {

// Test counting hook (read via dlsym after dlopen): ROS publish count.
std::atomic<int> g_field_pub_count{0};

}  // namespace

RosemapViewer::RosemapViewer() : nh("~") {
  registerCallbacks();
}

RosemapViewer::~RosemapViewer() {
  stop_flag.store(true);
  if (publish_thread.joinable()) {
    publish_thread.join();
  }
  unregisterCallbacks();
}

void RosemapViewer::configureParameters(const ayanami::ModuleContext& context) {
  readConfig(context);
  advertiseTopics();
  // The publisher thread starts only once topic/rate exist; the cache may
  // already hold the configure-time emission.
  stop_flag.store(false);
  publish_thread = std::thread([this] { publishLoop(); });
}

void RosemapViewer::readConfig(const ayanami::ModuleContext& context) {
  const ayanami::Config& extensions_config = context.config;
  const std::string& sec = context.name;
  field_topic = extensions_config.param<std::string>(sec, "field_topic",
                                                     field_topic);
  field_rate =
      extensions_config.param<double>(sec, "field_rate", field_rate);
}

void RosemapViewer::advertiseTopics() {
  // Latched: a newly subscribed rviz immediately receives the latest field.
  field_pub = nh.advertise<nav_msgs::OccupancyGrid>(field_topic, 1, true);
  SPDLOG_LOGGER_INFO(
    ayanami::rosLogger(),
    "rosemap_viewer publishers: field={} field_rate={:.3}Hz",
    field_topic,
    field_rate);
}

void RosemapViewer::registerCallbacks() {
  // Long-term caching in the slot callback (runs on the rosemap writer
  // thread): convert + copy only, never publish here.
  cb_field =
      ayanami::interfaces::NavigationCallbacks::on_rosemap_field.subscribe(
          [this](const ayanami::nav_msgs::OccupancyGrid& grid) {
            std::lock_guard<std::mutex> lock(cache_mutex);
            field_cache = toRosOccupancyGrid(grid);
            have_field = true;
            field_version.fetch_add(1);
          });
}

void RosemapViewer::unregisterCallbacks() {
  cb_field.reset();
}

void RosemapViewer::publishLoop() {
  constexpr auto k_tick = std::chrono::milliseconds(10);
  auto last_publish = std::chrono::steady_clock::time_point{};
  std::uint64_t sent_version = 0;
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(field_rate > 0.0 ? 1.0 / field_rate : 0.0));
  while (!stop_flag.load()) {
    const auto now = std::chrono::steady_clock::now();
    // Edge-triggered on the emission version (a fresh field reaches the
    // topic within one tick) OR the periodic rate (late-subscriber refresh
    // of the long-held cache).
    const bool fresh = field_version.load() != sent_version;
    const bool due = period.count() > 0 && now - last_publish >= period;
    if (fresh || due) {
      last_publish = now;
      nav_msgs::OccupancyGrid grid;
      bool have = false;
      {
        std::lock_guard<std::mutex> lock(cache_mutex);
        have = have_field;
        if (have) {
          grid = field_cache;
        }
      }
      if (have) {
        field_pub.publish(grid);
        g_field_pub_count.fetch_add(1);
      }
      sent_version = field_version.load();
    }
    std::this_thread::sleep_for(k_tick);
  }
}

}  // namespace ayanami_ros

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami_ros::RosemapViewer();
}

// ---- TDD test hooks (read via dlsym after dlopen) ----
extern "C" int rosemapViewerFieldPubCount() {
  return ayanami_ros::g_field_pub_count.load();
}
