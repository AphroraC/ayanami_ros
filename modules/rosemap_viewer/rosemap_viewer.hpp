#pragma once

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace ayanami_ros {

// Rosemap field -> ROS output bridge (dedicated extension; mirrors
// costmap_viewer for the rosemap family's single field artifact).
// Subscribes the core's on_rosemap_field slot (the sole rosemap writer
// emits unconditionally on every published snapshot -- sequence-gated, no
// rate parameter) and publishes the field as nav_msgs/OccupancyGrid from a
// dedicated thread:
// - Every received field is cached LONG-TERM (mutex-guarded copy in the slot
//   callback; ros publishing never happens on the writer thread).
// - The publisher thread is EDGE-TRIGGERED on the emission version (a fresh
//   field reaches the ROS topic within one tick -- the configure-time
//   emission may be the only one until observations flow) plus periodic
//   republish at the configured rate for late subscribers. Latched topic.
// Subscriptions register at construction (dlopen in
// AyanamiROS::loadExtensionModules, before navigation_module->start).
class RosemapViewer final : public ayanami::ExtensionModule {
 public:
  RosemapViewer();
  ~RosemapViewer() override;

  /// @brief Parameters arrive AFTER a successful load, injected by the loader
  /// (ModuleContext = logical name + the extensions config). Starts the
  /// publisher thread once topic/rate are known.
  void configureParameters(const ayanami::ModuleContext& context) override;

 private:
  void readConfig(const ayanami::ModuleContext& context);
  void advertiseTopics();
  void registerCallbacks();
  void unregisterCallbacks();
  void publishLoop();

  ros::NodeHandle nh;
  ros::Publisher field_pub;
  std::string field_topic = "/ayanami/rosemap_field";
  /// @brief Periodic republish rate (Hz) for the held cache (late-subscriber
  /// refresh; fresh fields always publish immediately via the edge trigger).
  double field_rate = 5.0;

  std::mutex cache_mutex;
  nav_msgs::OccupancyGrid field_cache;
  bool have_field{false};
  /// @brief Bumped on every slot emission; the publisher edge-triggers on it.
  std::atomic<std::uint64_t> field_version{0};

  ayanami::CallbackSubscription cb_field;
  std::thread publish_thread;
  std::atomic<bool> stop_flag{false};
};

}  // namespace ayanami_ros
