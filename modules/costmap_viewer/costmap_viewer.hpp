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

// Costmap -> ROS output bridge (dedicated extension; all costmap publishing
// was moved out of rviz_viewer). Subscribes the core's four per-layer
// artifact slots (on_costmap_global_{static,inflation} /
// on_costmap_local_{obstacle,inflation}) and publishes them as
// nav_msgs/OccupancyGrid from a dedicated thread:
// - Every received artifact is cached LONG-TERM: a static global map may be
//   emitted by the core only once (generation-gated writer), so the viewer
//   owns the artifact from then on.
// - The publisher thread republishes each cached grid at ITS OWN configured
//   rate (config_extensions.json::costmap_viewer.{global_static_rate,
//   global_inflation_rate, local_obstacle_rate, local_inflation_rate};
//   defaults 0.1 / 0.1 / 5.0 / 5.0 Hz). The global artifacts are
//   edge-triggered on their emission version (a fresh -- possibly one-shot
//   -- artifact reaches the topic within one publisher tick, independent of
//   the slow periodic rate); the local artifacts are purely rate-driven
//   (they update continuously). Late rviz subscribers get the latest frame
//   from the latched topic immediately.
// Subscriptions register at construction (dlopen in
// AyanamiROS::loadExtensionModules, before navigation_module->start), so the
// configure-time initial emission is captured.
class CostmapViewer final : public ayanami::ExtensionModule {
 public:
  CostmapViewer();
  ~CostmapViewer() override;

  /// @brief Parameters arrive AFTER a successful load, injected by the loader
  /// (ModuleContext = logical name + the extensions config). Starts the
  /// publisher thread once topics/rates are known.
  void configureParameters(const ayanami::ModuleContext& context) override;

 private:
  /// @brief One republished artifact: topic + rate + long-held cache.
  struct Layer {
    ros::Publisher pub;
    std::string topic;
    double rate = 0.1;
    nav_msgs::OccupancyGrid cache;
    bool have = false;
    std::uint64_t version = 0;      // emission counter (edge trigger)
    std::uint64_t sent_version = 0;
    bool edge_triggered = false;    // global artifacts: fresh emits publish
                                    // immediately; local: purely rate-driven
    std::chrono::steady_clock::time_point last_publish{};
  };

  void readConfig(const ayanami::ModuleContext& context);
  void advertiseTopics();
  void registerCallbacks();
  void unregisterCallbacks();
  void publishLoop();
  /// @brief Publish one layer if due (edge-triggered fresh emit or its own
  /// rate); assumes cache_mutex held.
  void publishLayerIfDue(Layer& layer,
                         const std::chrono::steady_clock::time_point& now,
                         std::atomic<int>& pub_count);

  ros::NodeHandle nh;
  std::mutex cache_mutex;
  Layer global_static;
  Layer global_inflation;
  Layer local_obstacle;
  Layer local_inflation;

  ayanami::CallbackSubscription cb_global_static, cb_global_inflation,
      cb_local_obstacle, cb_local_inflation;
  std::thread publish_thread;
  std::atomic<bool> stop_flag{false};
};

}  // namespace ayanami_ros
