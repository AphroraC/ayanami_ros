#pragma once

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace ayanami_ros {

// RoseNavigationFsm visualization bridge (costmap_viewer's successor, named
// after the host it serves). Everything arrives through the core's global
// callback slots and leaves as ROS messages for rviz:
// - the four costmap layer artifacts (on_global_costmap_{static,inflation} /
//   on_local_costmap_{obstacle,inflation}) -> nav_msgs/OccupancyGrid, with the
//   costmap_viewer caching discipline: every received artifact is cached
//   LONG-TERM (a static global map may be emitted only once), and a dedicated
//   publisher thread republishes each grid at its own configured rate; global
//   artifacts are edge-triggered on their emission version, local artifacts
//   are purely rate-driven; all topics latched;
// - the ESDF field artifact (on_esdfmap_field, emitted by the host through
//   the EsdfWriter snapshot listener) -> sensor_msgs/PointCloud2 whose
//   intensity carries the SIGNED DISTANCE in meters (rose_navigation esdf_out
//   style: rviz renders the distance field as a colored point cloud, x/y at
//   cell centers, z = 0), decimated by a configurable stride; plus a
//   nav_msgs/OccupancyGrid of the binarized occupied raster (100/0) for the
//   classic grid view.
// Subscriptions register at construction (dlopen in
// AyanamiROS::loadExtensionModules, before navigation_module->start), so
// configure-time initial emissions are captured.
class RoseViewer final : public ayanami::ExtensionModule {
 public:
  RoseViewer();
  ~RoseViewer() override;

  /// @brief Parameters arrive AFTER a successful load, injected by the loader
  /// (ModuleContext = logical name + the extensions config). Starts the
  /// publisher thread once topics/rates are known.
  void configureParameters(const ayanami::ModuleContext& context) override;

 private:
  /// @brief One republished costmap artifact: topic + rate + long-held cache.
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

  /// @brief The ESDF artifact pair: distance-field point cloud + occupied
  /// raster grid, both driven by one cached EsdfGrid at one shared rate.
  struct EsdfLayer {
    ros::Publisher cloud_pub;
    ros::Publisher grid_pub;
    std::string cloud_topic;
    std::string grid_topic;
    double rate = 2.0;
    int stride = 2;  // point decimation step (rose_navigation uses 5)
    ayanami::nav_msgs::EsdfGrid cache;
    bool have = false;
    std::chrono::steady_clock::time_point last_publish{};
  };

  void readConfig(const ayanami::ModuleContext& context);
  void advertiseTopics();
  void registerCallbacks();
  void unregisterCallbacks();
  void publishLoop();
  /// @brief Publish one costmap layer if due (edge-triggered fresh emit or
  /// its own rate); assumes cache_mutex held.
  void publishLayerIfDue(Layer& layer,
                         const std::chrono::steady_clock::time_point& now,
                         std::atomic<int>& pub_count);
  /// @brief Publish the ESDF cloud + grid if due; assumes cache_mutex held.
  void publishEsdfIfDue(const std::chrono::steady_clock::time_point& now);

  ros::NodeHandle nh;
  std::mutex cache_mutex;
  Layer global_static;
  Layer global_inflation;
  Layer local_obstacle;
  Layer local_inflation;
  EsdfLayer esdf;

  ayanami::CallbackSubscription cb_global_static, cb_global_inflation,
      cb_local_obstacle, cb_local_inflation, cb_esdfmap_field;
  std::thread publish_thread;
  std::atomic<bool> stop_flag{false};
};

}  // namespace ayanami_ros
