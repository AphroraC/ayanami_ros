#include <ayanami_ros/ayanami_ros.hpp>

#include <cmath>
#include <cstdlib>
#include <stdexcept>

#include <ros/package.h>
#include <tf2_eigen/tf2_eigen.h>

#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/utility/load_module.hpp>
#include <ayanami/utility/logging.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>

#include <ayanami_ros/ros_msg_converter.hpp>

#include <spdlog/spdlog.h>

namespace ayanami_ros {
namespace {

// tf_sync_edges / tf_static_edges entry format (config_ros.json):
//   "<target>-<source>[:<rate>hz]"   e.g. "map-odom:50hz", "base_link-laser"
// The rate suffix is optional; a missing suffix falls back to the global
// tf_sync_rate. For tf_static_edges the suffix is accepted but ignored
// (static transforms are re-asserted; their value never changes). The legacy
// [[target, source]] array format is still accepted (deprecated; every edge
// gets the default rate).
TfSyncEdge parseTfEdge(const std::string& entry, double default_rate,
                       std::pair<std::string, std::string>& pair_out) {
  TfSyncEdge edge;
  // Rate suffix: everything after the last ':' (frame names never contain ':').
  double rate = default_rate;
  std::string pair_part = entry;
  const auto colon = entry.rfind(':');
  if (colon != std::string::npos) {
    pair_part = entry.substr(0, colon);
    std::string rate_part = entry.substr(colon + 1);
    for (auto& ch : rate_part) {
      ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    if (rate_part.size() >= 2 &&
        rate_part.compare(rate_part.size() - 2, 2, "hz") == 0) {
      rate_part.erase(rate_part.size() - 2);
    }
    try {
      rate = std::stod(rate_part);
    } catch (const std::exception&) {
      rate = -1.0;
    }
    if (!(rate > 0.0)) {
      spdlog::warn("tf edge '{}' has an invalid rate suffix; using the "
                   "default {}hz",
                   entry, default_rate);
      rate = default_rate;
    }
  }
  // Edge: split at the LAST '-' (the source/child frame name must not
  // contain '-', see the TfSyncEdge doc).
  const auto dash = pair_part.rfind('-');
  if (dash == std::string::npos || dash == 0 || dash + 1 >= pair_part.size()) {
    spdlog::warn("tf edge '{}' is not '<target>-<source>', skipped", entry);
    edge.rate_hz = -1.0;  // invalid-entry marker
    pair_out = {entry, {}};
    return edge;
  }
  edge.target = pair_part.substr(0, dash);
  edge.source = pair_part.substr(dash + 1);
  edge.rate_hz = rate;
  pair_out = {edge.target, edge.source};
  return edge;
}

std::vector<TfSyncEdge> readTfSyncEdges(const std::string& config_ros_path) {
  ayanami::Config config_ros(config_ros_path);
  const std::string kSec = "ayanami_ros";
  const std::string map_frame = config_ros.param<std::string>(kSec, "map_frame", "map");
  const std::string odom_frame = config_ros.param<std::string>(kSec, "odom_frame", "odom");
  const std::string robot_frame = config_ros.param<std::string>(kSec, "robot_frame", "base_footprint");
  const double default_rate =
      config_ros.param<double>(kSec, "tf_sync_rate", 50.0);

  std::vector<TfSyncEdge> edges;
  // New format: a plain vector<string> of "target-source[:rate]hz" entries.
  std::vector<std::string> raw;
  try {
    raw = config_ros.param<std::vector<std::string>>(kSec, "tf_sync_edges", {});
  } catch (const std::exception&) {
    // Legacy format (deprecated): [[target, source], ...].
    for (const auto& edge : config_ros.param<std::vector<std::vector<std::string>>>(
             kSec, "tf_sync_edges", {})) {
      if (edge.size() == 2) {
        raw.push_back(edge[0] + "-" + edge[1]);
      }
    }
    if (!raw.empty()) {
      spdlog::warn("tf_sync_edges uses the deprecated [[target, source]] "
                   "array format; migrate to \"target-source:ratehz\" strings");
    }
  }
  for (const auto& entry : raw) {
    std::pair<std::string, std::string> pair_out;
    const TfSyncEdge edge = parseTfEdge(entry, default_rate, pair_out);
    if (edge.rate_hz > 0.0) {
      edges.push_back(edge);
    }
  }
  if (edges.empty()) {
    // Default: map-odom, odom-base_link, map-base_link (all default rate).
    const std::pair<std::string, std::string> defaults[] = {
        {map_frame, odom_frame}, {odom_frame, robot_frame},
        {map_frame, robot_frame}};
    for (const auto& [t, src] : defaults) {
      edges.push_back({t, src, default_rate});
    }
  }
  return edges;
}

std::set<std::pair<std::string, std::string>> readTfStaticEdges(
    const std::string& config_ros_path) {
  ayanami::Config config_ros(config_ros_path);
  const double default_rate =
      config_ros.param<double>("ayanami_ros", "tf_sync_rate", 50.0);
  std::set<std::pair<std::string, std::string>> edges;
  std::vector<std::string> raw;
  try {
    raw = config_ros.param<std::vector<std::string>>("ayanami_ros",
                                                     "tf_static_edges", {});
  } catch (const std::exception&) {
    for (const auto& edge : config_ros.param<std::vector<std::vector<std::string>>>(
             "ayanami_ros", "tf_static_edges", {})) {
      if (edge.size() == 2) {
        raw.push_back(edge[0] + "-" + edge[1]);
      }
    }
  }
  for (const auto& entry : raw) {
    std::pair<std::string, std::string> pair_out;
    // Static edge: the rate suffix is accepted but ignored (format check only).
    parseTfEdge(entry, default_rate, pair_out);
    if (!pair_out.second.empty()) {
      edges.insert(pair_out);
    }
  }
  return edges;
}

// Resolves config_path (used in the member init list, lazily evaluated):
// param -> env -> package.
std::string resolveConfigPath(const ros::NodeHandle& nh) {
  std::string path;
  nh.param<std::string>("config_path", path, "");
  if (path.empty()) {
    const char* env = std::getenv("AYANAMI_CONFIG_PATH");
    path = env ? env : (ros::package::getPath("ayanami") + "/config");
  }
  return path;
}

}  // namespace

AyanamiROS::AyanamiROS(ros::NodeHandle& nh)
: nh(nh),
  config_path(resolveConfigPath(nh)),
  tf_listener(nh, readTfSyncEdges(config_path + "/config_ros.json")),
  configured_static_edges(readTfStaticEdges(config_path + "/config_ros.json")),
  navigation_so_name("libayanami_costmap_navigation.so") {
  // verbatim ordering.
  // Note: in the default values, ros::package::getPath is only evaluated when
  // the param is missing (avoids triggering rospack's tinyxml2 symbol conflict
  // / roscore dependency in roscore-less test environments).
  ayanami::GlobalConfig::instance(config_path, /*override_path=*/true);
  ayanami::configureLogging();
  ayanami::Config config_ros(ayanami::GlobalConfig::getConfigPath("config_ros"));

  laserscan_topic = config_ros.param<std::string>("ayanami_ros", "laserscan_topic", "");
  pointcloud_topic = config_ros.param<std::string>("ayanami_ros", "pointcloud_topic", "/velodyne_points");
  odom_topic = config_ros.param<std::string>("ayanami_ros", "odom_topic", "/odom");
  goal_topic = config_ros.param<std::string>("ayanami_ros", "goal_topic", "/move_base_simple/goal");
  map_frame = config_ros.param<std::string>("ayanami_ros", "map_frame", "map");
  odom_frame = config_ros.param<std::string>("ayanami_ros", "odom_frame", "odom");
  // Default mirrors the core's default (config_navigation.json::
  // navigation.robot_frame). Keeping the two defaults equal means "neither
  // config sets it" resolves to one frame, not a silent mismatch.
  robot_frame = config_ros.param<std::string>("ayanami_ros", "robot_frame", "base_link");
  tf_sync_rate = config_ros.param<double>("ayanami_ros", "tf_sync_rate", 50.0);
 // tf_cache_duration: TransformStore dynamic-edge lookback window by stamp
 // (seconds, default 10s aligned with tf2 DEFAULT_MAX_STORAGE_TIME). Must be
 // configured before any TF write (this constructor runs before
 // loadNavigationModule/start, so this is the only valid configuration point).
  const double tf_cache_duration =
      config_ros.param<double>("ayanami_ros", "tf_cache_duration", 10.0);
  ayanami::interfaces::TransformStore::instance().configureCacheDuration(
      tf_cache_duration);
  localization_clear_yaw_threshold = config_ros.param<double>(
      "ayanami_ros", "localization_clear_yaw_threshold", 1.0);
  localization_clear_pos_threshold = config_ros.param<double>(
      "ayanami_ros", "localization_clear_pos_threshold", 2.0);
  navigation_so_name = config_ros.param<std::string>("ayanami_ros", "navigation_so_name", navigation_so_name);

  // robot_frame has one owner (config_navigation.json::navigation.robot_frame);
  // config_ros only mirrors it for the ROS-side TF producers. A mismatch means
  // the stack would build TF edges and costmap geometry around different robot
  // frames, so refuse to start rather than fail subtly at runtime. This check
  // lives here, in the ROS boundary, because it is a ROS-config consistency
  // rule -- the pure-C++ core has no business reading config_ros.json.
  verifyRobotFrameMatchesNavigation();

  // Order-critical: extensions must register their NavigationCallbacks
  // subscriptions before navigation_module->start, otherwise the first wave of
  // startup visualization emits is missed.
  loadExtensionModules();
  loadNavigationModule(config_ros);
  subscribeTopics();

  spdlog::info(
    "AyanamiROS initialized: config={} extensions={} navigation={}",
    config_path,
    extension_modules.size(),
    navigation_so_name);
}

AyanamiROS::~AyanamiROS() {
  stop();
}

void AyanamiROS::start() {
  if (loop_thread.joinable()) {
    return;
  }
  kill_switch.store(false);
  loop_thread = std::thread(&AyanamiROS::loop, this);
}

void AyanamiROS::stop() {
  kill_switch.store(true);
  if (loop_thread.joinable()) {
    loop_thread.join();
  }
  tf_queue.disable();
  if (tf_queue_thread.joinable()) {
    tf_queue_thread.join();
  }
  tf_queue.clear();
  if (navigation_module) {
    navigation_module->stop();
  }
}

void AyanamiROS::verifyRobotFrameMatchesNavigation() {
  // The core owns robot_frame (config_navigation.json::navigation.robot_frame);
  // config_ros only mirrors it for the ROS-side TF producers. A mismatch means
  // TF edges and costmap geometry would be built around different robot frames
  // -- a failure that surfaces much later as an unexplainable TF gap, so it is
  // checked here, on the ROS boundary, and refused at startup.
  //
  // This used to be an assert inside the core's CostmapNavigationServer. That
  // was wrong twice over: the pure-C++ core has no business opening
  // config_ros.json, and the assert was compiled out in the Release builds.
  const std::string nav_config =
      ayanami::GlobalConfig::getConfigPath("config_navigation");
  ayanami::Config nav_cfg(nav_config);
  // Same literal default as the core's reader, so an omitted key compares equal
  // instead of tripping the check.
  const std::string nav_frame =
      nav_cfg.param<std::string>("navigation", "robot_frame", "base_link");
  if (robot_frame != nav_frame) {
    throw std::runtime_error(
        "robot_frame mismatch: config_ros.json::ayanami_ros.robot_frame='" +
        robot_frame +
        "' vs config_navigation.json::navigation.robot_frame='" + nav_frame +
        "'. The navigation config owns the robot frame; config_ros must mirror "
        "it (or omit the key to inherit the default).");
  }
}

namespace {

/// @brief Logical module name from its .so: the base name without the library
/// prefix and extension (libayanami_rviz_viewer.so -> rviz_viewer). It is the
/// section key the module reads its parameters from, so the loader can hand a
/// module its context without the module knowing the config layout.
std::string extensionNameFromSo(const std::string& so_name) {
  std::string base = so_name;
  const std::size_t slash = base.find_last_of('/');
  if (slash != std::string::npos) {
    base = base.substr(slash + 1);
  }
  for (const std::string prefix : {"libayanami_", "lib"}) {
    if (base.rfind(prefix, 0) == 0) {
      base = base.substr(prefix.size());
      break;
    }
  }
  const std::size_t dot = base.rfind(".so");
  if (dot != std::string::npos && dot + 3 == base.size()) {
    base = base.substr(0, dot);
  }
  return base;
}

}  // namespace

void AyanamiROS::loadExtensionModules() {
  // Single source for the extension manifest =
  // config_extensions.json::extensions.loading_extension_modules (asuka mode);
  // each extension reads its own parameters from its same-named section (e.g.
  // rviz_viewer).
  // config_ros.json only carries the ROS input boundary and no longer holds
  // extension_modules.
  const ayanami::Config extensions_config(
      ayanami::GlobalConfig::getConfigPath("config_extensions"));
  const auto modules = extensions_config.param<std::vector<std::string>>(
      "extensions", "loading_extension_modules", {});
  for (const auto& module : modules) {
    spdlog::info("load ayanami_ros extension module {}", module);
    // The extension's context: logical name + the extensions config file. The
    // name is derived from the .so the same way the core derives it
    // (libayanami_<name>.so), and it doubles as the module's parameter section
    // key -- so the module never has to resolve the config path itself.
    const std::string name = extensionNameFromSo(module);
    auto ext = ayanami::ExtensionModule::loadModule(
        module, ayanami::ModuleContext{name, extensions_config});
    if (!ext) {
      spdlog::warn("failed to load extension module {}", module);
      continue;
    }
    extension_modules.push_back(std::move(ext));
  }
}

void AyanamiROS::loadNavigationModule(const ayanami::Config& config_ros) {
  (void)config_ros;
  spdlog::info("load ayanami navigation module {}", navigation_so_name);
  // Common factory loading (4): ExtensionModule::loadModule ->
  // typed dynamic_pointer_cast<AbstractNavigation> (shared ownership).
  auto ext = ayanami::ExtensionModule::loadModule(navigation_so_name);
  if (!ext) {
    throw std::runtime_error("Failed to load ayanami navigation module: " + navigation_so_name);
  }
  navigation_module = std::dynamic_pointer_cast<ayanami::abstract::AbstractNavigation>(std::move(ext));
  if (!navigation_module) {
    throw std::runtime_error("ayanami navigation module '" + navigation_so_name + "' is not an AbstractNavigation");
  }
  navigation_module->initialize(config_path);
  navigation_module->start();
}

void AyanamiROS::subscribeTopics() {
  if (!laserscan_topic.empty()) {
    laserscan_sub = nh.subscribe(laserscan_topic, 1, &AyanamiROS::laserscanCallback, this);
    spdlog::info("subscribe obstacle laserscan topic={}", laserscan_topic);
  }
  if (!pointcloud_topic.empty()) {
    pointcloud_sub = nh.subscribe(pointcloud_topic, 1, &AyanamiROS::pointcloudCallback, this);
    spdlog::info("subscribe obstacle pointcloud topic={}", pointcloud_topic);
  }
  if (!odom_topic.empty()) {
    odom_sub = nh.subscribe(odom_topic, 20, &AyanamiROS::odomCallback, this);
    spdlog::info("subscribe odometry topic={}", odom_topic);
  }
  if (!goal_topic.empty()) {
    goal_sub = nh.subscribe(goal_topic, 1, &AyanamiROS::goalCallback, this);
    spdlog::info("subscribe navigation goal topic={}", goal_topic);
  }

  // Event-driven TF sync: every /tf message immediately triggers a full-edge
  // sync (dedicated callback queue + thread, bypassing the loop's
  // ros::spinOnce -- otherwise the callback rate is clamped by the 50Hz
  // polling ceiling).
  // The 50Hz loop sync is kept as a fallback (covers /tf_static edges and
  // extreme message-loss scenarios).
  tf_nh = ros::NodeHandle();
  tf_nh.setCallbackQueue(&tf_queue);
  tf_sub = tf_nh.subscribe("/tf", 200, &AyanamiROS::tfCallback, this,
                             ros::TransportHints().tcpNoDelay());
  tf_queue_thread = std::thread([this] {
    while (ros::ok() && !kill_switch.load()) {
      tf_queue.callAvailable(ros::WallDuration(0.0005));
    }
  });
  spdlog::info("subscribe /tf for event-driven transform sync");
}

void AyanamiROS::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  spdlog::info(
    "navigation goal received frame={} x={:.3f} y={:.3f}",
    msg->header.frame_id,
    msg->pose.position.x,
    msg->pose.position.y);
  // The ROS callback only converts + overwrites the mailbox (latest wins); the
  // actual dispatch to the navigation module is done by the loop thread's
  // drainPendingGoal (does not block the ROS callback thread).
  submitGoal(fromRosPose(*msg));
}

void AyanamiROS::submitGoal(const ayanami::geometry_msgs::PoseStamped& goal) {
  std::lock_guard<std::mutex> lock(goal_mutex);
  pending_goal = goal;  // newest overwrites the older pending goal (latest wins, no queueing)
}

bool AyanamiROS::drainPendingGoal() {
  ayanami::geometry_msgs::PoseStamped goal;
  {
    std::lock_guard<std::mutex> lock(goal_mutex);
    if (!pending_goal) {
      return false;
    }
    goal = *pending_goal;
    pending_goal.reset();
  }
  if (navigation_module) {
    navigation_module->setGoal(goal);
  }
  return true;
}

bool AyanamiROS::hasPendingGoal() const {
  std::lock_guard<std::mutex> lock(goal_mutex);
  return pending_goal.has_value();
}

void AyanamiROS::setTransform(const std::string& parent,
                              const std::string& child,
                              ayanami::Timestamp stamp,
                              const Eigen::Isometry3d& transform,
                              bool is_static) {
  ayanami::components::RobotStateController::instance().setTransform(
      parent, child, std::move(stamp), transform, is_static);
}

void AyanamiROS::setOdometry(const ayanami::geometry_msgs::TwistStamped& twist) {
  // Writes directly into the core RobotStateController (core controller
  // execution reads the latest velocity; pose is still queried via TF on the
  // core side).
  ayanami::components::RobotStateController::instance().setOdometry(twist);
}

void AyanamiROS::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
  if (kill_switch.load()) {
    return;
  }
  // First inject the messages into the buffer synchronously (removing the
  // race with TransformListener's asynchronous insertion: otherwise this
  // callback might run before the insertion, read a stale sample, and skip the
  // frame forever), then trigger the sync.
  // syncTransformsToCore serializes internally with sync_mutex; no lock is
  // needed here.
  for (const auto& transform : msg->transforms) {
    tf_listener.setTransform(transform);
  }
  syncTransformsToCore(ros::Time(0), 0.0);
}

void AyanamiROS::submitSensor(const ayanami::sensor_msgs::PointCloud2& cloud) {
 // Populate the core TF buffer before delivering: prefer syncing at the
 // sensor's header.stamp (the ROS TF buffer's own cache can interpolate the
 // edge transforms at the sensor time and write exact samples, so the core's
 // stamp-based obstacle lookups can hit); on failures such as a stamp ahead of
 // the ROS buffer's newest sample, fall back to latest. timeout=0: the callback
 // thread is never blocked.
  const ros::Time sensor_stamp = detail::toRosTime(cloud.header.stamp);
  if (!sensor_stamp.isZero()) {
    syncTransformsToCore(sensor_stamp, 0.0);
  }
  syncTransformsToCore(ros::Time(0), 0.0);
  if (!navigation_module) {
    return;
  }
  // Obstacle observation input delivered directly to the host module (not via
  // global bypass slots; input/output layering discipline).
  navigation_module->setPointcloud(cloud);
}

void AyanamiROS::submitSensor(const ayanami::sensor_msgs::LaserScan& scan) {
  const ros::Time sensor_stamp = detail::toRosTime(scan.header.stamp);
  if (!sensor_stamp.isZero()) {
    syncTransformsToCore(sensor_stamp, 0.0);
  }
  syncTransformsToCore(ros::Time(0), 0.0);
  if (!navigation_module) {
    return;
  }
  navigation_module->setLaserScan(scan);
}

void AyanamiROS::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // Conversion goes uniformly through ros_msg_converter; the callback only
  // converts, and the core boundary is submitSensor (TF pre-sync + direct
  // delivery to the host).
  if (!msg) {
    return;
  }
  submitSensor(fromRosPointCloud2(*msg));
}

void AyanamiROS::laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (!msg) {
    return;
  }
  submitSensor(fromRosLaserScan(*msg));
}

void AyanamiROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  if (!msg) {
    return;
  }
  geometry_msgs::TwistStamped ros_twist;
  ros_twist.header = msg->header;
  ros_twist.header.frame_id = msg->child_frame_id;
  ros_twist.twist = msg->twist.twist;
  setOdometry(fromRosTwist(ros_twist));
}

void AyanamiROS::syncTransformsToCore() {
  // + (time semantics): real TF2Listener wired in.
  // For each edge in tf_sync_edges: lookupTransform(target, source,
  // ros::Time(0)) -> TransformStamped -> Eigen::Isometry3d ->
  // TransformStore::setTransform.
  // The stamp is the real TransformStamped.header.stamp returned by the TF2
  // buffer (sensor/TF publication time), no longer faked with ros::Time::now;
  // the lookup uses Time(0), i.e. latest.
  // timeout must be 0: tf2_ros's wait-with-timeout is clocked by ros::Time::now
  // (sim time); when /clock stalls (llvmpipe/physics hiccups) it would block
  // the 50Hz sync loop forever -- once the loop blocks, scan rasterization /
  // rolling window / goal dispatch all freeze (one measured root cause of
  // hangs). Missing edges are covered by checkTfHealth + the sync missing-edge
  // diagnostics; the next cycle simply retries.
  syncTransformsToCore(ros::Time(0), 0.0);
}

void AyanamiROS::syncTransformsToCore(const ros::Time& stamp, double timeout) {
  // tf thread / loop thread / sensor callbacks may enter concurrently;
  // serialization protects the tf_edge_* intermediate state and the downstream
  // writer's ordering assumptions.
  std::lock_guard<std::mutex> sync_lock(sync_mutex);
  const auto now = std::chrono::steady_clock::now();
  const auto& edges = tf_listener.tfSyncEdges();
  // Per-edge sync-rate gating (tf_sync_edges "<target>-<source>:<rate>hz"):
  // an edge is sampled at most at its configured rate, no matter whether this
  // pass was driven by the loop, the event-driven /tf callback or a
  // sensor-callback TF pre-sync. Fresh /tf data lands within one configured
  // period at worst.
  if (edge_last_sync.size() != edges.size()) {
    edge_last_sync.assign(edges.size(), std::chrono::steady_clock::time_point{});
  }
  for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
    const auto& edge = edges[edge_index];
    const auto period = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::duration<double>(1.0 / std::max(edge.rate_hz, 1e-6)));
    if (now - edge_last_sync[edge_index] < period) {
      continue;
    }
    edge_last_sync[edge_index] = now;
    const std::string& target = edge.target;
    const std::string& source = edge.source;
    geometry_msgs::TransformStamped tf_msg;
    if (tf_listener.lookupTransform(target, source, stamp, tf_msg, timeout)) {
      {
        std::lock_guard<std::mutex> lock(tf_edge_mutex);
        tf_missing_since.erase(std::make_pair(target, source));
      }
      // TF2 returns stamp=0 for a static edge queried as latest, but echoes a
      // concrete query time for the same static edge. fast_tf forbids changing
      // a link between static and dynamic, so classify each configured edge
      // once and keep that type for its process lifetime.
      bool is_static = false;
      {
        std::lock_guard<std::mutex> lock(tf_edge_mutex);
        const auto key = std::make_pair(target, source);
        const bool configured_static = configured_static_edges.count(key) != 0;
        const auto inserted = tf_edge_static.emplace(key, configured_static);
        is_static = inserted.first->second;
      }
      auto core_stamp = detail::fromRosTime(tf_msg.header.stamp);
 // Future-stamp clamping: the localization layer writes the map->odom stamp
 // into the future by transform_tolerance (measured AMCL tolerance=0.5 ->
 // +0.5s). Once TransformStore's 500ms stamp window is occupied by future
 // samples, sensor-stamp queries land before the window and all fail. Clamp to
 // the current clock so the stamp sequence stays close to real time.
      {
        const auto now_stamp = detail::fromRosTime(ros::Time::now());
        if (core_stamp.count() > now_stamp.count()) {
          core_stamp = now_stamp;
        }
      }
 // Note: the old tf_time_offset stamp-shift patch was retired along with
 // fast_tf's configurable lookback window (tf_cache_duration); dynamic edges
 // now write the real header.stamp.
      const Eigen::Isometry3d edge_tf = tf2::transformToEigen(tf_msg);
      setTransform(target, source, core_stamp, edge_tf, is_static);
      // Global dynamic map rebuild trigger (only the odom<-base_footprint
      // dynamic edge): accumulate robot rotation/translation and clear the
      // global dynamic layer once the threshold is exceeded (see the hpp
      // comment).
      if (!is_static && target == odom_frame && source == robot_frame) {
        std::lock_guard<std::mutex> jump_lock(localization_jump_mutex);
        const Eigen::Vector3d delta =
            edge_tf.translation() - localization_jump_prev.translation();
        const double dyaw_raw =
            Eigen::Rotation2Dd(edge_tf.linear().topLeftCorner<2, 2>())
                .angle() -
            Eigen::Rotation2Dd(
                localization_jump_prev.linear().topLeftCorner<2, 2>())
                .angle();
        const double dyaw = std::atan2(std::sin(dyaw_raw), std::cos(dyaw_raw));
        if (localization_jump_prev_valid) {
          localization_accum_yaw += std::abs(dyaw);
          localization_accum_pos += delta.norm();
        }
        localization_jump_prev = edge_tf;
        localization_jump_prev_valid = true;
        if (localization_jump_prev_valid &&
            (localization_accum_yaw > localization_clear_yaw_threshold ||
             localization_accum_pos > localization_clear_pos_threshold)) {
          if (navigation_module) {
            spdlog::info(
                "accumulated motion since last clear (dyaw={:.3f} rad, "
                "dpos={:.3f} m); clearing global dynamic costmap",
                localization_accum_yaw, localization_accum_pos);
            navigation_module->onLocalizationJump();
          }
          localization_accum_yaw = 0.0;
          localization_accum_pos = 0.0;
        }
      }
    } else {
      // Persistent missing-edge diagnostics (emitted only by the loop's
      // low-frequency path; sensor callbacks use timeout=0, a quick liveness
      // probe that never blocks here). When an edge has been unavailable for
      // >=2s continuously and >=5s have passed since the last diagnostic, log
      // once with the most common environmental root causes (multiple
      // simulation instances concurrently publishing the same TF names /
      // missing publisher).
      const auto key = std::make_pair(target, source);
      bool need_diag = false;
      {
        std::lock_guard<std::mutex> lock(tf_edge_mutex);
        const auto first = tf_missing_since.emplace(key, now);
        const auto diag_at = tf_diag_logged_at.find(key);
        if (!first.second) {
          need_diag =
              now - first.first->second > std::chrono::seconds(2) &&
              (diag_at == tf_diag_logged_at.end() ||
               now - diag_at->second > std::chrono::seconds(5));
        }
        if (need_diag) {
          tf_diag_logged_at[key] = now;
        }
      }
      if (need_diag) {
        spdlog::warn(
            "TF sync missing edge {} <- {} for over 2s: obstacle stamping "
            "and the robot pose will misbehave. Check the gazebo diff-drive "
            "(odom->base_footprint), AMCL (map->odom) and "
            "robot_state_publisher; if roslaunch ran several times, leftover "
            "simulation instances may publish duplicate TF frames",
            target, source);
      }
    }
  }
}

void AyanamiROS::checkTfHealth() {
  // Low-frequency health diagnostics: whether each sync edge is resolvable on
  // the ROS TF side; while persistently missing, log every 5s (with the full
  // frame tree), and log one info after recovery. Used to tell "frames missing
  // from the ROS-side TF tree" apart from "core-side lookup failures" -- on a
  // missing edge the frame tree is printed directly, showing at a glance which
  // producer is absent or fighting leftover instances.
  const auto now = std::chrono::steady_clock::now();
  std::string missing_edges;
  for (const auto& edge : tf_listener.tfSyncEdges()) {
    const std::string& target = edge.target;
    const std::string& source = edge.source;
    if (!tf_listener.canTransform(target, source, 0.0)) {
      if (!missing_edges.empty()) missing_edges += ", ";
      missing_edges += target + "<-" + source;
    }
  }
  if (missing_edges.empty()) {
    if (tf_health_was_bad) {
      spdlog::info("TF health check: all {} synced edges are available again",
                   tf_listener.tfSyncEdges().size());
      tf_health_was_bad = false;
    }
    return;
  }
  const bool due = (now - tf_health_last_log) > std::chrono::seconds(5);
  if (due) {
    tf_health_last_log = now;
    tf_health_was_bad = true;
    spdlog::warn(
        "ROS TF health check: unavailable edges [{}]. Current frame "
        "tree:\n{}\nIf odom<-base_footprint is missing, check that the "
        "gazebo diff-drive module (libgazebo_ros_diff_drive, publishOdomTF) "
        "loaded with the model; if map<-odom is missing, wait for or check "
        "AMCL; leftover simulation instances may publish duplicate TF "
        "frames",
        missing_edges, tf_listener.framesAsString());
  }
}

void AyanamiROS::loop() {
  const double rate_hz = tf_sync_rate > 0.0 ? tf_sync_rate : 50.0;
  ros::WallRate rate(rate_hz);
  spdlog::info("AyanamiROS loop started rate={:.1f}Hz", rate_hz);
  auto last_tf_health = std::chrono::steady_clock::time_point{};
  while (ros::ok() && !kill_switch.load()) {
    syncTransformsToCore();
    if (std::chrono::steady_clock::now() - last_tf_health >
        std::chrono::seconds(1)) {
      checkTfHealth();
      last_tf_health = std::chrono::steady_clock::now();
    }
    if (navigation_module) {
      navigation_module->spinOnce();
    }
    drainPendingGoal();  // latest-wins goal mailbox: dispatches at most the current latest one per round
    ros::spinOnce();
    rate.sleep();
  }
  spdlog::info("AyanamiROS loop exited");
}

}  // namespace ayanami_ros
