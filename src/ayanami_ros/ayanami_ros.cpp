#include <ayanami_ros/ayanami_ros.hpp>

#include <cmath>
#include <cstdlib>
#include <stdexcept>

#include <ros/package.h>
#include <ros/callback_queue.h>
#include <tf2_eigen/tf2_eigen.h>

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/load_module.hpp>
#include <ayanami/utility/logging.hpp>

#include <ayanami_ros/ros_msg_converter.hpp>

#include <spdlog/spdlog.h>

namespace ayanami_ros {

namespace {

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
  tf_listener(nh) {
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
  // Host navigation module = the ACTIVE ROUTE's own declaration
  // (config.json::global.config_navigation -> routes/*.json::navigation.so_name).
  // The key used to live in config_ros.json, which made route switching a
  // two-file edit: switching config.json alone booted the costmap host
  // against a rosemap route, and the resulting uncaught
  // "costmap_global.layers is required" killed the process with SIGABRT
  // (roslaunch: "died of signal 6"). One route file now carries its own host,
  // so the mismatch is unrepresentable.
  const ayanami::Config nav_route(
      ayanami::GlobalConfig::getConfigPath("config_navigation"));
  if (!nav_route.hasParam("navigation", "so_name")) {
    throw std::runtime_error(
        "navigation.so_name missing in " +
        ayanami::GlobalConfig::getConfigPath("config_navigation") +
        " (each route config must declare its host navigation module)");
  }
  navigation_so_name =
      nav_route.param<std::string>("navigation", "so_name", "");

  // robot_frame has one owner (config_navigation.json::navigation.robot_frame);
  // config_ros only mirrors it for the ROS-side TF producers. A mismatch means
  // the stack would build TF edges and costmap geometry around different robot
  // frames, so refuse to start rather than fail subtly at runtime. This check
  // lives here, in the ROS boundary, because it is a ROS-config consistency
  // rule -- the pure-C++ core has no business reading config_ros.json.
  verifyRobotFrameMatchesNavigation();

  // TF bridge: core TF queries are answered on demand from the ROS tf2
  // buffer (interpolation, static edges, stamp-time lookups included).
  installTfSource();

  // Input-slot bridge (must precede loadExtensionModules so no extension
  // emission can ever race an unbound slot): extensions (mbf_action_server)
  // emit NavigationInputs::on_goal_request; the hop into the latest-wins goal
  // mailbox stays owned by AyanamiROS, exactly like the ROS goal topic.
  cb_goal_request =
      ayanami::interfaces::NavigationInputs::on_goal_request.subscribe(
          [this](const ayanami::geometry_msgs::PoseStamped& goal) {
            submitGoal(goal);
          });

  // Order-critical: extensions must register their NavigationCallbacks
  // subscriptions before navigation_module->start, otherwise the first wave of
  // startup visualization emits is missed.
  loadExtensionModules();
  loadNavigationModule(config_ros);
  subscribeTopics();

  SPDLOG_LOGGER_INFO(
    ayanami::rosLogger(),
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
  // Unbind the extension goal input first: after this no extension emission
  // can reach submitGoal, so the mailbox is frozen before the loop thread and
  // the navigation module wind down.
  cb_goal_request.reset();
  kill_switch.store(true);
  if (loop_thread.joinable()) {
    loop_thread.join();
  }
  if (navigation_module) {
    navigation_module->stop();
  }
}

void AyanamiROS::verifyRobotFrameMatchesNavigation() {
  // The core owns robot_frame (config_navigation.json::navigation.robot_frame);
  // config_ros only mirrors it for the ROS-side TF producers. A mismatch means
  // TF edges and costmap geometry would be built around different robot
  // frames -- a failure that surfaces much later as an unexplainable TF gap, so
  // it is checked here, on the ROS boundary, and refused at startup.
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

void AyanamiROS::installTfSource() {
  // The bridge: core threads call RobotStateController::lookupTransform and
  // land here, on the tf2 buffer. The timeout is a steady-clock poll inside
  // TF2Listener (never tf2's ros::Time-clocked wait -- that stalls when
  // /clock stops advancing). Thread safety: tf2 buffers are internally
  // synchronized, so any core thread may query concurrently.
  ayanami::components::RobotStateController::instance().setTransformSource(
      [this](const std::string& target, const std::string& source,
             const ayanami::Timestamp& stamp, Eigen::Isometry3d& transform,
             double timeout) {
        return tf_listener.lookupTransform(target, source,
                                           detail::toRosTime(stamp),
                                           transform, timeout);
      });
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
  const auto modules = extensions_config.param<std::vector<std::string>>("extensions", "loading_extension_modules", {});
  for (const auto& module : modules) {
    SPDLOG_LOGGER_INFO(ayanami::rosLogger(), "load ayanami_ros extension module {}", module);
    // The extension's context: logical name + the extensions config file. The
    // name is derived from the .so the same way the core derives it
    // (libayanami_<name>.so), and it doubles as the module's parameter section
    // key -- so the module never has to resolve the config path itself.
    const std::string name = extensionNameFromSo(module);
    auto ext = ayanami::ExtensionModule::loadModule(module, ayanami::ModuleContext{name, extensions_config});
    if (!ext) {
      SPDLOG_LOGGER_WARN(ayanami::rosLogger(), "failed to load extension module {}", module);
      continue;
    }
    extension_modules.push_back(std::move(ext));
  }
}

void AyanamiROS::loadNavigationModule(const ayanami::Config& config_ros) {
  (void)config_ros;
  SPDLOG_LOGGER_INFO(ayanami::rosLogger(), "load ayanami navigation module {}", navigation_so_name);
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
    SPDLOG_LOGGER_INFO(ayanami::rosLogger(), "subscribe obstacle laserscan topic={}", laserscan_topic);
  }
  if (!pointcloud_topic.empty()) {
    pointcloud_sub = nh.subscribe(pointcloud_topic, 1, &AyanamiROS::pointcloudCallback, this);
    SPDLOG_LOGGER_INFO(ayanami::rosLogger(), "subscribe obstacle pointcloud topic={}", pointcloud_topic);
  }
  if (!odom_topic.empty()) {
    odom_sub = nh.subscribe(odom_topic, 20, &AyanamiROS::odomCallback, this);
    SPDLOG_LOGGER_INFO(ayanami::rosLogger(), "subscribe odometry topic={}", odom_topic);
  }
  if (!goal_topic.empty()) {
    goal_sub = nh.subscribe(goal_topic, 1, &AyanamiROS::goalCallback, this);
    SPDLOG_LOGGER_INFO(ayanami::rosLogger(), "subscribe navigation goal topic={}", goal_topic);
  }
}

void AyanamiROS::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  SPDLOG_LOGGER_INFO(
    ayanami::rosLogger(),
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

void AyanamiROS::setOdometry(const ayanami::geometry_msgs::TwistStamped& twist) {
  // Writes directly into the core RobotStateController (core controller
  // execution reads the latest velocity; pose is queried via the injected
  // tf2 source).
  ayanami::components::RobotStateController::instance().setOdometry(twist);
}

void AyanamiROS::submitSensor(const ayanami::sensor_msgs::PointCloud2& cloud) {
  if (!navigation_module) {
    return;
  }
  // Obstacle observation input delivered directly to the host module (not via
  // global bypass slots; input/output layering discipline).
  navigation_module->setPointcloud(cloud);
}

void AyanamiROS::submitSensor(const ayanami::sensor_msgs::LaserScan& scan) {
  if (!navigation_module) {
    return;
  }
  navigation_module->setLaserScan(scan);
}

void AyanamiROS::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // Conversion goes uniformly through ros_msg_converter; the callback only
  // converts, and the core boundary is submitSensor (direct delivery to the
  // host).
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

void AyanamiROS::checkTfHealth() {
  // Low-frequency health diagnostics: whether the frame-chain edges are
  // resolvable on the ROS tf2 side; while persistently missing, log every 5s
  // with the full frame tree, and log one info after recovery. Used to tell
  // "frames missing from the ROS-side TF tree" apart from "core-side query
  // failures" -- on a missing edge the frame tree is printed directly,
  // showing at a glance which producer is absent or fighting leftover
  // instances.
  const std::pair<std::string, std::string> edges[] = {
      {map_frame, odom_frame}, {odom_frame, robot_frame},
      {map_frame, robot_frame}};
  std::string missing_edges;
  for (const auto& [target, source] : edges) {
    if (!tf_listener.canTransform(target, source, 0.0)) {
      if (!missing_edges.empty()) missing_edges += ", ";
      missing_edges += target + "<-" + source;
    }
  }
  if (missing_edges.empty()) {
    if (tf_health_was_bad) {
      SPDLOG_LOGGER_INFO(ayanami::rosLogger(), "TF health check: all {} configured edges are available again", 3);
      tf_health_was_bad = false;
    }
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const bool due = (now - tf_health_last_log) > std::chrono::seconds(5);
  if (due) {
    tf_health_last_log = now;
    tf_health_was_bad = true;
    SPDLOG_LOGGER_WARN(
      ayanami::rosLogger(),
      "ROS TF health check: unavailable edges [{}]. Current frame "
      "tree:\n{}\nIf odom<-base_footprint is missing, check that the "
      "gazebo diff-drive module (libgazebo_ros_diff_drive, publishOdomTF) "
      "loaded with the model; if map<-odom is missing, wait for or check "
      "AMCL; leftover simulation instances may publish duplicate TF "
      "frames",
      missing_edges,
      tf_listener.framesAsString());
  }
}

void AyanamiROS::loop() {
  ros::WallRate rate(50.0);
  SPDLOG_LOGGER_INFO(ayanami::rosLogger(), "AyanamiROS loop started rate=50.0Hz");
  auto last_tf_health = std::chrono::steady_clock::time_point{};
  while (ros::ok() && !kill_switch.load()) {
    if (std::chrono::steady_clock::now() - last_tf_health >
        std::chrono::seconds(1)) {
      checkTfHealth();
      last_tf_health = std::chrono::steady_clock::now();
    }
    if (navigation_module) {
      navigation_module->spinOnce();
    }
    drainPendingGoal();  // latest-wins goal mailbox: dispatches at most the current latest one per round
    // Spin only THIS handle's queue: the global queue for the standalone
    // node (nh carries no custom queue there, so this equals ros::spinOnce),
    // the nodelet's dedicated queue when embedded -- reaching for the
    // process-global queue from this thread would steal the host manager's
    // other-nodelet callbacks and wreck both sides' pacing. Both queues are
    // ros::CallbackQueue instances, so the concrete cast always succeeds.
    if (auto* queue = dynamic_cast<ros::CallbackQueue*>(nh.getCallbackQueue())) {
      queue->callAvailable(ros::WallDuration(0));
    }
    rate.sleep();
  }
  SPDLOG_LOGGER_INFO(ayanami::rosLogger(), "AyanamiROS loop exited");
}

}  // namespace ayanami_ros
