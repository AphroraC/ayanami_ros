#include <ayanami_ros/ayanami_ros.hpp>

#include <cstdlib>
#include <stdexcept>

#include <ros/package.h>
#include <tf2_eigen/tf2_eigen.h>

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/utility/load_module.hpp>
#include <ayanami/utility/logging.hpp>

#include <ayanami_ros/ros_msg_converter.hpp>

#include <spdlog/spdlog.h>

namespace ayanami_ros {
namespace {

// 经 Config API (config_ros.json) 读取 tf_sync_edges 列表 [[target, source], ...]
// (spec §7, §4.4)。每条边须正好 2 项 (size==2), 否则跳过; 列表缺失/为空时用
// 配置 map_frame/odom_frame/robot_frame 构造默认三条边。
std::vector<std::pair<std::string, std::string>> readTfSyncEdges(
    const std::string& config_ros_path) {
  ayanami::Config config_ros(config_ros_path);
  const std::string kSec = "ayanami_ros";
  const std::string map_frame =
      config_ros.param<std::string>(kSec, "map_frame", "map");
  const std::string odom_frame =
      config_ros.param<std::string>(kSec, "odom_frame", "odom");
  const std::string robot_frame = config_ros.param<std::string>(
      kSec, "robot_frame", "base_footprint");

  std::vector<std::pair<std::string, std::string>> edges;
  const auto raw = config_ros.param<std::vector<std::vector<std::string>>>(
      kSec, "tf_sync_edges", {});
  for (const auto& edge : raw) {
    if (edge.size() != 2) {
      spdlog::warn("tf_sync_edges entry has size {} != 2, skipped",
                   edge.size());
      continue;
    }
    edges.emplace_back(edge[0], edge[1]);
  }
  if (edges.empty()) {
    // spec §4.4 默认: map→odom, odom→base_link, map→base_link。
    edges = {{map_frame, odom_frame},
             {odom_frame, robot_frame},
             {map_frame, robot_frame}};
  }
  return edges;
}

std::set<std::pair<std::string, std::string>> readTfStaticEdges(
    const std::string& config_ros_path) {
  ayanami::Config config_ros(config_ros_path);
  std::set<std::pair<std::string, std::string>> edges;
  const auto raw = config_ros.param<std::vector<std::vector<std::string>>>(
      "ayanami_ros", "tf_static_edges", {});
  for (const auto& edge : raw) {
    if (edge.size() == 2) {
      edges.emplace(edge[0], edge[1]);
    }
  }
  return edges;
}

// 解析 config_path (member init list 用, 懒求值): param → env → package。
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
    : nh_(nh),
      config_path_(resolveConfigPath(nh)),
      tf_listener_(nh, readTfSyncEdges(config_path_ + "/config_ros.json")),
      configured_static_edges_(
          readTfStaticEdges(config_path_ + "/config_ros.json")),
      navigation_so_name_("libayanami_costmap_navigation.so"),
      navigation_create_symbol_("create_navigation_module") {
  // spec §4.2 verbatim 顺序。
  // 注意: 默认值里 ros::package::getPath() 只有在 param 缺失时才求值 (避免
  // 无 roscore 测试环境触发 rospack 的 tinyxml2 符号冲突 / roscore 依赖)。
  ayanami::GlobalConfig::instance(config_path_, /*override_path=*/true);
  ayanami::configure_logging();
  ayanami::Config config_ros(
      ayanami::GlobalConfig::get_config_path("config_ros"));

  laserscan_topic_ =
      config_ros.param<std::string>("ayanami_ros", "laserscan_topic", "");
  pointcloud_topic_ = config_ros.param<std::string>(
      "ayanami_ros", "pointcloud_topic", "/velodyne_points");
  odom_topic_ =
      config_ros.param<std::string>("ayanami_ros", "odom_topic", "/odom");
  goal_topic_ = config_ros.param<std::string>(
      "ayanami_ros", "goal_topic", "/move_base_simple/goal");
  map_frame_ = config_ros.param<std::string>("ayanami_ros", "map_frame", "map");
  odom_frame_ =
      config_ros.param<std::string>("ayanami_ros", "odom_frame", "odom");
  robot_frame_ = config_ros.param<std::string>(
      "ayanami_ros", "robot_frame", "base_footprint");
  tf_sync_rate_ =
      config_ros.param<double>("ayanami_ros", "tf_sync_rate", 50.0);
  navigation_so_name_ = config_ros.param<std::string>(
      "ayanami_ros", "navigation_so_name", navigation_so_name_);
  navigation_create_symbol_ = config_ros.param<std::string>(
      "ayanami_ros", "navigation_create_symbol", navigation_create_symbol_);

  // 顺序关键 (M2-P1): 扩展必须先于 navigation_plugin->start() 注册其
  // NavigationCallbacks 订阅, 否则启动期第一波可视化 emit 被错过。
  loadExtensionModules(config_ros);
  loadNavigationPlugin(config_ros);
  subscribeTopics();

  spdlog::info(
      "AyanamiROS initialized: config={} extensions={} navigation={}",
      config_path_, extension_modules_.size(), navigation_so_name_);
}

AyanamiROS::~AyanamiROS() { stop(); }

void AyanamiROS::start() {
  if (loop_thread_.joinable()) {
    return;
  }
  kill_switch_.store(false);
  loop_thread_ = std::thread(&AyanamiROS::loop, this);
}

void AyanamiROS::stop() {
  kill_switch_.store(true);
  if (loop_thread_.joinable()) {
    loop_thread_.join();
  }
  if (navigation_plugin_) {
    navigation_plugin_->stop();
  }
}

void AyanamiROS::loadExtensionModules(const ayanami::Config& config_ros) {
  const auto modules = config_ros.param<std::vector<std::string>>(
      "ayanami_ros", "extension_modules", {});
  for (const auto& module : modules) {
    spdlog::info("load ayanami_ros extension module {}", module);
    auto ext = ayanami::ExtensionModule::load_module(module);
    if (!ext) {
      spdlog::warn("failed to load extension module {}", module);
      continue;
    }
    auto ros_ext = std::dynamic_pointer_cast<ExtensionModuleROS>(ext);
    if (!ros_ext) {
      spdlog::warn("extension module {} is not an ExtensionModuleROS", module);
      continue;
    }
    auto subs = ros_ext->create_subscriptions(nh_);
    for (auto& sub : subs) {
      extension_subs_.push_back(std::move(sub.sub));
    }
    extension_modules_.push_back(std::move(ros_ext));
  }
}

void AyanamiROS::loadNavigationPlugin(const ayanami::Config& config_ros) {
  (void)config_ros;
  spdlog::info("load ayanami navigation plugin {}", navigation_so_name_);
  auto create = ayanami::load_module_from_so<
      ayanami::abstract::AbstractNavigation* (*)()>(
      navigation_so_name_, navigation_create_symbol_);
  if (create == nullptr) {
    throw std::runtime_error("Failed to load ayanami navigation plugin: " +
                             navigation_so_name_);
  }
  navigation_plugin_.reset(create());
  navigation_plugin_->initialize(config_path_);
  navigation_plugin_->start();
}

void AyanamiROS::subscribeTopics() {
  if (!laserscan_topic_.empty()) {
    laserscan_sub_ =
        nh_.subscribe(laserscan_topic_, 1, &AyanamiROS::laserscanCallback, this);
    spdlog::info("subscribe obstacle laserscan topic={}", laserscan_topic_);
  }
  if (!pointcloud_topic_.empty()) {
    pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 1,
                                    &AyanamiROS::pointcloudCallback, this);
    spdlog::info("subscribe obstacle pointcloud topic={}", pointcloud_topic_);
  }
  if (!odom_topic_.empty()) {
    odom_sub_ = nh_.subscribe(odom_topic_, 20, &AyanamiROS::odomCallback, this);
    spdlog::info("subscribe odometry topic={}", odom_topic_);
  }
  if (!goal_topic_.empty()) {
    goal_sub_ = nh_.subscribe(goal_topic_, 1, &AyanamiROS::goalCallback, this);
    spdlog::info("subscribe navigation goal topic={}", goal_topic_);
  }
}

void AyanamiROS::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  spdlog::info("navigation goal received frame={} x={:.3f} y={:.3f}",
               msg->header.frame_id, msg->pose.position.x, msg->pose.position.y);
  const ayanami::geometry_msgs::PoseStamped goal = fromRosPose(*msg);
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (navigation_plugin_) {
      navigation_plugin_->setGoal(goal);
    }
  }
}

void AyanamiROS::pointcloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // Task 8: ros_msg_converter 统一转换 (spec §4.3)。
  // Populate the core TF buffer at the observation time before emitting it.
  syncTransformsToCore(msg->header.stamp, 0.05);
  const ayanami::sensor_msgs::PointCloud2 cloud = fromRosPointCloud2(*msg);
  ayanami::interfaces::NavigationCallbacks::on_obstacle_pointcloud(cloud);
}

void AyanamiROS::laserscanCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg) {
  syncTransformsToCore(msg->header.stamp, 0.05);
  const ayanami::sensor_msgs::LaserScan scan = fromRosLaserScan(*msg);
  ayanami::interfaces::NavigationCallbacks::on_obstacle_laserscan(scan);
}

void AyanamiROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  geometry_msgs::TwistStamped ros_twist;
  ros_twist.header = msg->header;
  ros_twist.header.frame_id = msg->child_frame_id;
  ros_twist.twist = msg->twist.twist;
  const ayanami::geometry_msgs::TwistStamped twist = fromRosTwist(ros_twist);
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    last_odom_ = twist;
  }
}

void AyanamiROS::syncTransformsToCore() {
  // Task 7+ (时间语义): 真实 TF2Listener 接入 (spec §4.4)。
  // 对 tf_sync_edges_ 每条边 lookup_transform(target, source, ros::Time(0))
  // → TransformStamped → Eigen::Isometry3d → TransformStore::setTransform。
  // stamp 用 TF2 buffer 返回的真实 TransformStamped.header.stamp (传感器/TF
  // 发布时刻), 不再用 ros::Time::now() 伪造; lookup 用 Time(0) 即 latest。
  syncTransformsToCore(ros::Time(0), 0.0);
}

void AyanamiROS::syncTransformsToCore(const ros::Time& stamp,
                                      double timeout) {
  ayanami::interfaces::TransformStore& store =
      ayanami::interfaces::TransformStore::instance();
  for (const auto& edge : tf_listener_.tfSyncEdges()) {
    const std::string& target = edge.first;
    const std::string& source = edge.second;
    geometry_msgs::TransformStamped tf_msg;
    if (tf_listener_.lookup_transform(target, source, stamp, tf_msg, timeout)) {
      // TF2 returns stamp=0 for a static edge queried as latest, but echoes a
      // concrete query time for the same static edge. fast_tf forbids changing
      // a link between static and dynamic, so classify each configured edge
      // once and keep that type for its process lifetime.
      bool is_static = false;
      {
        std::lock_guard<std::mutex> lock(tf_edge_mutex_);
        const auto key = std::make_pair(target, source);
        const bool configured_static = configured_static_edges_.count(key) != 0;
        const auto inserted = tf_edge_static_.emplace(key, configured_static);
        is_static = inserted.first->second;
      }
      const auto core_stamp =
          is_static ? ayanami::Timestamp::latest()
                    : detail::fromRosTime(tf_msg.header.stamp);
      store.setTransform(target, source, core_stamp,
                         tf2::transformToEigen(tf_msg), is_static);
    }
  }
}

void AyanamiROS::loop() {
  const double rate_hz = tf_sync_rate_ > 0.0 ? tf_sync_rate_ : 50.0;
  ros::WallRate rate(rate_hz);
  spdlog::info("AyanamiROS loop started rate={:.1f}Hz", rate_hz);
  while (ros::ok() && !kill_switch_.load()) {
    syncTransformsToCore();
    if (navigation_plugin_) {
      navigation_plugin_->spinOnce();
    }
    ros::spinOnce();
    rate.sleep();
  }
  spdlog::info("AyanamiROS loop exited");
}

}  // namespace ayanami_ros
