#include <ayanami_ros/ayanami_ros.hpp>

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

// 经 Config API (config_ros.json) 读取 tf_sync_edges 列表 [[target, source], ...]
// (spec §7, §4.4)。每条边须正好 2 项 (size==2), 否则跳过; 列表缺失/为空时用
// 配置 map_frame/odom_frame/robot_frame 构造默认三条边。
std::vector<std::pair<std::string, std::string>> readTfSyncEdges(const std::string& config_ros_path) {
  ayanami::Config config_ros(config_ros_path);
  const std::string kSec = "ayanami_ros";
  const std::string map_frame = config_ros.param<std::string>(kSec, "map_frame", "map");
  const std::string odom_frame = config_ros.param<std::string>(kSec, "odom_frame", "odom");
  const std::string robot_frame = config_ros.param<std::string>(kSec, "robot_frame", "base_footprint");

  std::vector<std::pair<std::string, std::string>> edges;
  const auto raw = config_ros.param<std::vector<std::vector<std::string>>>(kSec, "tf_sync_edges", {});
  for (const auto& edge : raw) {
    if (edge.size() != 2) {
      spdlog::warn("tf_sync_edges entry has size {} != 2, skipped", edge.size());
      continue;
    }
    edges.emplace_back(edge[0], edge[1]);
  }
  if (edges.empty()) {
    // spec §4.4 默认: map→odom, odom→base_link, map→base_link。
    edges = {{map_frame, odom_frame}, {odom_frame, robot_frame}, {map_frame, robot_frame}};
  }
  return edges;
}

std::set<std::pair<std::string, std::string>> readTfStaticEdges(const std::string& config_ros_path) {
  ayanami::Config config_ros(config_ros_path);
  std::set<std::pair<std::string, std::string>> edges;
  const auto raw = config_ros.param<std::vector<std::vector<std::string>>>("ayanami_ros", "tf_static_edges", {});
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
: nh(nh),
  config_path(resolveConfigPath(nh)),
  tf_listener(nh, readTfSyncEdges(config_path + "/config_ros.json")),
  configured_static_edges(readTfStaticEdges(config_path + "/config_ros.json")),
  navigation_so_name("libayanami_costmap_navigation.so") {
  // spec §4.2 verbatim 顺序。
  // 注意: 默认值里 ros::package::getPath() 只有在 param 缺失时才求值 (避免
  // 无 roscore 测试环境触发 rospack 的 tinyxml2 符号冲突 / roscore 依赖)。
  ayanami::GlobalConfig::instance(config_path, /*override_path=*/true);
  ayanami::configureLogging();
  ayanami::Config config_ros(ayanami::GlobalConfig::getConfigPath("config_ros"));

  laserscan_topic = config_ros.param<std::string>("ayanami_ros", "laserscan_topic", "");
  pointcloud_topic = config_ros.param<std::string>("ayanami_ros", "pointcloud_topic", "/velodyne_points");
  odom_topic = config_ros.param<std::string>("ayanami_ros", "odom_topic", "/odom");
  goal_topic = config_ros.param<std::string>("ayanami_ros", "goal_topic", "/move_base_simple/goal");
  map_frame = config_ros.param<std::string>("ayanami_ros", "map_frame", "map");
  odom_frame = config_ros.param<std::string>("ayanami_ros", "odom_frame", "odom");
  robot_frame = config_ros.param<std::string>("ayanami_ros", "robot_frame", "base_footprint");
  tf_sync_rate = config_ros.param<double>("ayanami_ros", "tf_sync_rate", 50.0);
  navigation_so_name = config_ros.param<std::string>("ayanami_ros", "navigation_so_name", navigation_so_name);

  // 顺序关键 (M2-P1): 扩展必须先于 navigation_plugin->start() 注册其
  // NavigationCallbacks 订阅, 否则启动期第一波可视化 emit 被错过。
  loadExtensionModules(config_ros);
  loadNavigationPlugin(config_ros);
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
  if (navigation_plugin) {
    navigation_plugin->stop();
  }
}

void AyanamiROS::loadExtensionModules(const ayanami::Config& config_ros) {
  const auto modules = config_ros.param<std::vector<std::string>>("ayanami_ros", "extension_modules", {});
  for (const auto& module : modules) {
    spdlog::info("load ayanami_ros extension module {}", module);
    auto ext = ayanami::ExtensionModule::load_module(module);
    if (!ext) {
      spdlog::warn("failed to load extension module {}", module);
      continue;
    }
    extension_modules.push_back(std::move(ext));
  }
}

void AyanamiROS::loadNavigationPlugin(const ayanami::Config& config_ros) {
  (void)config_ros;
  spdlog::info("load ayanami navigation plugin {}", navigation_so_name);
  // 公共工厂装载 (requirement 2/4): ExtensionModule::load_module →
  // 类型化 dynamic_pointer_cast<AbstractNavigation> (所有权共享)。
  auto ext = ayanami::ExtensionModule::load_module(navigation_so_name);
  if (!ext) {
    throw std::runtime_error("Failed to load ayanami navigation plugin: " + navigation_so_name);
  }
  navigation_plugin = std::dynamic_pointer_cast<ayanami::abstract::AbstractNavigation>(std::move(ext));
  if (!navigation_plugin) {
    throw std::runtime_error("ayanami navigation plugin '" + navigation_so_name + "' is not an AbstractNavigation");
  }
  navigation_plugin->initialize(config_path);
  navigation_plugin->start();
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
}

void AyanamiROS::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  spdlog::info(
    "navigation goal received frame={} x={:.3f} y={:.3f}",
    msg->header.frame_id,
    msg->pose.position.x,
    msg->pose.position.y);
  // ROS 回调只转换 + 邮箱覆写 (latest wins); 实际下发到导航插件由 loop 线程
  // drainPendingGoal 完成 (不阻塞 ROS 回调线程)。
  submitGoal(fromRosPose(*msg));
}

void AyanamiROS::submitGoal(const ayanami::geometry_msgs::PoseStamped& goal) {
  std::lock_guard<std::mutex> lock(goal_mutex);
  pending_goal = goal;  // 最新覆盖旧待发 goal (latest wins, 不排队)
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
  if (navigation_plugin) {
    navigation_plugin->setGoal(goal);
  }
  return true;
}

bool AyanamiROS::hasPendingGoal() const {
  std::lock_guard<std::mutex> lock(goal_mutex);
  return pending_goal.has_value();
}

void AyanamiROS::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // Task 8: ros_msg_converter 统一转换 (spec §4.3)。
  // Populate the core TF buffer at the observation time before converting.
  syncTransformsToCore(msg->header.stamp, 0.05);
  if (!navigation_plugin) {
    return;
  }
  const ayanami::sensor_msgs::PointCloud2 cloud = fromRosPointCloud2(*msg);
  // 障碍观测输入直递宿主插件 (不走全局旁路槽, 输出/观测分层纪律)。
  navigation_plugin->setPointcloud(cloud);
}

void AyanamiROS::laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  syncTransformsToCore(msg->header.stamp, 0.05);
  if (!navigation_plugin) {
    return;
  }
  const ayanami::sensor_msgs::LaserScan scan = fromRosLaserScan(*msg);
  navigation_plugin->setLaserScan(scan);
}

void AyanamiROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  geometry_msgs::TwistStamped ros_twist;
  ros_twist.header = msg->header;
  ros_twist.header.frame_id = msg->child_frame_id;
  ros_twist.twist = msg->twist.twist;
  updateOdomVelocity(fromRosTwist(ros_twist));
}

void AyanamiROS::updateOdomVelocity(const ayanami::geometry_msgs::TwistStamped& twist) {
  // 直写全局 RobotStateController (核心 controller execution 读取最新速度;
  // 位姿仍由核心侧经 TransformStore 查询)。
  ayanami::components::RobotStateController::instance().setOdometry(twist);
}

void AyanamiROS::syncTransformsToCore() {
  // Task 7+ (时间语义): 真实 TF2Listener 接入 (spec §4.4)。
  // 对 tf_sync_edges 每条边 lookupTransform(target, source, ros::Time(0))
  // → TransformStamped → Eigen::Isometry3d → TransformStore::setTransform。
  // stamp 用 TF2 buffer 返回的真实 TransformStamped.header.stamp (传感器/TF
  // 发布时刻), 不再用 ros::Time::now() 伪造; lookup 用 Time(0) 即 latest。
  // TF producers start alongside this node. Use a short bounded wait so a
  // normal TF warm-up race does not become an immediate lookup failure.
  syncTransformsToCore(ros::Time(0), 0.02);
}

void AyanamiROS::syncTransformsToCore(const ros::Time& stamp, double timeout) {
  auto& robot_state_controller = ayanami::components::RobotStateController::instance();
  for (const auto& edge : tf_listener.tfSyncEdges()) {
    const std::string& target = edge.first;
    const std::string& source = edge.second;
    geometry_msgs::TransformStamped tf_msg;
    if (tf_listener.lookupTransform(target, source, stamp, tf_msg, timeout)) {
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
      const auto core_stamp = is_static ? ayanami::Timestamp::latest() : detail::fromRosTime(tf_msg.header.stamp);
      robot_state_controller.setTransform(target, source, core_stamp, tf2::transformToEigen(tf_msg), is_static);
    }
  }
}

void AyanamiROS::loop() {
  const double rate_hz = tf_sync_rate > 0.0 ? tf_sync_rate : 50.0;
  ros::WallRate rate(rate_hz);
  spdlog::info("AyanamiROS loop started rate={:.1f}Hz", rate_hz);
  while (ros::ok() && !kill_switch.load()) {
    syncTransformsToCore();
    if (navigation_plugin) {
      navigation_plugin->spinOnce();
    }
    drainPendingGoal();  // latest-wins goal 邮箱: 每次最多下发当前最新一条
    ros::spinOnce();
    rate.sleep();
  }
  spdlog::info("AyanamiROS loop exited");
}

}  // namespace ayanami_ros
