#include <ayanami_ros/ayanami_ros.hpp>

#include <cstdlib>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include <ros/package.h>

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/utility/load_module.hpp>

#include <ayanami_ros/ros_msg_converter.hpp>

#include <spdlog/spdlog.h>

namespace ayanami_ros {
namespace {

// M1 Config 未显式实例化 std::vector<std::string> param (config.cpp 的
// DEFINE_CONFIG_IO_SPECIALIZATION 列表无 vector<string>)。故 loadExtensionModules
// 直接从 config_ros.json 文件读取 extension_modules 数组 (spec §4.5 step 1)。
// 用 ayanami vendored nlohmann (3.11.3, 系统 3.7.3 与本 toolchain 不兼容)。
std::vector<std::string> readExtensionModules(const std::string& path) {
  try {
    std::ifstream ifs(path);
    if (!ifs) {
      return {};
    }
    const nlohmann::json doc = nlohmann::json::parse(ifs, nullptr, false);
    const auto ros_it = doc.find("ayanami_ros");
    if (ros_it == doc.end() || !ros_it->is_object()) {
      return {};
    }
    const auto it = ros_it->find("extension_modules");
    if (it == ros_it->end() || !it->is_array()) {
      return {};
    }
    return it->get<std::vector<std::string>>();
  } catch (const nlohmann::json::exception& e) {
    spdlog::error("readExtensionModules({}): {}", path, e.what());
    return {};
  }
}

// M1 Config 未实例化 vector<string> 的嵌套 pair 序列, 直接 nlohmann 读
// config_ros.json::ayanami_ros::tf_sync_edges 列表 [[target, source], ...]
// (spec §7, §4.4)。缺失时用配置 frame 回退默认三条边。
std::vector<std::pair<std::string, std::string>> readTfSyncEdges(
    const std::string& path) {
  std::vector<std::pair<std::string, std::string>> edges;
  std::string map_frame = "map", odom_frame = "odom", robot_frame = "base_link";
  try {
    std::ifstream ifs(path);
    if (ifs) {
      const nlohmann::json doc = nlohmann::json::parse(ifs, nullptr, false);
      const auto ros_it = doc.find("ayanami_ros");
      if (ros_it != doc.end() && ros_it->is_object()) {
        auto read_str = [&ros_it](const char* key, const std::string& def) {
          const auto it = ros_it->find(key);
          return (it != ros_it->end() && it->is_string())
                     ? it->get<std::string>()
                     : def;
        };
        map_frame = read_str("map_frame", map_frame);
        odom_frame = read_str("odom_frame", odom_frame);
        robot_frame = read_str("robot_frame", robot_frame);
        const auto it = ros_it->find("tf_sync_edges");
        if (it != ros_it->end() && it->is_array()) {
          for (const auto& e : *it) {
            if (e.is_array() && e.size() == 2 && e[0].is_string() &&
                e[1].is_string()) {
              edges.emplace_back(e[0].get<std::string>(),
                                 e[1].get<std::string>());
            }
          }
        }
      }
    }
  } catch (const nlohmann::json::exception& e) {
    spdlog::error("readTfSyncEdges({}): {}", path, e.what());
  }
  if (edges.empty()) {
    // spec §4.4 默认: map→odom, odom→base_link, map→base_link。
    edges = {{map_frame, odom_frame},
             {odom_frame, robot_frame},
             {map_frame, robot_frame}};
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
      navigation_so_name_("libayanami_costmap_navigation.so"),
      navigation_create_symbol_("create_navigation_module") {
  // spec §4.2 verbatim 顺序。
  // 注意: 默认值里 ros::package::getPath() 只有在 param 缺失时才求值 (避免
  // 无 roscore 测试环境触发 rospack 的 tinyxml2 符号冲突 / roscore 依赖)。
  ayanami::GlobalConfig::instance(config_path_, /*override_path=*/true);
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
  (void)config_ros;
  const std::string config_ros_path =
      ayanami::GlobalConfig::get_config_path("config_ros");
  const auto modules = readExtensionModules(config_ros_path);
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
  const ayanami::sensor_msgs::PointCloud2 cloud = fromRosPointCloud2(*msg);
  ayanami::interfaces::NavigationCallbacks::on_obstacle_pointcloud(cloud);
}

void AyanamiROS::laserscanCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg) {
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
  // Task 7: 真实 TF2Listener 接入 (spec §4.4)。
  // 对 tf_sync_edges_ 每条边 lookup_transform(target, source, ros::Time(0))
  // → Eigen::Isometry3d → TransformStore::setTransform (stamp 用当前时间,
  // 替代 M1 默认 0s; lookup 用 Time(0) 即 latest)。
  ayanami::interfaces::TransformStore& store =
      ayanami::interfaces::TransformStore::instance();
  const double now = ros::Time::now().toSec();
  for (const auto& edge : tf_listener_.tfSyncEdges()) {
    const std::string& target = edge.first;
    const std::string& source = edge.second;
    Eigen::Isometry3d T;
    if (tf_listener_.lookup_transform(target, source, ros::Time(0), T)) {
      store.setTransform(
          target, source,
          ayanami::interfaces::TransformStore::fromSeconds(now), T);
    }
  }
}

void AyanamiROS::loop() {
  const double rate_hz = tf_sync_rate_ > 0.0 ? tf_sync_rate_ : 50.0;
  ros::Rate rate(rate_hz);
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
