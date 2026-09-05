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
  localization_clear_yaw_threshold = config_ros.param<double>(
      "ayanami_ros", "localization_clear_yaw_threshold", 1.0);
  localization_clear_pos_threshold = config_ros.param<double>(
      "ayanami_ros", "localization_clear_pos_threshold", 2.0);
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

void AyanamiROS::setTransform(const std::string& parent,
                              const std::string& child,
                              ayanami::Timestamp stamp,
                              const Eigen::Isometry3d& transform,
                              bool is_static) {
  ayanami::components::RobotStateController::instance().setTransform(
      parent, child, std::move(stamp), transform, is_static);
}

void AyanamiROS::setOdometry(const ayanami::geometry_msgs::TwistStamped& twist) {
  // 直写核心 RobotStateController (核心 controller execution 读取最新速度;
  // 位姿仍由核心侧经 TF 查询)。
  ayanami::components::RobotStateController::instance().setOdometry(twist);
}

void AyanamiROS::submitSensor(const ayanami::sensor_msgs::PointCloud2& cloud) {
  // Populate the core TF buffer before converting. 用 latest (Time(0)) 而
  // 非传感器戳: 快于实时的仿真下传感器戳恒晚于 loop 已同步样本, fast_tf
  // 无外推, 精确戳同步/查询会失败。latest 语义与障碍落图查询完全同源。
  // timeout=0: 只同步当前已可用的边, TF 生产者短暂缺席时回调不阻塞。
  syncTransformsToCore(ros::Time(0), 0.0);
  if (!navigation_plugin) {
    return;
  }
  // 障碍观测输入直递宿主插件 (不走全局旁路槽, 输出/观测分层纪律)。
  navigation_plugin->setPointcloud(cloud);
}

void AyanamiROS::submitSensor(const ayanami::sensor_msgs::LaserScan& scan) {
  syncTransformsToCore(ros::Time(0), 0.0);
  if (!navigation_plugin) {
    return;
  }
  navigation_plugin->setLaserScan(scan);
}

void AyanamiROS::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // Task 8: ros_msg_converter 统一转换 (spec §4.3); 回调只转换, 核心边界经
  // submitSensor (TF 预同步 + 直递宿主)。
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
  // Task 7+ (时间语义): 真实 TF2Listener 接入 (spec §4.4)。
  // 对 tf_sync_edges 每条边 lookupTransform(target, source, ros::Time(0))
  // → TransformStamped → Eigen::Isometry3d → TransformStore::setTransform。
  // stamp 用 TF2 buffer 返回的真实 TransformStamped.header.stamp (传感器/TF
  // 发布时刻), 不再用 ros::Time::now() 伪造; lookup 用 Time(0) 即 latest。
  // timeout 必须为 0: tf2_ros 的带 timeout 等待以 ros::Time::now() (sim time)
  // 计时, /clock 停摆 (llvmpipe/物理卡顿) 时会无限阻塞 50Hz 同步循环 —— 循环
  // 一旦阻塞, 扫描落图/滚动窗口/目标下发全部冻结 (实测卡死根因之一)。缺边由
  // checkTfHealth + 同步缺边诊断兜底, 下一周期重试即可。
  syncTransformsToCore(ros::Time(0), 0.0);
}

void AyanamiROS::syncTransformsToCore(const ros::Time& stamp, double timeout) {
  const auto now = std::chrono::steady_clock::now();
  for (const auto& edge : tf_listener.tfSyncEdges()) {
    const std::string& target = edge.first;
    const std::string& source = edge.second;
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
      const auto core_stamp = is_static ? ayanami::Timestamp::latest() : detail::fromRosTime(tf_msg.header.stamp);
      const Eigen::Isometry3d edge_tf = tf2::transformToEigen(tf_msg);
      setTransform(target, source, core_stamp, edge_tf, is_static);
      // 全局动态图重建触发 (仅 odom<-base_footprint 动态边): 累计机器人
      // 旋转/平移, 超阈值即清空全局动态层 (见 hpp 注释)。
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
          if (navigation_plugin) {
            spdlog::info(
                "accumulated motion since last clear (dyaw={:.3f} rad, "
                "dpos={:.3f} m); clearing global dynamic costmap",
                localization_accum_yaw, localization_accum_pos);
            navigation_plugin->onLocalizationJump();
          }
          localization_accum_yaw = 0.0;
          localization_accum_pos = 0.0;
        }
      }
    } else {
      // 持续缺边诊断 (只由 loop 的低频路径输出; 传感器回调 timeout=0 只快速
      // 探活, 不在这里阻塞)。边连续 ≥2s 不可用且距上次诊断 ≥5s 时提示一次,
      // 并给出最常见的环境根因 (多个仿真实例并发发布同名 TF / 发布者缺席)。
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
            "TF 同步缺边 {} <- {} 持续 ≥2s: 障碍落图与机器人位姿将异常。"
            "检查 gazebo 差速 (odom->base_footprint)、AMCL (map->odom)、"
            "robot_state_publisher 是否存活; 若曾多次 roslaunch, 残留的多个"
            "仿真实例会同时发布同名 TF 造成帧缺失/乱序",
            target, source);
      }
    }
  }
}

void AyanamiROS::checkTfHealth() {
  // 低频健康诊断: 每条 sync 边在 ROS TF 侧是否可解析; 持续缺失时每 5s
  // 输出一次 (附完整帧树), 恢复后输出一次 info。用于把"ROS 侧 TF 树缺帧"
  // 与"核心侧查询失败"区分开 —— 缺边时直接打印帧树, 一眼定位是哪个生产者
  // 缺席或残留多实例争抢。
  const auto now = std::chrono::steady_clock::now();
  std::string missing_edges;
  for (const auto& edge : tf_listener.tfSyncEdges()) {
    const std::string& target = edge.first;
    const std::string& source = edge.second;
    if (!tf_listener.canTransform(target, source, 0.0)) {
      if (!missing_edges.empty()) missing_edges += ", ";
      missing_edges += target + "<-" + source;
    }
  }
  if (missing_edges.empty()) {
    if (tf_health_was_bad) {
      spdlog::info("TF 健康诊断: 全部 {} 条同步边已恢复可用",
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
        "ROS TF 健康诊断: 边不可用 [{}]。当前帧树:\n{}\n"
        "若 odom<-base_footprint 缺失: 检查 gazebo 差速插件 (libgazebo_ros_"
        "diff_drive, publishOdomTF) 是否随模型加载; 若 map<-odom 缺失: 等待/"
        "检查 AMCL 定位; 多个仿真实例残留会同时发布同名 TF 造成帧缺失与乱序",
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
