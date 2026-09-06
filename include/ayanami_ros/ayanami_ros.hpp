#pragma once

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Geometry>

#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/sensor_msgs.hpp>
#include <ayanami/messages/timestamp.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <ayanami_ros/tf2_listener.hpp>

namespace ayanami_ros {

/**
 * @brief ayanami ROS1 边界宿主节点 (-§4.2)。
 *
 * 职责: 加载核心 AbstractNavigation 宿主插件 (dlopen
 * libayanami_costmap_navigation.so)
 * + 加载 ayanami::ExtensionModule 扩展 (rviz_viewer 等), 把 ROS topics → 核心
 * callback 转换, tf sync 与 spinOnce 周期驱动。
 *
 * 关键顺序 (verbatim): loadExtensionModules → loadNavigationPlugin →
 * subscribeTopics → loop_thread 启。扩展必须先于 navigation_plugin->start()
 * 注册 NavigationCallbacks 订阅, 否则启动期第一波可视化 emit 会被错过 (
 * ruling)。
 */
class AyanamiROS {
public:
  explicit AyanamiROS(ros::NodeHandle& nh);
  ~AyanamiROS();

  void start();
  void stop();

  // ---- 测试/宿主支撑 getter (brief: getter for tests) ----
  const std::vector<ayanami::ExtensionModule::Ptr>& extensionModules() const { return extension_modules; }
  ayanami::abstract::AbstractNavigation::Ptr navigationPlugin() const { return navigation_plugin; }
  const std::string& configPath() const { return config_path; }

  // ---- 输入接线 (核心边界, 供 ROS 回调 / loop / 测试) ----
  /// @brief TF 边写入: ROS 侧查到的 TransformStamped 直写核心
  /// RobotStateController (核心唯一的 tf 查询供应商)。
  void setTransform(const std::string& parent, const std::string& child,
                    ayanami::Timestamp stamp,
                    const Eigen::Isometry3d& transform, bool is_static);
  /// @brief odom 速度直写核心 RobotStateController (位姿读取仍由核心执行层
  /// 经 TF 查询)。
  void setOdometry(const ayanami::geometry_msgs::TwistStamped& twist);
  /// @brief 障碍观测输入: 点云 (先同步当前可用的 TF 边, 再直递导航宿主)。
  void submitSensor(const ayanami::sensor_msgs::PointCloud2& cloud);
  /// @brief 障碍观测输入: 激光扫描 (同上)。
  void submitSensor(const ayanami::sensor_msgs::LaserScan& scan);
  /// @brief latest-wins goal 邮箱投递 (ROS goalCallback 转换后调用; 覆写待发
  /// goal, 不排队)。
  void submitGoal(const ayanami::geometry_msgs::PoseStamped& goal);
  /// @brief loop 线程侧: 取当前最新 goal (若有) 调 navigation_plugin->setGoal。
  /// @return true 表示确实下发了一个 goal。
  bool drainPendingGoal();
  /// @brief 测试/观察: 邮箱内是否还有待发 goal。
  bool hasPendingGoal() const;

private:
  void loadExtensionModules(const ayanami::Config& config_ros);
  void loadNavigationPlugin(const ayanami::Config& config_ros);
  void subscribeTopics();
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void syncTransformsToCore();
  void syncTransformsToCore(const ros::Time& stamp, double timeout);
  /// @brief ROS TF 树健康诊断 (loop 低频调用): 检查 tf_sync_edges 各边在 ROS
  /// 侧是否可解析; 缺失时输出一次带完整帧树的警告 (定位环境/残留实例问题)。
  void checkTfHealth();
  void loop();

private:
  ros::NodeHandle& nh;
  std::string config_path;

  ayanami::abstract::AbstractNavigation::Ptr navigation_plugin;
  // 扩展模块统一继承 ayanami::ExtensionModule (无 ROS 特有子类): 输入归
  // AyanamiROS, 输出发布由 rviz_viewer 之类的扩展自身在构造期完成。
  std::vector<ayanami::ExtensionModule::Ptr> extension_modules;

  ayanami_ros::TF2Listener tf_listener;

  ros::Subscriber goal_sub, pointcloud_sub, odom_sub, laserscan_sub;

  std::atomic<bool> kill_switch{false};
  std::thread loop_thread;
  // latest-wins goal 邮箱 : ROS 回调只转换 + 覆写 pending_goal,
  // loop 线程每次取当前最新一条下发 setGoal (不排队, 不丢未处理)。
  mutable std::mutex goal_mutex;
  std::optional<ayanami::geometry_msgs::PoseStamped> pending_goal;
  std::mutex tf_edge_mutex;
  std::map<std::pair<std::string, std::string>, bool> tf_edge_static;
  // TF 缺边诊断 (50Hz loop 维护): 边首次失败时间 + 上次诊断输出时间。
  // 传感器回调路径不阻塞 (timeout=0), 诊断只由 loop 的低频路径输出。
  std::map<std::pair<std::string, std::string>,
           std::chrono::steady_clock::time_point>
      tf_missing_since;
  std::map<std::pair<std::string, std::string>,
           std::chrono::steady_clock::time_point>
      tf_diag_logged_at;
  // TF 健康诊断状态 (checkTfHealth 使用)。
  bool tf_health_was_bad{false};
  std::chrono::steady_clock::time_point tf_health_last_log{};
  std::set<std::pair<std::string, std::string>> configured_static_edges;
  double tf_sync_rate = 50.0;

  // config_ros.json::ayanami_ros 边界参数 。
  std::string laserscan_topic, pointcloud_topic, odom_topic, goal_topic;
  std::string map_frame, odom_frame, robot_frame;
  // 全局动态代价图重建触发 (odom<-base_footprint 边): 累计"自上次清空以来"
  // 的机器人旋转/平移。旋转是标记错位的主源 —— 轮式里程计在原地旋转时打滑
  // 欠报转角 (~6%/rad 实测), 而 AMCL 修正以有限速率追赶, 期间 scan 按滞后
  // 变换落图, 全局图动态标记逐渐错位; 累计旋转/平移超阈值即清空重建。
  // 注意: 不能用 map<-odom 修正增量做信号 —— AMCL 修正渐进且可能长期不收敛
  // (恢复注入关闭时永不收敛), 修正增量恒为小值, 检测不到错位积累。
  bool localization_jump_prev_valid{false};
  double localization_clear_yaw_threshold{1.0};  // rad, 累计自上次清空
  double localization_clear_pos_threshold{2.0};  // m, 累计自上次清空
  double localization_accum_yaw{0.0};
  double localization_accum_pos{0.0};
  Eigen::Isometry3d localization_jump_prev{Eigen::Isometry3d::Identity()};
  std::mutex localization_jump_mutex;
  std::string navigation_so_name;
};

}  // namespace ayanami_ros
