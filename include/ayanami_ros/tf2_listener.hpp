#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ayanami_ros {

/**
 * @brief TF2 封装: tf2_ros::Buffer + TransformListener (verbatim)。
 *
 * 构造时建 buffer + listener (spin_thread=true 起专用线程填 buffer), 保存
 * tf_sync_edges (由 config_ros.json 控, 默认 [map→odom, odom→base_link,
 * map→base_link])。lookupTransform 内部 lookupTransform(target, source,
 * stamp, ros::Duration(timeout)) → tf2::transformToEigen → Eigen::Isometry3d,
 * 抛 tf2::TransformException 时返回 false。
 *
 * 时间语义: stamp=ros::Time(0) 表示 latest; TransformStamped 重载同时返回
 * buffer 给出的真实 geometry_msgs::TransformStamped.header.stamp, 供写入
 * TransformStore 时不用 ros::Time::now() 伪造。
 */
class TF2Listener {
public:
  TF2Listener(ros::NodeHandle& nh,
              const std::vector<std::pair<std::string, std::string>>& edges);

  /// @brief 查 source→target 变换; stamp=ros::Time(0) 表示 latest。
  /// @return true 填 @p out (真实 header.stamp 一并返回), false (timeout 秒内无
  /// 可用 TF)。
  bool lookupTransform(const std::string& target, const std::string& source,
                        const ros::Time& stamp,
                        geometry_msgs::TransformStamped& out,
                        double timeout = 0.0);

  /// @brief 同上的 Eigen 版本 (内部委托 TransformStamped 重载)。
  /// @return true 填 @p out, false (timeout 秒内无可用 TF)。
  bool lookupTransform(const std::string& target, const std::string& source,
                        const ros::Time& stamp, Eigen::Isometry3d& out,
                        double timeout = 0.0);

  /// @brief 配置的 tf sync 边集 (target, source) 列表。
  const std::vector<std::pair<std::string, std::string>>& tfSyncEdges() const {
    return tf_sync_edges;
  }

  /// @brief ROS TF 树健康诊断 (缺边排障): 边 (target←source) 当前是否可解析。
  bool canTransform(const std::string& target, const std::string& source,
                    double timeout = 0.0) const;

  /// @brief ROS TF 树中现有帧列表 (含父子关系), 格式:
  /// "Frame base_link exists with parent base_footprint"。
  std::string framesAsString() const;

private:
  tf2_ros::Buffer buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  std::vector<std::pair<std::string, std::string>> tf_sync_edges;
};

}  // namespace ayanami_ros
