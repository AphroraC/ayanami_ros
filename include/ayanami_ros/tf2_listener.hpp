#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ayanami_ros {

/**
 * @brief TF2 封装: tf2_ros::Buffer + TransformListener (spec §7 verbatim)。
 *
 * 构造时建 buffer_ + listener_ (spin_thread=true 起专用线程填 buffer), 保存
 * tf_sync_edges_ (由 config_ros.json 控, 默认 [map→odom, odom→base_link,
 * map→base_link])。lookup_transform 内部 lookupTransform(target, source,
 * stamp, ros::Duration(timeout)) → tf2::transformToEigen → Eigen::Isometry3d,
 * 抛 tf2::TransformException 时返回 false。
 */
class TF2Listener {
public:
  TF2Listener(ros::NodeHandle& nh,
              const std::vector<std::pair<std::string, std::string>>& edges);

  /// @brief 查 source→target 变换; stamp=ros::Time(0) 表示 latest。
  /// @return true 填 @p out; false (timeout 秒内无可用 TF)。
  bool lookup_transform(const std::string& target, const std::string& source,
                        const ros::Time& stamp, Eigen::Isometry3d& out,
                        double timeout = 0.0);

  /// @brief 配置的 tf sync 边集 (target, source) 列表。
  const std::vector<std::pair<std::string, std::string>>& tfSyncEdges() const {
    return tf_sync_edges_;
  }

private:
  tf2_ros::Buffer buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::vector<std::pair<std::string, std::string>> tf_sync_edges_;
};

}  // namespace ayanami_ros
