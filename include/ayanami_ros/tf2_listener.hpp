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
 * @brief TF2 wrapper: tf2_ros::Buffer + TransformListener (verbatim).
 *
 * The constructor creates the buffer + listener (spin_thread=true starts a
 * dedicated thread feeding the buffer) and stores tf_sync_edges (controlled by
 * config_ros.json, default [map->odom, odom->base_link, map->base_link]).
 * lookupTransform internally does lookupTransform(target, source, stamp,
 * ros::Duration(timeout)) -> tf2::transformToEigen -> Eigen::Isometry3d, and
 * returns false when tf2::TransformException is thrown.
 *
 * Time semantics: stamp=ros::Time(0) means latest; the TransformStamped
 * overload also returns the real geometry_msgs::TransformStamped.header.stamp
 * provided by the buffer, so writing into the TransformStore does not need to
 * fake it with ros::Time::now().
 */
class TF2Listener {
public:
  TF2Listener(ros::NodeHandle& nh,
              const std::vector<std::pair<std::string, std::string>>& edges);

  /// @brief Looks up the source->target transform; stamp=ros::Time(0) means
  /// latest.
  /// @return true fills @p out (the real header.stamp is returned as well),
  /// false (no TF available within timeout seconds).
  bool lookupTransform(const std::string& target, const std::string& source,
                        const ros::Time& stamp,
                        geometry_msgs::TransformStamped& out,
                        double timeout = 0.0);

  /// @brief Eigen version of the above (delegates internally to the
  /// TransformStamped overload).
  /// @return true fills @p out, false (no TF available within timeout seconds).
  bool lookupTransform(const std::string& target, const std::string& source,
                        const ros::Time& stamp, Eigen::Isometry3d& out,
                        double timeout = 0.0);

  /// @brief List of configured tf sync edges (target, source).
  const std::vector<std::pair<std::string, std::string>>& tfSyncEdges() const {
    return tf_sync_edges;
  }

  /// @brief Synchronously injects one /tf transform into the internal tf2
  /// buffer.
  ///
  /// TransformListener inserts into the buffer asynchronously (separate
  /// thread), so the event-driven /tf callback may run before that insertion --
  /// the lookup would read a stale sample and skip that frame forever. The
  /// callback first writes the message into the buffer through this method and
  /// only then queries, removing the race (a duplicate insertion of the same
  /// value after the Listener is idempotent).
  void setTransform(const geometry_msgs::TransformStamped& transform) {
    buffer.setTransform(transform, "ayanami_event_sync", /*is_static=*/false);
  }

  /// @brief ROS TF tree health diagnostics (missing-edge troubleshooting):
  /// whether edge (target<-source) is currently resolvable.
  bool canTransform(const std::string& target, const std::string& source,
                    double timeout = 0.0) const;

  /// @brief List of frames currently present in the ROS TF tree (with
  /// parent-child relations), format:
  /// "Frame base_link exists with parent base_footprint".
  std::string framesAsString() const;

private:
  tf2_ros::Buffer buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  std::vector<std::pair<std::string, std::string>> tf_sync_edges;
};

}  // namespace ayanami_ros
