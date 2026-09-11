#pragma once

#include <string>

#include <Eigen/Geometry>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ayanami_ros {

/**
 * @brief TF2 wrapper: tf2_ros::Buffer + TransformListener.
 *
 * The constructor creates the buffer + listener (spin_thread=true starts a
 * dedicated thread feeding the buffer). The auto-subscription is the ONLY
 * insertion path -- every /tf and /tf_static message lands in the buffer
 * exactly once. This object is installed into the core
 * RobotStateController as the injected TF source: core threads query it
 * on demand (tf2 interpolation + static edges included), so no periodic
 * sampling into a core-side store exists anymore.
 *
 * Timeout semantics: implemented as a steady-clock POLLING loop over
 * canTransform (never tf2's own ros::Time-clocked wait, which stalls
 * forever when /clock stops advancing); each probe is a zero-timeout
 * lookup, so a warm-up gap costs at most one 5 ms poll.
 */
class TF2Listener {
public:
  explicit TF2Listener(ros::NodeHandle& nh);

  /// @brief Looks up the source->target transform; stamp=ros::Time(0) means
  /// latest.
  /// @return true fills @p out, false (no TF available within timeout
  /// seconds; timeout <= 0 = single zero-timeout probe).
  bool lookupTransform(const std::string& target, const std::string& source,
                        const ros::Time& stamp,
                        geometry_msgs::TransformStamped& out,
                        double timeout = 0.0);

  /// @brief Eigen version of the above (delegates internally to the
  /// TransformStamped overload).
  bool lookupTransform(const std::string& target, const std::string& source,
                        const ros::Time& stamp, Eigen::Isometry3d& out,
                        double timeout = 0.0);

  /// @brief ROS TF tree health diagnostics (missing-edge troubleshooting):
  /// whether edge (target<-source) is currently resolvable (latest).
  bool canTransform(const std::string& target, const std::string& source,
                    double timeout = 0.0) const;

  /// @brief List of frames currently present in the ROS TF tree (with
  /// parent-child relations), format:
  /// "Frame base_link exists with parent base_footprint".
  std::string framesAsString() const;

private:
  tf2_ros::Buffer buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
};

}  // namespace ayanami_ros
