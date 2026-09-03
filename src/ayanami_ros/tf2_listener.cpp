#include <ayanami_ros/tf2_listener.hpp>

#include <tf2_eigen/tf2_eigen.h>

namespace ayanami_ros {

TF2Listener::TF2Listener(ros::NodeHandle& nh, const std::vector<std::pair<std::string, std::string>>& edges)
: tf_sync_edges(edges) {
  (void)nh;
  // spin_thread=true (TransformListener 默认) 在独立线程填 buffer;
  // 带 timeout 的 lookup 要求 buffer 标记 dedicated thread (tf2_ros 线程注记)。
  buffer.setUsingDedicatedThread(true);
  listener = std::make_unique<tf2_ros::TransformListener>(buffer);
}

bool TF2Listener::lookupTransform(
  const std::string& target,
  const std::string& source,
  const ros::Time& stamp,
  geometry_msgs::TransformStamped& out,
  double timeout) {
  try {
    // During startup AMCL and Gazebo do not publish all frames at once. A
    // canTransform probe avoids turning that expected warm-up into a warning
    // from lookupTransform while retaining the bounded wait semantics.
    if (!buffer.canTransform(target, source, stamp, ros::Duration(timeout))) {
      return false;
    }
    out = buffer.lookupTransform(target, source, stamp, ros::Duration(timeout));
    return true;
  } catch (const tf2::TransformException& e) {
    ROS_WARN_THROTTLE(1.0, "TF2 lookup failed (%s <- %s): %s", target.c_str(), source.c_str(), e.what());
    return false;
  }
}

bool TF2Listener::lookupTransform(
  const std::string& target,
  const std::string& source,
  const ros::Time& stamp,
  Eigen::Isometry3d& out,
  double timeout) {
  geometry_msgs::TransformStamped tf_msg;
  if (!lookupTransform(target, source, stamp, tf_msg, timeout)) {
    return false;
  }
  out = tf2::transformToEigen(tf_msg);
  return true;
}

}  // namespace ayanami_ros
