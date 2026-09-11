#include <ayanami_ros/tf2_listener.hpp>

#include <chrono>
#include <thread>

#include <tf2_eigen/tf2_eigen.h>

namespace ayanami_ros {

namespace {
constexpr auto k_poll_granularity = std::chrono::milliseconds(5);

/// @brief Bounded steady-clock wait for a transform to appear. tf2's own
/// wait is clocked by ros::Time and stalls forever when /clock stops
/// advancing (sim-time hiccups), so the timeout is a wall-clock poll.
bool waitForCanTransform(const tf2_ros::Buffer& buffer,
                         const std::string& target, const std::string& source,
                         const ros::Time& stamp, double timeout) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(timeout);
  while (true) {
    if (buffer.canTransform(target, source, stamp)) {
      return true;
    }
    if (timeout <= 0.0 || std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(k_poll_granularity);
  }
}
}  // namespace

TF2Listener::TF2Listener(ros::NodeHandle& nh) {
  (void)nh;
  // spin_thread=true (the TransformListener default) feeds the buffer from a
  // separate thread; lookups from foreign threads require the buffer to be
  // marked as having a dedicated thread (tf2_ros threading note).
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
    if (!waitForCanTransform(buffer, target, source, stamp, timeout)) {
      return false;
    }
    out = buffer.lookupTransform(target, source, stamp);
    return true;
  } catch (const tf2::TransformException& e) {
    ROS_WARN_THROTTLE(1.0, "TF2 lookup failed (%s <- %s): %s", target.c_str(), source.c_str(), e.what());
    return false;
  }
}

bool TF2Listener::canTransform(const std::string& target,
                              const std::string& source,
                              double timeout) const {
  try {
    return waitForCanTransform(buffer, target, source, ros::Time(0), timeout);
  } catch (const tf2::TransformException&) {
    return false;
  }
}

std::string TF2Listener::framesAsString() const {
  try {
    return buffer.allFramesAsString();
  } catch (const std::exception& e) {
    return std::string("(frames unavailable: ") + e.what() + ")";
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
