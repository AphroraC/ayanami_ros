// TF2Listener TDD tests.
//
// Two cases:
// 1. No roscore / empty buffer: buffer.lookupTransform throws
// tf2::TransformException -> TF2Listener::lookupTransform returns false (catch
// branch).
// 2. Real roscore: publish a static transform with tf2_ros::StaticBroadcaster,
// wait for the latched /tf_static to reach the listener's dedicated thread,
// then look up and verify a non-zero, non-identity Eigen::Isometry3d is
// returned. GTEST_SKIP when there is no master.

#include <Eigen/Geometry>

#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_broadcaster.h>

#include <ayanami_ros/tf2_listener.hpp>

namespace {

TEST(TF2Listener, lookupTransformEmptyReturnsFalse) {
  ros::NodeHandle nh("~");
  ayanami_ros::TF2Listener listener(nh);

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  // No roscore: the buffer stays empty; lookupTransform(..., Duration(0))
  // immediately throws tf2::TransformException (LookupException) ->
  // lookupTransform swallows the exception and returns false.
  EXPECT_FALSE(listener.lookupTransform("map", "odom", ros::Time(0), out));

  geometry_msgs::TransformStamped ts;
  EXPECT_FALSE(listener.lookupTransform("map", "odom", ros::Time(0), ts));
}

TEST(TF2Listener, lookupStaticBroadcastTransform) {
  if (!ros::master::check()) {
    GTEST_SKIP() << "requires roscore";
  }

  ros::NodeHandle nh("~");
  ayanami_ros::TF2Listener listener(nh);

  // Dynamic /tf broadcast (a real TF in the odom sense): header.stamp marks
  // the frame's time.
  tf2_ros::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.header.frame_id = "map";
  ts.child_frame_id = "odom";
  ts.transform.translation.x = 1.0;
  ts.transform.translation.y = 2.0;
  ts.transform.translation.z = 3.0;
  ts.transform.rotation.x = 0.0;
  ts.transform.rotation.y = 0.0;
  ts.transform.rotation.z = 0.0;
  ts.transform.rotation.w = 1.0;
  broadcaster.sendTransform(ts);

  // Wait for /tf to reach the TransformListener's dedicated thread and be
  // written into the buffer.
  ros::WallDuration(1.0).sleep();

  // TransformStamped overload: also fetches back the TF2 buffer's real
  // header.stamp (for TransformStore writes, no longer faked with
  // ros::Time::now). Time(0)=latest -> returns the newest dynamic frame; the
  // stamp should be the publication time.
  geometry_msgs::TransformStamped ts_out;
  ASSERT_TRUE(
      listener.lookupTransform("map", "odom", ros::Time(0), ts_out, 2.0));
  EXPECT_NEAR(ts_out.header.stamp.toSec(), ts.header.stamp.toSec(), 0.1);
  EXPECT_NEAR(ts_out.transform.translation.x, 1.0, 1e-6);

  // The Eigen overload still works, and the translation equals the published
  // (1, 2, 3).
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  ASSERT_TRUE(listener.lookupTransform("map", "odom", ros::Time(0), out, 2.0));
  // Verify non-zero, non-identity, and translation equal to the published
  // (1, 2, 3).
  EXPECT_FALSE(out.isApprox(Eigen::Isometry3d::Identity(), 1e-6));
  EXPECT_GT(out.translation().norm(), 0.0);
  EXPECT_NEAR(out.translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(out.translation().y(), 2.0, 1e-6);
  EXPECT_NEAR(out.translation().z(), 3.0, 1e-6);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tf2_listener_test",
            ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
  ros::master::setRetryTimeout(ros::WallDuration(0.3));
  return RUN_ALL_TESTS();
}
