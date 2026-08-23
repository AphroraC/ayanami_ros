// Task 7: TF2Listener TDD 测试。
//
// 两条用例:
//   1. 无 roscore / 空 buffer: buffer_.lookupTransform 抛 tf2::TransformException
//      → TF2Listener::lookup_transform 返回 false (catch 分支)。
//   2. 真 roscore: 用 tf2_ros::StaticBroadcaster 发布一个静态 transform, 等待
//      latched /tf_static 送达 listener 的 dedicated thread 后 lookup, 验证
//      返回非零、非单位阵的 Eigen::Isometry3d。无 master 时 GTEST_SKIP。

#include <Eigen/Geometry>

#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_broadcaster.h>

#include <ayanami_ros/tf2_listener.hpp>

namespace {

TEST(TF2Listener, LookupTransformEmptyReturnsFalse) {
  ros::NodeHandle nh("~");
  ayanami_ros::TF2Listener listener(
      nh, {{"map", "odom"}, {"odom", "base_link"}, {"map", "base_link"}});

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  // 无 roscore: buffer 恒空, lookupTransform(..., Duration(0)) 立即抛
  // tf2::TransformException (LookupException) → lookup_transform 吞异常返 false。
  EXPECT_FALSE(listener.lookup_transform("map", "odom", ros::Time(0), out));

  geometry_msgs::TransformStamped ts;
  EXPECT_FALSE(listener.lookup_transform("map", "odom", ros::Time(0), ts));
}

TEST(TF2Listener, LookupStaticBroadcastTransform) {
  if (!ros::master::check()) {
    GTEST_SKIP() << "requires roscore";
  }

  ros::NodeHandle nh("~");
  ayanami_ros::TF2Listener listener(nh, {{"map", "odom"}});

  // 动态 /tf 广播 (odom 意义上的真实 TF): header.stamp 标注该帧时刻。
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

  // 等待 /tf 送达 TransformListener 的 dedicated thread 并写入 buffer。
  ros::WallDuration(1.0).sleep();

  // TransformStamped 重载: 一块拿回 TF2 buffer 的真实 header.stamp (供
  // TransformStore 写入, 不再用 ros::Time::now() 伪造)。Time(0)=latest →
  // 返回最新动态帧, stamp 应为发布时刻。
  geometry_msgs::TransformStamped ts_out;
  ASSERT_TRUE(
      listener.lookup_transform("map", "odom", ros::Time(0), ts_out, 2.0));
  EXPECT_NEAR(ts_out.header.stamp.toSec(), ts.header.stamp.toSec(), 0.1);
  EXPECT_NEAR(ts_out.transform.translation.x, 1.0, 1e-6);

  // Eigen 重载仍可用, 且平移 == 发布的 (1, 2, 3)。
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  ASSERT_TRUE(listener.lookup_transform("map", "odom", ros::Time(0), out, 2.0));
  // 验证非零非单位阵, 且平移 == 发布的 (1, 2, 3)。
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
