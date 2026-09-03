// Task 8: ros_msg_converter TDD 测试 (spec §6 Eigen ↔ ROS msgs)。
//
// 用例覆盖 6+ 类 converter:
//   1. PoseStamped round-trip (ROS→core→ROS 反查 frame_id / translation 一致)
//   2. TwistStamped round-trip
//   3. Path → ROS (多 pose, 逐 pose 经 fromRosPose 反查一致)
//   4. OccupancyGrid → ROS 基本字段 (width/height/resolution/origin/data)
//   5. LaserScan ← ROS 基本字段
//   6. PointCloud2 简单转 + round-trip (height=1, width=10, point_step=12 xyz)

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Dense>

#include <ayanami_ros/ros_msg_converter.hpp>

namespace {

constexpr double kTol = 1e-6;

ayanami::geometry_msgs::Pose makePose(double x, double y, double z, double yaw) {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translation() = Eigen::Vector3d(x, y, z);
  iso.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return ayanami::geometry_msgs::Pose::fromIsometry(iso);
}

TEST(RosMsgConverter, poseStampedRosToCoreToRosRoundTrip) {
  geometry_msgs::PoseStamped ros_in;
  ros_in.header.frame_id = "map";
  ros_in.header.stamp = ros::Time(10.0);
  ros_in.pose.position.x = 1.0;
  ros_in.pose.position.y = 2.0;
  ros_in.pose.position.z = 3.0;
  const Eigen::Quaterniond q(
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  ros_in.pose.orientation.x = q.x();
  ros_in.pose.orientation.y = q.y();
  ros_in.pose.orientation.z = q.z();
  ros_in.pose.orientation.w = q.w();

  const auto core = ayanami_ros::fromRosPose(ros_in);
  EXPECT_EQ(core.header.frame_id, "map");
  EXPECT_EQ(core.header.stamp.count(), 10000000000LL);
  EXPECT_NEAR(core.header.stamp.toSeconds(), 10.0, kTol);
  EXPECT_NEAR(core.pose.position.x(), 1.0, kTol);
  EXPECT_NEAR(core.pose.position.y(), 2.0, kTol);
  EXPECT_NEAR(core.pose.position.z(), 3.0, kTol);
  EXPECT_NEAR(core.pose.orientation.norm(), 1.0, kTol);

  const auto ros_out = ayanami_ros::toRosPose(core);
  EXPECT_EQ(ros_out.header.frame_id, "map");
  EXPECT_EQ(ros_out.header.stamp, ros::Time(10.0));
  EXPECT_NEAR(ros_out.pose.position.x, 1.0, kTol);
  EXPECT_NEAR(ros_out.pose.position.y, 2.0, kTol);
  EXPECT_NEAR(ros_out.pose.position.z, 3.0, kTol);
  EXPECT_NEAR(ros_out.pose.orientation.w, q.w(), kTol);
  EXPECT_NEAR(ros_out.pose.orientation.z, q.z(), kTol);
}

TEST(RosMsgConverter, poseStampedCoreToRosToCoreRoundTrip) {
  ayanami::geometry_msgs::PoseStamped core_in =
      ayanami::geometry_msgs::PoseStamped::fromIsometry(
          "odom", makePose(0.5, -1.5, 2.0, M_PI).toIsometry());

  const auto ros = ayanami_ros::toRosPose(core_in);
  EXPECT_EQ(ros.header.frame_id, "odom");
  EXPECT_NEAR(ros.pose.position.x, 0.5, kTol);
  EXPECT_NEAR(ros.pose.position.y, -1.5, kTol);
  EXPECT_NEAR(ros.pose.position.z, 2.0, kTol);

  const auto core_out = ayanami_ros::fromRosPose(ros);
  EXPECT_EQ(core_out.header.frame_id, "odom");
  EXPECT_TRUE(core_out.pose.toIsometry().matrix().isApprox(core_in.pose.toIsometry().matrix(), kTol));
}

TEST(RosMsgConverter, twistStampedRoundTrip) {
  geometry_msgs::TwistStamped ros_in;
  ros_in.header.frame_id = "base_link";
  ros_in.header.stamp = ros::Time(3.5);
  ros_in.twist.linear.x = 1.0;
  ros_in.twist.linear.y = -0.5;
  ros_in.twist.linear.z = 0.0;
  ros_in.twist.angular.x = 0.0;
  ros_in.twist.angular.y = 0.0;
  ros_in.twist.angular.z = 0.3;

  const auto core = ayanami_ros::fromRosTwist(ros_in);
  EXPECT_EQ(core.header.frame_id, "base_link");
  EXPECT_EQ(core.header.stamp.count(), 3500000000LL);
  EXPECT_NEAR(core.twist.linear.x(), 1.0, kTol);
  EXPECT_NEAR(core.twist.linear.y(), -0.5, kTol);
  EXPECT_NEAR(core.twist.angular.z(), 0.3, kTol);

  const auto ros_out = ayanami_ros::toRosTwist(core);
  EXPECT_EQ(ros_out.header.frame_id, "base_link");
  EXPECT_EQ(ros_out.header.stamp, ros::Time(3.5));
  EXPECT_NEAR(ros_out.twist.linear.x, 1.0, kTol);
  EXPECT_NEAR(ros_out.twist.linear.y, -0.5, kTol);
  EXPECT_NEAR(ros_out.twist.linear.z, 0.0, kTol);
  EXPECT_NEAR(ros_out.twist.angular.z, 0.3, kTol);
}

TEST(RosMsgConverter, pathToRosMultiPose) {
  ayanami::nav_msgs::Path core_in;
  core_in.header.frame_id = "map";
  core_in.poses.push_back(ayanami::geometry_msgs::PoseStamped::fromIsometry(
      "map", makePose(0.0, 0.0, 0.0, 0.0).toIsometry()));
  core_in.poses.push_back(ayanami::geometry_msgs::PoseStamped::fromIsometry(
      "map", makePose(1.0, 0.0, 0.0, M_PI / 4.0).toIsometry()));
  core_in.poses.push_back(ayanami::geometry_msgs::PoseStamped::fromIsometry(
      "map", makePose(2.0, 1.0, 0.0, M_PI / 2.0).toIsometry()));

  const auto ros = ayanami_ros::toRosPath(core_in);
  EXPECT_EQ(ros.header.frame_id, "map");
  ASSERT_EQ(ros.poses.size(), core_in.poses.size());
  for (size_t i = 0; i < ros.poses.size(); ++i) {
    // Path 无 fromRosPath 反向接口, 逐 pose 经 fromRosPose 反查一致。
    const auto back = ayanami_ros::fromRosPose(ros.poses[i]);
    EXPECT_TRUE(back.pose.toIsometry().matrix().isApprox(
        core_in.poses[i].pose.toIsometry().matrix(), kTol))
        << "pose " << i;
  }
}

TEST(RosMsgConverter, occupancyGridToRosBasicFields) {
  ayanami::nav_msgs::OccupancyGrid core_in;
  core_in.header.frame_id = "map";
  core_in.info.resolution = 0.05;
  core_in.info.width = 40;
  core_in.info.height = 30;
  core_in.info.origin = makePose(-1.0, -2.0, 0.0, 0.0);
  core_in.data = {0, 100, -1, 50, 25};

  const auto ros = ayanami_ros::toRosOccupancyGrid(core_in);
  EXPECT_EQ(ros.header.frame_id, "map");
  EXPECT_NEAR(ros.info.resolution, 0.05, kTol);
  EXPECT_EQ(ros.info.width, 40u);
  EXPECT_EQ(ros.info.height, 30u);
  EXPECT_NEAR(ros.info.origin.position.x, -1.0, kTol);
  EXPECT_NEAR(ros.info.origin.position.y, -2.0, kTol);
  EXPECT_NEAR(ros.info.origin.position.z, 0.0, kTol);
  ASSERT_EQ(ros.data.size(), core_in.data.size());
  EXPECT_EQ(ros.data[0], 0);
  EXPECT_EQ(ros.data[1], 100);
  EXPECT_EQ(ros.data[2], -1);
}

TEST(RosMsgConverter, laserScanFromRosBasicFields) {
  sensor_msgs::LaserScan ros_in;
  ros_in.header.frame_id = "laser";
  ros_in.header.stamp = ros::Time(2.0);
  ros_in.angle_min = -1.5707963f;
  ros_in.angle_max = 1.5707963f;
  ros_in.angle_increment = 0.0174533f;
  ros_in.time_increment = 0.0001f;
  ros_in.scan_time = 0.1f;
  ros_in.range_min = 0.1f;
  ros_in.range_max = 30.0f;
  ros_in.ranges = {1.0f, 2.5f, 0.3f};
  ros_in.intensities = {10.0f, 20.0f};

  const auto core = ayanami_ros::fromRosLaserScan(ros_in);
  EXPECT_EQ(core.header.frame_id, "laser");
  EXPECT_EQ(core.header.stamp.count(), 2000000000LL);
  EXPECT_FLOAT_EQ(core.angle_min, -1.5707963f);
  EXPECT_FLOAT_EQ(core.angle_max, 1.5707963f);
  EXPECT_FLOAT_EQ(core.angle_increment, 0.0174533f);
  EXPECT_FLOAT_EQ(core.time_increment, 0.0001f);
  EXPECT_FLOAT_EQ(core.scan_time, 0.1f);
  EXPECT_FLOAT_EQ(core.range_min, 0.1f);
  EXPECT_FLOAT_EQ(core.range_max, 30.0f);
  ASSERT_EQ(core.ranges.size(), 3u);
  EXPECT_FLOAT_EQ(core.ranges[0], 1.0f);
  ASSERT_EQ(core.intensities.size(), 2u);
  EXPECT_FLOAT_EQ(core.intensities[0], 10.0f);
}

TEST(RosMsgConverter, pointCloud2FromRosFields) {
  sensor_msgs::PointCloud2 ros_in;
  ros_in.header.frame_id = "lidar";
  ros_in.header.stamp = ros::Time(4.25);
  ros_in.height = 1;
  ros_in.width = 10;
  sensor_msgs::PointField fx, fy, fz;
  fx.name = "x";
  fx.offset = 0;
  fx.datatype = sensor_msgs::PointField::FLOAT32;
  fx.count = 1;
  fy.name = "y";
  fy.offset = 4;
  fy.datatype = sensor_msgs::PointField::FLOAT32;
  fy.count = 1;
  fz.name = "z";
  fz.offset = 8;
  fz.datatype = sensor_msgs::PointField::FLOAT32;
  fz.count = 1;
  ros_in.fields = {fx, fy, fz};
  ros_in.is_bigendian = false;
  ros_in.point_step = 12;
  ros_in.row_step = 120;
  ros_in.data.assign(ros_in.row_step, 0);
  ros_in.is_dense = true;

  const auto core = ayanami_ros::fromRosPointCloud2(ros_in);
  EXPECT_EQ(core.header.frame_id, "lidar");
  EXPECT_EQ(core.header.stamp.count(), 4250000000LL);
  EXPECT_EQ(core.height, 1u);
  EXPECT_EQ(core.width, 10u);
  ASSERT_EQ(core.fields.size(), 3u);
  EXPECT_EQ(core.fields[0].name, "x");
  EXPECT_EQ(core.fields[0].offset, 0u);
  EXPECT_EQ(core.fields[0].datatype, ayanami::sensor_msgs::PointField::FLOAT32);
  EXPECT_EQ(core.fields[0].count, 1u);
  EXPECT_EQ(core.fields[2].name, "z");
  EXPECT_EQ(core.fields[2].offset, 8u);
  EXPECT_EQ(core.point_step, 12u);
  EXPECT_EQ(core.row_step, 120u);
  EXPECT_EQ(core.data.size(), ros_in.data.size());
  EXPECT_EQ(core.is_dense, true);
}

TEST(RosMsgConverter, pointCloud2RoundTrip) {
  sensor_msgs::PointCloud2 ros_in;
  ros_in.header.frame_id = "lidar";
  ros_in.height = 1;
  ros_in.width = 4;
  sensor_msgs::PointField fx;
  fx.name = "x";
  fx.offset = 0;
  fx.datatype = sensor_msgs::PointField::FLOAT32;
  fx.count = 1;
  ros_in.fields = {fx};
  ros_in.point_step = 4;
  ros_in.row_step = 16;
  ros_in.data = {0, 0, 0x80, 0x3f, 0, 0, 0, 0, 0, 0, 0x20, 0x40, 0, 0, 0, 0};

  const auto core = ayanami_ros::fromRosPointCloud2(ros_in);
  const auto ros_out = ayanami_ros::toRosPointCloud2(core);
  EXPECT_EQ(ros_out.header.frame_id, "lidar");
  EXPECT_EQ(ros_out.height, 1u);
  EXPECT_EQ(ros_out.width, 4u);
  ASSERT_EQ(ros_out.fields.size(), 1u);
  EXPECT_EQ(ros_out.fields[0].name, "x");
  EXPECT_EQ(ros_out.fields[0].offset, 0u);
  EXPECT_EQ(ros_out.point_step, 4u);
  EXPECT_EQ(ros_out.row_step, 16u);
  EXPECT_EQ(ros_out.data, ros_in.data);
  EXPECT_EQ(ros_out.is_dense, false);
}

TEST(RosMsgConverter, headerStampRoundTripThroughAllConverterKinds) {
  // Pose / Twist / LaserScan / PointCloud2 的 header.stamp 双向完整转换:
  // ros::Time ↔ ayanami::Timestamp (纳秒)。
  geometry_msgs::PoseStamped pose_in;
  pose_in.header.frame_id = "map";
  pose_in.header.stamp = ros::Time(123456.789);
  pose_in.pose.position.x = 1.0;
  const auto pose_core = ayanami_ros::fromRosPose(pose_in);
  EXPECT_EQ(pose_core.header.stamp.count(), 123456789000000LL);
  const auto pose_out = ayanami_ros::toRosPose(pose_core);
  EXPECT_EQ(pose_out.header.stamp, pose_in.header.stamp);

  sensor_msgs::PointCloud2 cloud_in;
  cloud_in.header.frame_id = "lidar";
  cloud_in.header.stamp = ros::Time(9.0);
  cloud_in.height = 1;
  cloud_in.width = 1;
  const auto cloud_core = ayanami_ros::fromRosPointCloud2(cloud_in);
  EXPECT_EQ(cloud_core.header.stamp.count(), 9000000000LL);
  const auto cloud_out = ayanami_ros::toRosPointCloud2(cloud_core);
  EXPECT_EQ(cloud_out.header.stamp, ros::Time(9.0));
}

TEST(RosMsgConverter, headerLatestSentinelMapsToRosTimeZero) {
  // ayanami 显式 latest 哨兵 (0ns) ↔ ros::Time(0) (TF latest 惯例)。
  ayanami::geometry_msgs::PoseStamped core_in =
      ayanami::geometry_msgs::PoseStamped::fromIsometry(
          "map", makePose(0.0, 0.0, 0.0, 0.0).toIsometry());
  ASSERT_TRUE(core_in.header.stamp.isLatest());

  const auto ros = ayanami_ros::toRosPose(core_in);
  EXPECT_EQ(ros.header.stamp, ros::Time(0));

  const auto core_back = ayanami_ros::fromRosPose(ros);
  EXPECT_TRUE(core_back.header.stamp.isLatest());
  EXPECT_EQ(core_back.header.stamp.count(), 0);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ros_msg_converter_test",
            ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
  ros::master::setRetryTimeout(ros::WallDuration(0.3));
  return RUN_ALL_TESTS();
}
