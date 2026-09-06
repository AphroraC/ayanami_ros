#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/messages/sensor_msgs.hpp>

namespace ayanami_ros {

// Eigen ↔ ROS msgs。header-only inline 实现 (轻便, 无需 .cpp)。
// Header stamp 双向完整转换: ros::Time ↔ ayanami::Timestamp (纳秒)。
// 语义对齐以 TF 通语义: ayanami::Timestamp::latest (0ns) ↔ ros::Time(0)。

namespace detail {

inline ayanami::Timestamp fromRosTime(const ros::Time& in) {
  return ayanami::Timestamp{static_cast<std::int64_t>(in.toNSec())};
}

inline ros::Time toRosTime(const ayanami::Timestamp& in) {
  if (in.isLatest()) {
    return ros::Time(0);
  }
  ros::Time out;
  out.fromNSec(static_cast<uint64_t>(in.count()));
  return out;
}

inline ayanami::geometry_msgs::Header fromRosHeader(const std_msgs::Header& in) {
  ayanami::geometry_msgs::Header out;
  out.stamp = fromRosTime(in.stamp);
  out.frame_id = in.frame_id;
  return out;
}

inline std_msgs::Header toRosHeader(const ayanami::geometry_msgs::Header& in) {
  std_msgs::Header out;
  out.stamp = toRosTime(in.stamp);
  out.frame_id = in.frame_id;
  return out;
}

}  // namespace detail

// ---- Pose / PoseStamped ----

inline ayanami::geometry_msgs::PoseStamped fromRosPose(const geometry_msgs::PoseStamped& in) {
  ayanami::geometry_msgs::PoseStamped out;
  out.header = detail::fromRosHeader(in.header);
  out.pose.position =
      Eigen::Vector3d(in.pose.position.x, in.pose.position.y, in.pose.position.z);
  out.pose.orientation =
      Eigen::Quaterniond(in.pose.orientation.w, in.pose.orientation.x,
                         in.pose.orientation.y, in.pose.orientation.z)
          .normalized();
  return out;
}

inline geometry_msgs::PoseStamped toRosPose(const ayanami::geometry_msgs::PoseStamped& in) {
  geometry_msgs::PoseStamped out;
  out.header = detail::toRosHeader(in.header);
  out.pose.position.x = in.pose.position.x();
  out.pose.position.y = in.pose.position.y();
  out.pose.position.z = in.pose.position.z();
  const Eigen::Quaterniond q = in.pose.orientation.normalized();
  out.pose.orientation.x = q.x();
  out.pose.orientation.y = q.y();
  out.pose.orientation.z = q.z();
  out.pose.orientation.w = q.w();
  return out;
}

// ---- Twist / TwistStamped ----

inline ayanami::geometry_msgs::TwistStamped fromRosTwist(const geometry_msgs::TwistStamped& in) {
  ayanami::geometry_msgs::TwistStamped out;
  out.header = detail::fromRosHeader(in.header);
  out.twist.linear = Eigen::Vector3d(in.twist.linear.x, in.twist.linear.y, in.twist.linear.z);
  out.twist.angular = Eigen::Vector3d(in.twist.angular.x, in.twist.angular.y, in.twist.angular.z);
  return out;
}

inline geometry_msgs::TwistStamped toRosTwist(const ayanami::geometry_msgs::TwistStamped& in) {
  geometry_msgs::TwistStamped out;
  out.header = detail::toRosHeader(in.header);
  out.twist.linear.x = in.twist.linear.x();
  out.twist.linear.y = in.twist.linear.y();
  out.twist.linear.z = in.twist.linear.z();
  out.twist.angular.x = in.twist.angular.x();
  out.twist.angular.y = in.twist.angular.y();
  out.twist.angular.z = in.twist.angular.z();
  return out;
}

// ---- Path ----

inline nav_msgs::Path toRosPath(const ayanami::nav_msgs::Path& in) {
  nav_msgs::Path out;
  out.header = detail::toRosHeader(in.header);
  out.poses.reserve(in.poses.size());
  for (const auto& pose : in.poses) {
    out.poses.push_back(toRosPose(pose));
  }
  return out;
}

// ---- OccupancyGrid ----

inline nav_msgs::OccupancyGrid toRosOccupancyGrid(const ayanami::nav_msgs::OccupancyGrid& in) {
  nav_msgs::OccupancyGrid out;
  out.header = detail::toRosHeader(in.header);
  out.info.resolution = in.info.resolution;
  out.info.width = in.info.width;
  out.info.height = in.info.height;
  out.info.origin.position.x = in.info.origin.position.x();
  out.info.origin.position.y = in.info.origin.position.y();
  out.info.origin.position.z = in.info.origin.position.z();
  out.info.origin.orientation.x = in.info.origin.orientation.x();
  out.info.origin.orientation.y = in.info.origin.orientation.y();
  out.info.origin.orientation.z = in.info.origin.orientation.z();
  out.info.origin.orientation.w = in.info.origin.orientation.w();
  out.data.assign(in.data.begin(), in.data.end());
  return out;
}

// ---- PointCloud2 (PCL aware: 与传感器 data blob 对齐, 逐字段拷贝) ----

inline ayanami::sensor_msgs::PointCloud2 fromRosPointCloud2(const sensor_msgs::PointCloud2& in) {
  ayanami::sensor_msgs::PointCloud2 out;
  out.header = detail::fromRosHeader(in.header);
  out.height = in.height;
  out.width = in.width;
  out.fields.reserve(in.fields.size());
  for (const auto& f : in.fields) {
    ayanami::sensor_msgs::PointField pf;
    pf.name = f.name;
    pf.offset = f.offset;
    pf.datatype = f.datatype;
    pf.count = f.count;
    out.fields.push_back(pf);
  }
  out.is_bigendian = in.is_bigendian;
  out.point_step = in.point_step;
  out.row_step = in.row_step;
  out.data.assign(in.data.begin(), in.data.end());
  out.is_dense = in.is_dense;
  return out;
}

inline sensor_msgs::PointCloud2 toRosPointCloud2(const ayanami::sensor_msgs::PointCloud2& in) {
  sensor_msgs::PointCloud2 out;
  out.header = detail::toRosHeader(in.header);
  out.height = in.height;
  out.width = in.width;
  out.fields.reserve(in.fields.size());
  for (const auto& f : in.fields) {
    sensor_msgs::PointField pf;
    pf.name = f.name;
    pf.offset = f.offset;
    pf.datatype = f.datatype;
    pf.count = f.count;
    out.fields.push_back(pf);
  }
  out.is_bigendian = in.is_bigendian;
  out.point_step = in.point_step;
  out.row_step = in.row_step;
  out.data.assign(in.data.begin(), in.data.end());
  out.is_dense = in.is_dense;
  return out;
}

// ---- LaserScan ----

inline ayanami::sensor_msgs::LaserScan fromRosLaserScan(const sensor_msgs::LaserScan& in) {
  ayanami::sensor_msgs::LaserScan out;
  out.header = detail::fromRosHeader(in.header);
  out.angle_min = in.angle_min;
  out.angle_max = in.angle_max;
  out.angle_increment = in.angle_increment;
  out.time_increment = in.time_increment;
  out.scan_time = in.scan_time;
  out.range_min = in.range_min;
  out.range_max = in.range_max;
  out.ranges.assign(in.ranges.begin(), in.ranges.end());
  out.intensities.assign(in.intensities.begin(), in.intensities.end());
  return out;
}

}  // namespace ayanami_ros
