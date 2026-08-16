#pragma once
#include <ros/ros.h>
#include <string>
namespace ayanami_ros {
class AyanamiROS {
  ros::NodeHandle& nh_;
public:
  explicit AyanamiROS(ros::NodeHandle& nh) : nh_(nh) {}
  void start() {}
  void stop() {}
};
}
