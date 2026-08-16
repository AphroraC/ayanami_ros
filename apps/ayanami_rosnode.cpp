#include <ros/ros.h>
#include <ayanami_ros/ayanami_ros.hpp>
int main(int argc, char** argv) {
  ros::init(argc, argv, "ayanami_rosnode");
  ros::NodeHandle nh("~");
  ayanami_ros::AyanamiROS node(nh);
  node.start();
  ros::spin();
  node.stop();
  return 0;
}
