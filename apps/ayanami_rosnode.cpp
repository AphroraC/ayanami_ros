#include <cstdio>
#include <exception>
#include <memory>

#include <spdlog/spdlog.h>
#include <ros/ros.h>
#include <ayanami_ros/ayanami_ros.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ayanami_rosnode");
  std::unique_ptr<ayanami_ros::AyanamiROS> node;
  try {
    ros::NodeHandle nh("~");
    node = std::make_unique<ayanami_ros::AyanamiROS>(nh);
    node->start();
  } catch (const std::exception& e) {
    // Startup failures are expected shapes (bad route config, missing module
    // .so, family mismatch): report and exit non-zero instead of letting the
    // exception escape main -- an uncaught throw aborts the process, which
    // roslaunch reports as an opaque "died of signal 6".
    spdlog::critical("ayanami_rosnode: fatal: {}", e.what());
    std::fprintf(stderr, "ayanami_rosnode: fatal: %s\n", e.what());
    return 1;
  } catch (...) {
    spdlog::critical("ayanami_rosnode: fatal: unknown exception");
    std::fprintf(stderr, "ayanami_rosnode: fatal: unknown exception\n");
    return 1;
  }
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  spinner.stop();
  node->stop();
  return 0;
}
