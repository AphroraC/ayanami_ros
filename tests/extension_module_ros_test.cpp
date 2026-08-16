#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/master.h>

#include <ayanami/utility/extension_module.hpp>
#include <ayanami_ros/extension_module_ros.hpp>

#include <memory>
#include <string>

namespace {

#ifndef STUB_EXTENSION_SO
#error "STUB_EXTENSION_SO must be defined by CMake"
#endif

const char* const g_stub_so = STUB_EXTENSION_SO;

} // namespace

TEST(ExtensionModuleROS, DlopenCreateSubscriptions) {
  auto ext = ayanami::ExtensionModule::load_module(g_stub_so);
  ASSERT_NE(ext, nullptr);
  auto ros_ext = std::dynamic_pointer_cast<ayanami_ros::ExtensionModuleROS>(ext);
  ASSERT_NE(ros_ext, nullptr);
  ros::NodeHandle nh("~");
  auto subs = ros_ext->create_subscriptions(nh);
  EXPECT_TRUE(subs.empty());
}

TEST(ExtensionModuleROS, DefaultLifecycleHooks) {
  auto ext = ayanami::ExtensionModule::load_module(g_stub_so);
  ASSERT_NE(ext, nullptr);
  auto ros_ext = std::dynamic_pointer_cast<ayanami_ros::ExtensionModuleROS>(ext);
  ASSERT_NE(ros_ext, nullptr);
  EXPECT_FALSE(ros_ext->needs_wait());
  EXPECT_TRUE(ros_ext->ok());
  ros_ext->at_exit("/tmp/ayanami_ros_stub_dump_path");
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // 无 roscore 环境下避免 roscpp master 无限重连：限定重试超时 (brief: fake_nh 绕过)。
  ros::init(argc, argv, "extension_module_ros_test",
            ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
  ros::master::setRetryTimeout(ros::WallDuration(0.3));
  return RUN_ALL_TESTS();
}
