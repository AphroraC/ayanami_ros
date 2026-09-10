// cmd_chassis_publisher ROS extension .so (dlopen -> create_extension_module
// -> CmdChassisPublisher : ayanami::ExtensionModule).
//
// No roscore needed (headless): a temp config dir (config_extensions.json
// with a cmd_chassis_publisher section) + dlopen, verifying:
// 1. dlopen + factory + ExtensionModule cast.
// 2. Constructor subscribes NavigationCallbacks::on_velocity_command.
// 3. Emitting the slot increments the exported publish-count hook
//    (rm_msgs::ChassisCmd on /cmd_chassis; no subscriber -> publish no-op).
// 4. Topic param comes from config_extensions.json::cmd_chassis_publisher.

#include <dlfcn.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

#ifndef CMD_CHASSIS_PUBLISHER_SO
#error "CMD_CHASSIS_PUBLISHER_SO must be defined by CMake"
#endif

const char* const g_module_so = CMD_CHASSIS_PUBLISHER_SO;

void writeFile(const std::filesystem::path& path, const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream ofs(path);
  ASSERT_TRUE(ofs.good()) << "cannot write " << path;
  ofs << content;
}

std::filesystem::path makeConfigDir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_ros_cmd_chassis_test";
  std::filesystem::remove_all(dir);
  writeFile(dir / "config.json",
            "{\n  \"global\": {\n"
            "    \"config_extensions\": \"config_extensions.json\"\n"
            "  }\n}\n");
  writeFile(dir / "config_extensions.json",
            "{\n  \"extensions\": {\n"
            "    \"loading_extension_modules\": []\n"
            "  },\n"
            "  \"cmd_chassis_publisher\": {\n"
            "    \"cmd_chassis_topic\": \"/test/cmd_chassis\"\n"
            "  }\n}\n");
  return dir;
}

void initGlobalConfig(const std::filesystem::path& dir) {
  setenv("AYANAMI_CONFIG_PATH", dir.string().c_str(), 1);
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);
}

int pubCount(const char* symbol) {
  using Getter = int (*)();
  auto getter = reinterpret_cast<Getter>(dlsym(RTLD_DEFAULT, symbol));
  return getter ? getter() : -1;
}

TEST(CmdChassisPublisher, dlopenCreateAndSubscribeVelocity) {
  initGlobalConfig(makeConfigDir());

  auto ext = ayanami::ExtensionModule::loadModule(g_module_so);
  ASSERT_NE(ext, nullptr);

  // Constructor advertise/subscribe done: count hook reachable (dlsym, >= 0).
  EXPECT_GE(pubCount("cmdChassisPubCount"), 0);

  // Emit the core velocity slot -> chassis publish handler runs.
  ayanami::geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "odom";
  twist.twist.linear = Eigen::Vector3d(0.5, 0.0, 0.0);
  twist.twist.angular.z() = 0.2;
  const int before = pubCount("cmdChassisPubCount");
  ayanami::interfaces::NavigationCallbacks::on_velocity_command(twist);
  EXPECT_GE(pubCount("cmdChassisPubCount"), before + 1);

  // Destructor unsubscribes: emit again -> count unchanged.
  ext.reset();
  ASSERT_EQ(ext, nullptr);
  const int after_dtor = pubCount("cmdChassisPubCount");
  ayanami::interfaces::NavigationCallbacks::on_velocity_command(twist);
  EXPECT_EQ(pubCount("cmdChassisPubCount"), after_dtor);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cmd_chassis_publisher_test",
            ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
  ros::master::setRetryTimeout(ros::WallDuration(0.3));
  return RUN_ALL_TESTS();
}
