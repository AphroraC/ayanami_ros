cmake_minimum_required(VERSION 3.16)
project(ayanami_ros VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use the same shared_ptr atomic implementation strategy as ayanami (-mcx16),
# so TUs in this package that include the CallbackSlot/CostmapWrapper headers
# (ayanami_ros lib, rviz_viewer, tests) never mix shared_ptr objects built with
# a different implementation path than libayanami internals.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mcx16" AYANAMI_ROS_HAS_MCX16)
if(AYANAMI_ROS_HAS_MCX16)
  add_compile_options(-mcx16)
else()
  message(WARNING "ayanami_ros: -mcx16 not supported; it may disagree with the ayanami atomic policy of this workspace")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib tf2 tf2_eigen tf2_geometry_msgs tf2_ros
  nav_msgs sensor_msgs geometry_msgs std_msgs pcl_conversions
  actionlib mbf_msgs nodelet pluginlib
  rm_msgs
)

# BT is statically embedded inside ayanami (PRIVATE dependency) -- ayanami_ros
# consumes it only through ayanami::ayanami and needs no BT find_package.
find_package(ayanami CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(spdlog REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

# <nlohmann/...> resolves from the workspace install space: ayanami installs
# its vendored nlohmann tree into devel/include/nlohmann (see ayanami's
# thirdparty.cmake), and devel/include is on this package's include path.

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ayanami_ros ayanami_nodelet
  CATKIN_DEPENDS roscpp roslib tf2 tf2_eigen tf2_geometry_msgs tf2_ros nav_msgs sensor_msgs geometry_msgs std_msgs pcl_conversions actionlib mbf_msgs rm_msgs
)

add_library(ayanami_ros
  src/ayanami_ros/ayanami_ros.cpp
  src/ayanami_ros/tf2_listener.cpp
)
target_link_libraries(ayanami_ros PUBLIC
  ayanami::ayanami
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  pcl_common pcl_io
  spdlog::spdlog
)

add_executable(ayanami_rosnode apps/ayanami_rosnode.cpp)
target_link_libraries(ayanami_rosnode ayanami_ros)

# nodelet wrapper (asuka mode): same AyanamiROS boundary, loaded via
# nodelet_plugins.xml (ayanami/AyanamiNodelet) into a shared manager such as
# nav_bringup's "evangelion" -- the /nagisa/obstacle input then stays
# intra-process.
add_library(ayanami_nodelet SHARED apps/ayanami_nodelet.cpp)
target_link_libraries(ayanami_nodelet ayanami_ros ${catkin_LIBRARIES})

# Task 9: rviz_viewer ROS extension .so (spec §4.5 core->ROS). Exports the
# create_extension_module factory symbol for AyanamiROS to dlopen;
# config_ros.json::extension_modules references it by the bare name
# "libayanami_rviz_viewer.so".
add_library(rviz_viewer SHARED modules/rviz_viewer/rviz_viewer.cpp)
target_include_directories(rviz_viewer PRIVATE
  ${PROJECT_SOURCE_DIR}/modules)
target_link_libraries(rviz_viewer PRIVATE ayanami_ros ayanami::ayanami)
set_target_properties(rviz_viewer PROPERTIES PREFIX "libayanami_")

# rose_viewer: dedicated costmap output extension (all costmap publishing
# moved out of rviz_viewer). Subscribes the core's per-layer artifact slots,
# caches the artifacts long-term and republishes them from its own thread at
# the configured rates (config_extensions.json::rose_viewer).
add_library(rose_viewer SHARED
  modules/rose_viewer/rose_viewer.cpp)
target_include_directories(rose_viewer PRIVATE
  ${PROJECT_SOURCE_DIR}/modules)
target_link_libraries(rose_viewer PRIVATE ayanami_ros ayanami::ayanami)
set_target_properties(rose_viewer PROPERTIES PREFIX "libayanami_")


# cmd_chassis_publisher: sole chassis command output extension (rm_msgs
# allowed here; rviz_viewer stays rm_msgs-free). Loaded via
# config_extensions.json::extensions.loading_extension_modules.
add_library(cmd_chassis_publisher SHARED
  modules/cmd_chassis_publisher/cmd_chassis_publisher.cpp)
target_include_directories(cmd_chassis_publisher PRIVATE
  ${PROJECT_SOURCE_DIR}/modules)
target_link_libraries(cmd_chassis_publisher PRIVATE ayanami_ros ayanami::ayanami ${catkin_LIBRARIES})
set_target_properties(cmd_chassis_publisher PROPERTIES PREFIX "libayanami_")

# mbf_action_server: MBF compatibility layer for the behavior tree's
# SimpleActionClient<mbf_msgs::MoveBaseAction>("/move_base_flex/move_base").
# Goals go through NavigationInputs::on_goal_request (AyanamiROS forwards
# them into the goal mailbox); NavigationCallbacks::on_navigation_result
# completes the action. Loaded via
# config_extensions.json::extensions.loading_extension_modules.
add_library(mbf_action_server SHARED
  modules/mbf_action_server/mbf_action_server.cpp)
target_include_directories(mbf_action_server PRIVATE
  ${PROJECT_SOURCE_DIR}/modules)
target_link_libraries(mbf_action_server PRIVATE ayanami_ros ayanami::ayanami ${catkin_LIBRARIES})
set_target_properties(mbf_action_server PROPERTIES PREFIX "libayanami_")

# fast_tf in-process, recording encoded-value change events; dynamically loaded
# via extension_modules by scripts/tf_sync_stress_test.sh, not part of the
# production extension_modules default list).

install(TARGETS ayanami_ros ayanami_rosnode ayanami_nodelet rviz_viewer cmd_chassis_publisher mbf_action_server
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)

  # catkin_add_gtest always passes EXCLUDE_FROM_ALL to the executable (see
  # catkin/cmake/test/gtest.cmake), so `catkin build ayanami_ros` never
  # rebuilds the test binaries and `ctest` can silently run stale executables
  # against edited sources. Put each test back into `all` so a normal build
  # keeps them current. The TARGET guard is required: catkin_add_gtest is a
  # no-op when testing is disabled or GTest is unavailable.
  function(ayanami_ros_include_test_in_all target)
    if(TARGET ${target})
      set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL FALSE)
    endif()
  endfunction()

  catkin_add_gtest(ayanami_ros_load_test tests/ayanami_ros_load_test.cpp)
  ayanami_ros_include_test_in_all(ayanami_ros_load_test)
  if(TARGET ayanami_ros_load_test)
    target_link_libraries(ayanami_ros_load_test ayanami_ros ${CMAKE_DL_LIBS})
    target_compile_definitions(ayanami_ros_load_test PRIVATE
      RVIZ_VIEWER_SO="$<TARGET_FILE:rviz_viewer>"
      ROSE_VIEWER_SO="$<TARGET_FILE:rose_viewer>")
  endif()

  # Task 7: TF2Listener (tf2_ros Buffer + TransformListener + lookup_transform).
  catkin_add_gtest(tf2_listener_test tests/tf2_listener_test.cpp)
  ayanami_ros_include_test_in_all(tf2_listener_test)
  if(TARGET tf2_listener_test)
    target_link_libraries(tf2_listener_test ayanami_ros)
  endif()

  # Task 8: ros_msg_converter (Eigen <-> ROS msgs), header-only inline
  # implementation.
  catkin_add_gtest(ros_msg_converter_test tests/ros_msg_converter_test.cpp)
  ayanami_ros_include_test_in_all(ros_msg_converter_test)
  if(TARGET ros_msg_converter_test)
    target_link_libraries(ros_msg_converter_test ayanami_ros)
  endif()

  # Task 9: rviz_viewer ROS extension .so (dlopen; creates its own NodeHandle
  # at construction time and advertises + registers NavigationCallbacks).
  catkin_add_gtest(rviz_viewer_test tests/rviz_viewer_test.cpp)
  ayanami_ros_include_test_in_all(rviz_viewer_test)
  if(TARGET rviz_viewer_test)
    target_link_libraries(rviz_viewer_test ayanami_ros ${CMAKE_DL_LIBS})
    target_compile_definitions(rviz_viewer_test PRIVATE
      RVIZ_VIEWER_SO="$<TARGET_FILE:rviz_viewer>")
  endif()

  # Task: rose_viewer ROS extension .so (dlopen; per-layer costmap slots
  # -> cached latched ROS topics, republished from its own thread).
  catkin_add_gtest(rose_viewer_test tests/rose_viewer_test.cpp)
  ayanami_ros_include_test_in_all(rose_viewer_test)
  if(TARGET rose_viewer_test)
    target_link_libraries(rose_viewer_test ayanami_ros ${CMAKE_DL_LIBS})
    target_compile_definitions(rose_viewer_test PRIVATE
      ROSE_VIEWER_SO="$<TARGET_FILE:rose_viewer>")
  endif()

  catkin_add_gtest(cmd_chassis_publisher_test
                   tests/cmd_chassis_publisher_test.cpp)
  ayanami_ros_include_test_in_all(cmd_chassis_publisher_test)
  if(TARGET cmd_chassis_publisher_test)
    target_link_libraries(cmd_chassis_publisher_test ayanami_ros
                          ${CMAKE_DL_LIBS})
    target_compile_definitions(cmd_chassis_publisher_test PRIVATE
      CMD_CHASSIS_PUBLISHER_SO="$<TARGET_FILE:cmd_chassis_publisher>")
  endif()

  # Task 10: content consistency verification of config_ros.json + costmap.json
  # + costmap_navigation.json (aligned with Gazebo nav_demo)
  # (spec §8.1-§8.3). Uses the real in-tree config/assets directories plus the
  # nav_demo map directory for compatibility checks.
  catkin_add_gtest(config_spec_verify_test tests/config_spec_verify_test.cpp)
  ayanami_ros_include_test_in_all(config_spec_verify_test)
  if(TARGET config_spec_verify_test)
    target_link_libraries(config_spec_verify_test ayanami_ros)
    target_include_directories(config_spec_verify_test PRIVATE
      ${PROJECT_SOURCE_DIR}/../ayanami/packages/costmap_2d/include)
    target_compile_definitions(config_spec_verify_test PRIVATE
      AYANAMI_TEST_CONFIG_DIR="${PROJECT_SOURCE_DIR}/../ayanami/config"
      AYANAMI_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/../ayanami/config/assets"
      NAV_DEMO_MAP_DIR="${PROJECT_SOURCE_DIR}/../node/nav_demo/map")
  endif()
endif()
