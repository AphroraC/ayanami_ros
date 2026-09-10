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
  rm_msgs
)

find_package(behaviortree_cpp REQUIRED)
find_package(ayanami CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(spdlog REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

# Use ayanami's vendored nlohmann (3.11.3): the system 3.7.3 is incompatible
# with this toolchain (GCC10) (see thirdparty.cmake note). ayanami does not
# export the vendored headers downstream, so point here at the thirdparty
# directory of the ayanami source tree in the same workspace. Same for fast_tf
# (header-only; the TransformStore header includes <fast_tf.hpp>).
include_directories(${PROJECT_SOURCE_DIR}/../ayanami/thirdparty)
include_directories(${PROJECT_SOURCE_DIR}/../ayanami/thirdparty/fast_tf)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ayanami_ros
  CATKIN_DEPENDS roscpp roslib tf2 tf2_eigen tf2_geometry_msgs tf2_ros nav_msgs sensor_msgs geometry_msgs std_msgs pcl_conversions rm_msgs
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

# Task 9: rviz_viewer ROS extension .so (spec §4.5 core->ROS). Exports the
# create_extension_module factory symbol for AyanamiROS to dlopen;
# config_ros.json::extension_modules references it by the bare name
# "libayanami_rviz_viewer.so".
add_library(rviz_viewer SHARED modules/rviz_viewer.cpp)
target_link_libraries(rviz_viewer PRIVATE ayanami_ros ayanami::ayanami)
set_target_properties(rviz_viewer PROPERTIES PREFIX "libayanami_")

# cmd_chassis_publisher: sole chassis command output extension (rm_msgs
# allowed here; rviz_viewer stays rm_msgs-free). Loaded via
# config_extensions.json::extensions.loading_extension_modules.
add_library(cmd_chassis_publisher SHARED modules/cmd_chassis_publisher.cpp)
target_link_libraries(cmd_chassis_publisher PRIVATE ayanami_ros ayanami::ayanami ${catkin_LIBRARIES})
set_target_properties(cmd_chassis_publisher PROPERTIES PREFIX "libayanami_")

# tf_sync_probe: TF sync stress-test probe extension (polls TransformStore/
# fast_tf in-process, recording encoded-value change events; dynamically loaded
# via extension_modules by scripts/tf_sync_stress_test.sh, not part of the
# production extension_modules default list).
add_library(tf_sync_probe SHARED modules/tf_sync_probe.cpp)
target_link_libraries(tf_sync_probe PRIVATE ayanami_ros ayanami::ayanami)
set_target_properties(tf_sync_probe PROPERTIES PREFIX "libayanami_")

install(TARGETS ayanami_ros ayanami_rosnode rviz_viewer cmd_chassis_publisher
  tf_sync_probe
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

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
      RVIZ_VIEWER_SO="$<TARGET_FILE:rviz_viewer>")
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
