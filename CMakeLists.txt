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
add_library(rviz_viewer SHARED modules/rviz_viewer/rviz_viewer.cpp)
target_include_directories(rviz_viewer PRIVATE
  ${PROJECT_SOURCE_DIR}/modules)
target_link_libraries(rviz_viewer PRIVATE ayanami_ros ayanami::ayanami)
set_target_properties(rviz_viewer PROPERTIES PREFIX "libayanami_")

# costmap_viewer: dedicated costmap output extension (all costmap publishing
# moved out of rviz_viewer). Subscribes the core's per-layer artifact slots,
# caches the artifacts long-term and republishes them from its own thread at
# the configured rates (config_extensions.json::costmap_viewer).
add_library(costmap_viewer SHARED
  modules/costmap_viewer/costmap_viewer.cpp)
target_include_directories(costmap_viewer PRIVATE
  ${PROJECT_SOURCE_DIR}/modules)
target_link_libraries(costmap_viewer PRIVATE ayanami_ros ayanami::ayanami)
set_target_properties(costmap_viewer PROPERTIES PREFIX "libayanami_")

# rosemap_viewer: dedicated rosemap field output extension (mirrors
# costmap_viewer). Subscribes on_rosemap_field, caches the field long-term
# and republishes from its own thread at the configured rate
# (config_extensions.json::rosemap_viewer).
add_library(rosemap_viewer SHARED
  modules/rosemap_viewer/rosemap_viewer.cpp)
target_include_directories(rosemap_viewer PRIVATE
  ${PROJECT_SOURCE_DIR}/modules)
target_link_libraries(rosemap_viewer PRIVATE ayanami_ros ayanami::ayanami)
set_target_properties(rosemap_viewer PROPERTIES PREFIX "libayanami_")

# cmd_chassis_publisher: sole chassis command output extension (rm_msgs
# allowed here; rviz_viewer stays rm_msgs-free). Loaded via
# config_extensions.json::extensions.loading_extension_modules.
add_library(cmd_chassis_publisher SHARED
  modules/cmd_chassis_publisher/cmd_chassis_publisher.cpp)
target_include_directories(cmd_chassis_publisher PRIVATE
  ${PROJECT_SOURCE_DIR}/modules)
target_link_libraries(cmd_chassis_publisher PRIVATE ayanami_ros ayanami::ayanami ${catkin_LIBRARIES})
set_target_properties(cmd_chassis_publisher PROPERTIES PREFIX "libayanami_")

# fast_tf in-process, recording encoded-value change events; dynamically loaded
# via extension_modules by scripts/tf_sync_stress_test.sh, not part of the
# production extension_modules default list).

install(TARGETS ayanami_ros ayanami_rosnode rviz_viewer cmd_chassis_publisher
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
      RVIZ_VIEWER_SO="$<TARGET_FILE:rviz_viewer>"
      COSTMAP_VIEWER_SO="$<TARGET_FILE:costmap_viewer>")
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

  # Task: costmap_viewer ROS extension .so (dlopen; per-layer costmap slots
  # -> cached latched ROS topics, republished from its own thread).
  catkin_add_gtest(costmap_viewer_test tests/costmap_viewer_test.cpp)
  ayanami_ros_include_test_in_all(costmap_viewer_test)
  if(TARGET costmap_viewer_test)
    target_link_libraries(costmap_viewer_test ayanami_ros ${CMAKE_DL_LIBS})
    target_compile_definitions(costmap_viewer_test PRIVATE
      COSTMAP_VIEWER_SO="$<TARGET_FILE:costmap_viewer>")
  endif()

  # Task: rosemap_viewer ROS extension .so (dlopen; on_rosemap_field ->
  # cached latched ROS topic, edge-triggered + rate republish).
  catkin_add_gtest(rosemap_viewer_test tests/rosemap_viewer_test.cpp)
  ayanami_ros_include_test_in_all(rosemap_viewer_test)
  if(TARGET rosemap_viewer_test)
    target_link_libraries(rosemap_viewer_test ayanami_ros ${CMAKE_DL_LIBS})
    target_compile_definitions(rosemap_viewer_test PRIVATE
      ROSEMAP_VIEWER_SO="$<TARGET_FILE:rosemap_viewer>")
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
