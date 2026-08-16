cmake_minimum_required(VERSION 3.16)
project(ayanami_ros VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib tf2 tf2_eigen tf2_geometry_msgs tf2_ros
  nav_msgs sensor_msgs geometry_msgs pcl_conversions
)

find_package(behaviortree_cpp REQUIRED)
find_package(ayanami CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(spdlog REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

# 用 ayanami vendored nlohmann (3.11.3): 系统 3.7.3 与本 toolchain (GCC10)
# 不兼容 (thirdparty.cmake 注)。ayanami 未将 vendored 头导出给下游, 此处指向
# 同一 workspace 内 ayanami 源码树 thirdparty。fast_tf 同 (header-only,
# TransformStore 头 include <fast_tf.hpp>)。
include_directories(${PROJECT_SOURCE_DIR}/../ayanami/thirdparty)
include_directories(${PROJECT_SOURCE_DIR}/../ayanami/thirdparty/fast_tf)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ayanami_ros
  CATKIN_DEPENDS roscpp roslib tf2 tf2_eigen tf2_geometry_msgs tf2_ros nav_msgs sensor_msgs geometry_msgs pcl_conversions
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

# Task 9: rviz_viewer ROS extension .so (spec §4.5 核心->ROS)。导出
# create_extension_module 工厂符号供 AyanamiROS dlopen; config_ros.json::extension_modules
# 以 "librviz_viewer.so" 裸名引用 (T10 写)。
add_library(rviz_viewer SHARED src/ayanami_ros/extensions/rviz_viewer.cpp)
target_link_libraries(rviz_viewer PRIVATE ayanami_ros ayanami::ayanami)
set_target_properties(rviz_viewer PROPERTIES PREFIX "lib")

install(TARGETS ayanami_ros ayanami_rosnode rviz_viewer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)

  # stub extension .so 用于 dlopen 测试 (Task 5; 真订阅留给 rviz_viewer T9)
  add_library(_extension_template SHARED src/ayanami_ros/extensions/_extension_template.cpp)
  target_link_libraries(_extension_template PRIVATE ayanami_ros)
  set_target_properties(_extension_template PROPERTIES PREFIX "lib")

  catkin_add_gtest(extension_module_ros_test tests/extension_module_ros_test.cpp)
  if(TARGET extension_module_ros_test)
    target_link_libraries(extension_module_ros_test ayanami_ros)
    target_compile_definitions(extension_module_ros_test PRIVATE
      STUB_EXTENSION_SO="$<TARGET_FILE:_extension_template>")
  endif()

  catkin_add_gtest(ayanami_ros_load_test tests/ayanami_ros_load_test.cpp)
  if(TARGET ayanami_ros_load_test)
    target_link_libraries(ayanami_ros_load_test ayanami_ros ${CMAKE_DL_LIBS})
    target_compile_definitions(ayanami_ros_load_test PRIVATE
      STUB_EXTENSION_SO="$<TARGET_FILE:_extension_template>")
  endif()

  # Task 7: TF2Listener (tf2_ros Buffer + TransformListener + lookup_transform)。
  catkin_add_gtest(tf2_listener_test tests/tf2_listener_test.cpp)
  if(TARGET tf2_listener_test)
    target_link_libraries(tf2_listener_test ayanami_ros)
  endif()

  # Task 8: ros_msg_converter (Eigen ↔ ROS msgs) 纯头文件 inline 实现。
  catkin_add_gtest(ros_msg_converter_test tests/ros_msg_converter_test.cpp)
  if(TARGET ros_msg_converter_test)
    target_link_libraries(ros_msg_converter_test ayanami_ros)
  endif()

  # Task 9: rviz_viewer ROS extension .so (dlopen + create_subscriptions 链)。
  catkin_add_gtest(rviz_viewer_test tests/rviz_viewer_test.cpp)
  if(TARGET rviz_viewer_test)
    target_link_libraries(rviz_viewer_test ayanami_ros ${CMAKE_DL_LIBS})
    target_compile_definitions(rviz_viewer_test PRIVATE
      RVIZ_VIEWER_SO="$<TARGET_FILE:rviz_viewer>")
  endif()

  # Task 10: config_ros.json + costmap.json + costmap_navigation.json
  # (Gazebo nav_demo 对齐) 内容一致性验证 (spec §8.1-§8.3)。
  # 用源树真实 config/assets 目录 + nav_demo 地图目录做兼容性检查。
  catkin_add_gtest(config_spec_verify_test tests/config_spec_verify_test.cpp)
  if(TARGET config_spec_verify_test)
    target_link_libraries(config_spec_verify_test ayanami_ros)
    target_include_directories(config_spec_verify_test PRIVATE
      ${PROJECT_SOURCE_DIR}/../ayanami/packages/costmap_2d/include)
    target_compile_definitions(config_spec_verify_test PRIVATE
      AYANAMI_TEST_CONFIG_DIR="${PROJECT_SOURCE_DIR}/../ayanami/config"
      AYANAMI_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/../ayanami/assets"
      NAV_DEMO_MAP_DIR="${PROJECT_SOURCE_DIR}/../node/nav_demo/map")
  endif()
endif()
