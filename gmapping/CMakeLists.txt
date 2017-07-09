cmake_minimum_required(VERSION 3.5)
project(gmapping)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

set(req_deps
  "nav_msgs"
  "std_msgs"
  "sensor_msgs"
  "std_srvs"
  "geometry_msgs"
  "openslam_gmapping"
  "rclcpp"
  "rcutils"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_msgs"
  "tf2_ros"
)
ament_auto_find_build_dependencies(REQUIRED ${req_deps})

include_directories(src)

ament_auto_add_executable(slam_gmapping src/slam_gmapping.cpp src/main.cpp)
ament_target_dependencies(slam_gmapping ${req_deps})

# add_executable(slam_gmapping_replay src/slam_gmapping.cpp src/replay.cpp)
# ament_target_dependencies(slam_gmapping_replay ${req_deps})

ament_auto_package()
