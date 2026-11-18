cmake_minimum_required(VERSION 3.8)
project(simulation_fly)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(serial REQUIRED)

# gz-transport 版本回退链：优先 Harmonic=13，其次 Garden=12，最后未版本化
find_package(gz-transport13 QUIET)
if(TARGET gz-transport13::gz-transport13)
  set(GZ_TRANSPORT_LIB gz-transport13::gz-transport13)
else()
  find_package(gz-transport12 QUIET)
  if(TARGET gz-transport12::gz-transport12)
    set(GZ_TRANSPORT_LIB gz-transport12::gz-transport12)
  else()
    find_package(gz-transport QUIET)
    if(TARGET gz-transport::gz-transport)
      set(GZ_TRANSPORT_LIB gz-transport::gz-transport)
    else()
      find_package(gz-transport REQUIRED)
      set(GZ_TRANSPORT_LIB gz-transport::gz-transport)
    endif()
  endif()
endif()

# 包含头文件目录
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}   # 让 #include "lib/Arduino.h" 解析到 lib/
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)

# lib 库（fly_px4 / fuck_mid360 依赖的 Arduino、FixedPoint、PID）
add_library(sim_common
  lib/Arduino.cpp
  lib/fixed_point.cpp
  lib/pidcontroller.cpp
)
ament_target_dependencies(sim_common rclcpp std_msgs serial)

# 可执行文件
add_executable(fly_px4 src/fly_px4.cpp)
add_executable(fuck_mid360 src/fuck_mid360.cpp)
add_executable(camer_bridge src/camer_bridge.cpp)
add_executable(time_bridge src/time_bridge.cpp)
add_executable(point_bridge src/point_bridge.cpp)

# fly_px4 依赖
ament_target_dependencies(fly_px4
  rclcpp geometry_msgs mavros_msgs std_msgs std_srvs
  tf2 tf2_geometry_msgs tf2_ros serial nav_msgs
)
target_link_libraries(fly_px4 sim_common)

# fuck_mid360 依赖
ament_target_dependencies(fuck_mid360
  rclcpp geometry_msgs mavros_msgs std_msgs std_srvs
  tf2 tf2_geometry_msgs tf2_ros serial nav_msgs
)
target_link_libraries(fuck_mid360 sim_common)

# time_bridge（gz clock -> ROS2 clock）
ament_target_dependencies(time_bridge
  rclcpp sensor_msgs std_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs
)
target_link_libraries(time_bridge ${GZ_TRANSPORT_LIB})

# point_bridge（点云变换）
ament_target_dependencies(point_bridge
  rclcpp sensor_msgs std_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs
)

# camer_bridge（gz-sim 传感器 -> ROS2）
ament_target_dependencies(camer_bridge
  rclcpp sensor_msgs std_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs
)
target_link_libraries(camer_bridge ${GZ_TRANSPORT_LIB})

# 安装目标
install(TARGETS
  fly_px4
  fuck_mid360
  camer_bridge
  time_bridge
  point_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp geometry_msgs mavros_msgs std_msgs std_srvs sensor_msgs
  tf2 tf2_geometry_msgs tf2_ros quadrotor_msgs nav_msgs serial
)

ament_export_include_directories(include)

ament_package()
