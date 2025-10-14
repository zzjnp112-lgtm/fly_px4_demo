cmake_minimum_required(VERSION 3.8)
project(px4_control)

# Default to C++17
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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial REQUIRED)
find_package(nav_msgs REQUIRED)





# 包含头文件目录
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
  ${PROJECT_NAME}
)

# 添加库文件
add_library(arduino_lib
  src/lib/Arduino.cpp
)

add_library(fixed_point_lib
src/lib/fixed_point.cpp
)

add_library(pid_controller_lib
src/lib/pidcontroller.cpp

)

# 设置库的属性和依赖关系

# 设置 pid_controller_lib 的链接
target_link_libraries(pid_controller_lib PUBLIC
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# 设置 fixed_point_lib 的链接，确保依赖 pid_controller_lib
target_link_libraries(fixed_point_lib PUBLIC
  pid_controller_lib
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# 设置 arduino_lib 的链接
target_link_libraries(arduino_lib PUBLIC
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# 添加可执行文件
add_executable(fly_px4 src/fly_px4.cpp)
target_link_libraries(fly_px4
  arduino_lib
  fixed_point_lib  # 这会自动链接其依赖的 pid_controller_lib
)

add_executable(fuck_mid360 src/fuck_mid360.cpp)
target_link_libraries(fuck_mid360
  arduino_lib
  
)

# 添加依赖
ament_target_dependencies(fly_px4
  rclcpp
  geometry_msgs
  mavros_msgs
  std_msgs

  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros

  serial
  nav_msgs

)
ament_target_dependencies(fuck_mid360
  rclcpp
  geometry_msgs

  mavros_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  serial
  nav_msgs

)



# 安装目标
install(TARGETS
  fly_px4
  fuck_mid360
  DESTINATION lib/${PROJECT_NAME}
)

# 安装库文件
install(TARGETS
  arduino_lib
fixed_point_lib
pid_controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  geometry_msgs
  mavros_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros


  serial

)

ament_export_include_directories(include)
ament_export_libraries(
  arduino_lib
  pid_controller_lib
  fixed_point_lib
)

ament_package()
