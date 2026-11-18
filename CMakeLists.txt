cmake_minimum_required(VERSION 3.8)
project(px4_control)

# 说明：原控制代码(fly_px4/fuck_mid360)和 lib 库已整合进 simulation_uav/simulation_fly 包，
# 此根包仅保留占位，实际代码都在 src/simulation_uav/ 下。

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

ament_package()
