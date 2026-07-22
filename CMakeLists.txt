cmake_minimum_required(VERSION 3.28)
project(small_gimbal_tracker)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE Release)

# ── 依赖库 ──
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

# ── Galaxy 相机 SDK（头文件和库在原项目目录）──
set(GALAXY_SDK /home/sg/下载/little-trouble-change-master/io/galaxy)

# ── serial 库 (静态编译，头文件在 serial/include) ──
add_library(serial STATIC
  io/serial/src/serial.cc
  io/serial/src/impl/unix.cc
  io/serial/src/impl/list_ports/list_ports_linux.cc
)
target_include_directories(serial PUBLIC io/serial/include)
target_link_libraries(serial rt pthread)

# ── io 层 (相机 + 云台串口) ──
add_library(io STATIC
  io/camera.cpp
  io/galaxy/galaxy.cpp
  io/gimbal/gimbal.cpp
)
target_include_directories(io PUBLIC io/galaxy/include)
target_link_directories(io PUBLIC ${GALAXY_SDK}/lib/x86_64)
target_link_libraries(io gxiapi usb-1.0 yaml-cpp serial)

# ── auto_aim (YOLO检测 + PnP解算) ──
add_library(auto_aim STATIC
  tasks/auto_aim/armor.cpp
  tasks/auto_aim/classifier.cpp
  tasks/auto_aim/detector.cpp
  tasks/auto_aim/solver.cpp
  tasks/auto_aim/yolo.cpp
  tasks/auto_aim/yolos/yolov8.cpp
)
target_link_libraries(auto_aim openvino::runtime)

# ── tools (logger, math, etc.) ──
add_library(tools STATIC
  tools/exiter.cpp
  tools/img_tools.cpp
  tools/math_tools.cpp
  tools/logger.cpp
)
target_link_libraries(tools ${OpenCV_LIBS} fmt::fmt)

# ── 主程序 ──
add_executable(small_gimbal src/main.cpp)
target_link_libraries(small_gimbal
  ${OpenCV_LIBS}
  fmt::fmt
  yaml-cpp
  auto_aim
  tools
  io
)
