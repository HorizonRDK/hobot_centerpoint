cmake_minimum_required(VERSION 3.5)
project(hobot_centerpoint)

# SET(CMAKE_BUILD_TYPE "Debug")  # 定义编译类型
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb") # 定义Debug编译参数
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") # 定义Release编译参数

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(dnn_node REQUIRED)

# j5|x86
set(PREFIX_PATH j5)
set(SYS_ROOT ${CMAKE_SYSROOT})

if(PLATFORM_X3)
  message("build platform X3")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
elseif(PLATFORM_J5)
  message("build platform J5")
  add_definitions(-DPLATFORM_J5)
  set(PREFIX_PATH j5)
elseif(PLATFORM_X86)
  message("build platform x86")
  add_definitions(-DPLATFORM_X86)
  set(PREFIX_PATH x86)
  set(SYS_ROOT ${THIRD_PARTY})
else()
  if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    message("invalid platform, build platform X3 default")
    add_definitions(-DPLATFORM_X3)
    set(PREFIX_PATH x3)
  elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    message("build platform X86")
    add_definitions(-DPLATFORM_X86)
    set(PREFIX_PATH x86)
    set(SYS_ROOT ${THIRD_PARTY})
  endif()
endif()

message("PREFIX_PATH is " ${PREFIX_PATH})
message("SYS_ROOT is " ${SYS_ROOT})

link_directories(
  ${SYS_ROOT}/usr/lib/
  ${SYS_ROOT}/usr/lib/hbbpu/
  ${SYS_ROOT}/usr/lib/hbmedia/
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${SYS_ROOT}/usr/include
  ${SYS_ROOT}/usr/include/opencv/
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/centerpoint_node.cpp
  src/preprocess.cpp
  src/postprocess.cpp
  src/utils/box3d_nms.cc
  src/centerpoint_publisher.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  dnn_node
  sensor_msgs
  ai_msgs
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
