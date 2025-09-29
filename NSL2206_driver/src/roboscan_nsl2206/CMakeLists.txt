cmake_minimum_required(VERSION 3.0.1)
project(roboscan_nsl2206)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros
  pcl_conversions
  pcl_ros
  dynamic_reconfigure
  message_generation
)

generate_dynamic_reconfigure_options(
  cfg/roboscan_nsl2206.cfg
)


find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)


#find_path(NSL_LIB_INCLUDE_DIR NAMES nanolib.h PATHS ${CMAKE_CURRENT_SOURCE_DIR}/nsl_lib/include REQUIRED)
#find_library(NSL_LIB_PATH NAMES nanolib PATHS ${CMAKE_CURRENT_SOURCE_DIR}/nsl_lib/lib/linux-7.5/static REQUIRED)
find_path(NSL_LIB_INCLUDE_DIR NAMES nanolib.h PATHS nsl_lib/include REQUIRED)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/aarch-7.5/shared REQUIRED)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/linux-7.5/shared REQUIRED)
endif()

add_message_files(
  FILES
  CustomMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime dynamic_reconfigure
  DEPENDS PCL Boost OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${NSL_LIB_INCLUDE_DIR}
)

add_executable(roboscan_publish_node src/roboscan_publish_node.cpp)

add_dependencies(roboscan_publish_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(roboscan_publish_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${NSL_LIB_PATH}
  yaml-cpp
  stdc++fs
)


install(TARGETS roboscan_publish_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)