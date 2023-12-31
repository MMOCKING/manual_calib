cmake_minimum_required(VERSION 2.8.3)
project(ilcc2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  camera_model
  rosbag
  roslib
  dynamic_reconfigure
)

find_package(Eigen3)
include_directories( ${EIGEN3_INCLUDE_DIR} )

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

generate_dynamic_reconfigure_options(
  cfg/Locator.cfg
  # cfg/DynReconf2.cfg
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES ilcc2_lib
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable (lidar_cam_calib src/lidar_cam_calib.cpp src/ImageEst.cpp )
target_link_libraries (lidar_cam_calib camera_model  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )
# make sure configure headers are built before any node using them
add_dependencies(lidar_cam_calib ${PROJECT_NAME}_gencfg)


add_executable(bag_extractor src/bag_extractor.cpp)
target_link_libraries(bag_extractor ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(lidar_radar_calib src/lidar_radar_calib.cpp)
target_link_libraries(lidar_radar_calib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )