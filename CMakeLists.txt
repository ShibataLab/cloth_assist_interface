cmake_minimum_required(VERSION 2.8.3)
project(cloth_assist_interface)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter -Wreorder")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

find_package(
  catkin
  REQUIRED
  COMPONENTS
  roscpp
  rostime
  rosbag
  std_msgs
  sensor_msgs
  message_filters
  cv_bridge
  image_transport
  compressed_image_transport
  compressed_depth_image_transport
  kinect2_bridge
  ar_track_alvar_msgs
  baxter_core_msgs
  baxter_interface
  control_msgs
  actionlib
  message_generation)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

## MSG Files
add_message_files(
  FILES
  TopCoord.msg
)

catkin_python_setup()

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

generate_messages(
  DEPENDENCIES
  std_msgs)

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime)

## Build

execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME} SOURCES ${FILES_LIST})

include_directories(include ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(cloth_tracker nodes/cloth_tracker.cpp src/tracker.cpp)
target_link_libraries(cloth_tracker ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(cloth_recorder nodes/cloth_recorder.cpp)
target_link_libraries(cloth_recorder ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(kinect_recorder nodes/kinect_recorder.cpp src/recorder.cpp)
target_link_libraries(kinect_recorder ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} -lzmq)

add_executable(kinect_processor nodes/kinect_processor.cpp src/processor.cpp)
target_link_libraries(kinect_processor ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(kinect_extract nodes/kinect_extract.cpp)
target_link_libraries(kinect_extract ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(feature_extract nodes/feature_extract.cpp)
target_link_libraries(feature_extract ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(marker_recorder nodes/marker_recorder.cpp)
target_link_libraries(marker_recorder ${catkin_LIBRARIES} -lzmq)

add_executable(kinect_mocap_sync nodes/kinect_mocap_sync.cpp)
target_link_libraries(kinect_mocap_sync -lzmq)

add_executable(baxter_mocap_sync nodes/baxter_mocap_sync.cpp)
target_link_libraries(baxter_mocap_sync -lzmq)

add_executable(sync nodes/sync.cpp)
target_link_libraries(sync -lzmq)

## Install
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(DIRECTORY scripts/ DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} USE_SOURCE_PERMISSIONS)
