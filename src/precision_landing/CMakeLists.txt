cmake_minimum_required(VERSION 3.0.2)
project(precision_landing)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Eigen3 REQUIRED)
set(OpenCV_DIR "/usr/local/opencv-455-cuda/lib/cmake/opencv4")
find_package(OpenCV 4.5.5 EXACT REQUIRED COMPONENTS aruco calib3d core imgproc)
if(NOT OpenCV_VERSION VERSION_EQUAL "4.5.5")
  message(FATAL_ERROR "precision_landing requires OpenCV 4.5.5, found ${OpenCV_VERSION}")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs roscpp sensor_msgs std_msgs
  DEPENDS EIGEN3 OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(precision_landing_core
  src/controller_core.cpp
  src/landing_target_anchor.cpp)
target_link_libraries(precision_landing_core ${catkin_LIBRARIES})

add_library(precision_landing_px4_native_core src/px4_landing_target_core.cpp)
target_link_libraries(precision_landing_px4_native_core ${catkin_LIBRARIES})

add_library(precision_landing_vision_geometry src/board_pose_estimator.cpp)
target_link_libraries(precision_landing_vision_geometry ${OpenCV_LIBRARIES})

add_executable(precision_landing_node src/precision_landing_node.cpp)
target_link_libraries(precision_landing_node precision_landing_core ${catkin_LIBRARIES})

add_executable(px4_landing_target_node src/px4_landing_target_node.cpp)
target_link_libraries(px4_landing_target_node
  precision_landing_px4_native_core ${catkin_LIBRARIES})

add_executable(precision_landing_debug_overlay src/debug_overlay_node.cpp)
target_link_libraries(precision_landing_debug_overlay
  precision_landing_vision_geometry ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
set_target_properties(precision_landing_debug_overlay PROPERTIES
  BUILD_RPATH "/usr/local/opencv-455-cuda/lib"
  INSTALL_RPATH "/usr/local/opencv-455-cuda/lib")

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_landing_accuracy test/test_landing_accuracy.cpp)
  target_link_libraries(test_landing_accuracy ${catkin_LIBRARIES} gtest_main)
  catkin_add_gtest(test_px4_landing_target_core test/test_px4_landing_target_core.cpp)
  target_link_libraries(test_px4_landing_target_core precision_landing_px4_native_core ${catkin_LIBRARIES} gtest_main)
  catkin_add_gtest(test_controller_core test/test_controller_core.cpp)
  target_link_libraries(test_controller_core precision_landing_core ${catkin_LIBRARIES} gtest_main)
  catkin_add_gtest(test_landing_target_anchor test/test_landing_target_anchor.cpp)
  target_link_libraries(test_landing_target_anchor precision_landing_core ${catkin_LIBRARIES} gtest_main)
  catkin_add_gtest(test_heading_from_board test/test_heading_from_board.cpp)
  target_link_libraries(test_heading_from_board ${catkin_LIBRARIES} gtest_main)
  catkin_add_gtest(test_board_geometry test/test_board_geometry.cpp)
  target_link_libraries(test_board_geometry ${catkin_LIBRARIES} gtest_main)
  catkin_add_gtest(test_board_pose_estimator test/test_board_pose_estimator.cpp)
  target_link_libraries(test_board_pose_estimator precision_landing_vision_geometry ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtest_main)
endif()
