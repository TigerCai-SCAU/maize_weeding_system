cmake_minimum_required(VERSION 3.10)
project(calib_axxb)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)

add_executable(calib_axxb_lidar_rtk
  app/calib_lidar_rtk.cpp
  src/hand_eye_calibrator.cpp
)

target_include_directories(calib_axxb_lidar_rtk PRIVATE
  include
  ${CERES_INCLUDE_DIRS}
)

target_link_libraries(calib_axxb_lidar_rtk
  Eigen3::Eigen
  yaml-cpp
  ${CERES_LIBRARIES}
)

install(TARGETS calib_axxb_lidar_rtk
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config data
  DESTINATION share/${PROJECT_NAME}
)

install(FILES readme README_ROS2.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
