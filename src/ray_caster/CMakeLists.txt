cmake_minimum_required(VERSION 3.0)
project(ray_casting_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rosbag roscpp cv_bridge sensor_msgs std_msgs
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}  # 找到 happly.h
)

if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_executable(ray_caster main.cpp)
target_link_libraries(ray_caster
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

include_directories(${CMAKE_SOURCE_DIR}/nanoflann/include)
