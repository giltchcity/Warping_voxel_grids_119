








cmake_minimum_required(VERSION 3.10)
project(CeresExample)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必要的包
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# 正确设置Sophus路径
include_directories(
    /ORB_SLAM3/Thirdparty/Sophus
)

# 添加可执行文件
add_executable(ceres_example main.cpp)

# 设置包含目录
target_include_directories(ceres_example 
    PRIVATE 
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# 链接库
target_link_libraries(ceres_example 
    ${CERES_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
