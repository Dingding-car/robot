cmake_minimum_required(VERSION 3.10)

# 项目名称
project(RobotControl)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${PROJECT_SOURCE_DIR}/include)


aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_LIST)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# 添加可执行文件
add_executable(
    robot_control
    ${SRC_LIST}
)


# 链接库
target_link_libraries(robot_control pthread)
