cmake_minimum_required(VERSION 3.10)

# 项目名称
project(RobotControl)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加可执行文件
add_executable(robot_control
    main.cpp
    base/SerialCom.cpp
    base/VoyCmd.cpp
)

# 链接库
target_link_libraries(robot_control pthread)
