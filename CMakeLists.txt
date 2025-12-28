cmake_minimum_required(VERSION 3.0.2)
project(rs06_ros)

## 使用 ROS1 catkin
find_package(catkin REQUIRED)
find_package(Threads REQUIRED)

catkin_package()

## 编译选项
add_definitions(-std=c++14)

include_directories(
	${catkin_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/include
)

## 源文件列表（核心库部分）
set(RS06_SOURCES
	src/rs06.cpp
	src/socketcan_driver.cpp
)

## 测试节点：test_motor
add_executable(test_motor
	src/test_motor.cpp
	${RS06_SOURCES}
)

target_link_libraries(test_motor
	${catkin_LIBRARIES}
	Threads::Threads
)

## 测试节点：test_socketcan
add_executable(test_socketcan
	src/test_socketcan.cpp
	src/socketcan_driver.cpp
)

target_link_libraries(test_socketcan
	${catkin_LIBRARIES}
	Threads::Threads
)

## 如果需要安装，可按需添加 install 规则
# install(TARGETS test_motor
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

