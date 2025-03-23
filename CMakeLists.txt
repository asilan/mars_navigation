cmake_minimum_required(VERSION 3.0.2)
project(mars_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mars_canopen
  message_generation
)

# 添加消息文件
add_message_files(
  FILES
  Waypoint.msg
)

# 添加服务文件
add_service_files(
  FILES
  UploadMission.srv
  EmergencyStop.srv
)

# 添加动作文件
add_action_files(
  FILES
  ExecuteMission.action
  NavigateToTarget.action
)

# 生成消息依赖
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

# 声明catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    actionlib
    actionlib_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    mars_canopen
    message_runtime
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 添加库目标
add_library(${PROJECT_NAME}
  src/mars_navigation_node.cpp
)

# 添加可执行目标
add_executable(${PROJECT_NAME}_node
  src/mars_navigation_node.cpp
  src/main.cpp
)

# 添加依赖
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 安装目标
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)