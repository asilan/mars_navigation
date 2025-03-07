cmake_minimum_required(VERSION 3.0.2)
project(mars_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    sensor_msgs 
    actionlib 
    actionlib_msgs 
    tf2 
    tf2_ros 
    tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 复制conversions.h到include目录
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/conversions.h 
     DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/include/mars_navigation)

# 添加服务文件
add_service_files(
  FILES
    UploadMission.srv
    NavigateToTarget.srv
    EmergencyStop.srv
)

# 添加action文件
add_action_files(
  FILES
    ExecuteMission.action
    NavigateToTarget.action
)

# 生成消息和服务
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

# 添加可执行文件
add_executable(${PROJECT_NAME}_node
  src/mars_navigation_node.cpp
)

# 添加依赖
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 链接库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)