cmake_minimum_required(VERSION 3.0.2)
project(lidar_fusion)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        pcl_ros
        tf2
        tf2_ros
        tf2_geometry_msgs
        # pcl_conversions
        message_filters
)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(yaml-cpp REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp 
        sensor_msgs 
        pcl_ros 
        tf2 
        tf2_ros 
        tf2_geometry_msgs 
        message_filters
)

# 包含头文件路径
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
)

# 链接库路径
link_directories(
        ${PCL_LIBRARY_DIRS}
        ${YAML_CPP_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# 生成可执行文件
add_executable(lidar_fusion_node 
                src/lidar_fusion_node.cpp 
                src/main.cpp 
                src/lidar_timeout_manager.cpp)
target_link_libraries(lidar_fusion_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
)

# # 安装规则（可选，方便部署）
# install(TARGETS lidar_fusion_node
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
