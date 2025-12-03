#include <ros/ros.h>
#include "lidar_fusion_node.h"
#include "lidar_timeout_manager.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "lidar_fusion_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 初始化雷达超时管理器
    auto lidar_timeout_manager_ptr = std::make_shared<SystemMonitor::LidarTimeoutManager>(nh);
    if (!lidar_timeout_manager_ptr->Init()) {
        ROS_FATAL("Failed to initialize LidarTimeoutManager!");
        return 1;
    }
    if (!lidar_timeout_manager_ptr->Start()) {
        ROS_FATAL("Failed to start LidarTimeoutManager!");
        return 1;
    }

    try {
        // 创建融合节点实例
        lidar_fusion::LidarFusionNode node(nh, pnh);
        // 将超时管理器指针绑定到融合节点
        node.lidar_timeout_manager_ptr = lidar_timeout_manager_ptr;
        ROS_INFO("\033[1;32m Fusion lidars are running ======>>>  \033[0m");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Node running exception: %s", e.what());
        return 1;
    }

    // 停止超时管理器
    lidar_timeout_manager_ptr->Stop();
    return 0;
}
