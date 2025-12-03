#include "lidar_timeout_manager.h"

#include <ros/ros.h>
#include <thread>
#include <std_msgs/UInt8.h>

namespace SystemMonitor {

// #define MY_DEBUG

LidarTimeoutManager::LidarTimeoutManager(const ros::NodeHandle& nh) : nh_(nh) {
}

bool LidarTimeoutManager::Init() {
    ROS_INFO("LidarTimeoutManager::Init");
    pub_ = nh_.advertise<std_msgs::UInt8>("/rslidar/dataTimeout", 5);

    return true;
}

bool LidarTimeoutManager::Start() {
//     ROS_INFO("LidarTimeoutManager::Start++");
//     timeout_manager_.RegisterTopic(MONITOR_NAME, std::chrono::milliseconds(TIMEOUT_INTERVAL),
//         [this]() { this->LidarTimeoutCb();}
//     );

//     std::thread([this] {
//         ros::Rate rate(10);
//         while (is_running_.load()) {
//             std_msgs::UInt8 msg;
//             msg.data = error_code_.load();
//             pub_.publish(msg);
// #ifdef MY_DEBUG
//             ROS_INFO("pub data:%d", msg.data);
// #endif
//             rate.sleep();
//         }
//     }).detach();

//     ROS_INFO("LidarTimeoutManager::Start--");

//     return true;
    ROS_INFO("LidarTimeoutManager::Start++");
    timeout_manager_.RegisterTopic(
        MONITOR_NAME, 
        std::chrono::milliseconds(TIMEOUT_INTERVAL),
        // 新增 size_t 参数占位符（/*trigger_count*/ 表示该参数暂不使用）
        [this](size_t /*trigger_count*/) { this->LidarTimeoutCb(); }
    );

    std::thread([this] {
        ros::Rate rate(10);
        while (is_running_.load()) {
            std_msgs::UInt8 msg;
            msg.data = error_code_.load();
            pub_.publish(msg);
    #ifdef MY_DEBUG
                ROS_INFO("pub data:%d", msg.data);
    #endif
                rate.sleep();
            }
        }).detach();

        ROS_INFO("LidarTimeoutManager::Start--");
    return true;
}

void LidarTimeoutManager::Reset(const std::string& msg_name) {
    // ROS_INFO("Reset, msg_name: %s", msg_name.c_str());
    timeout_manager_.Reset(msg_name);
#ifdef MY_DEBUG
    ROS_INFO("msg_name:%s, value:0", msg_name.c_str());
#endif
    error_code_.store(0x0);
}

void LidarTimeoutManager::Stop() {
    is_running_.store(false);
}

void LidarTimeoutManager::LidarTimeoutCb() {
    ROS_INFO("Lidar Timeout");
    error_code_.store(Error_code_level::ERROR);
}
} // end of namespace SystemMonitor