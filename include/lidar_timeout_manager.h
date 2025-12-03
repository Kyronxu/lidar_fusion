#pragma once

#include <string>
#include <atomic>
#include <ros/ros.h>
#include "timeout_manager.h"

namespace SystemMonitor {
    // const std::string MONITOR_NAME = "/rslidar_points";
    const std::string MONITOR_NAME = "/concat_cloud";
    class LidarTimeoutManager {
        public:
            LidarTimeoutManager(const ros::NodeHandle& nh);
            ~LidarTimeoutManager() = default;

            bool Init();
            bool Start();
            void Reset(const std::string& msg_name);
            void Stop();

        private:
            void LidarTimeoutCb();

        private:
            enum Error_code_level {
                NORMAL = 0x0,
                WARNING,
                ERROR,
                FATAL
            };

            const uint16_t TIMEOUT_INTERVAL = 300; // 1000ms

        private:
            std::atomic<std::uint8_t> error_code_{0x0};
            std::atomic<bool> is_running_{true};
            ros::NodeHandle nh_;
            ros::Publisher pub_;
            SystemMonitor::TimeoutManager timeout_manager_;
    };
} // end of namespace SystemMonitor