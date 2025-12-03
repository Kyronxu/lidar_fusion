#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2/LinearMath/Transform.h>
#include "lidar_timeout_manager.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <pcl/filters/crop_box.h>
#include <limits>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace lidar_fusion
{
    class LidarFusionNode
    {
        public:
            LidarFusionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
            ~LidarFusionNode() = default;
            std::shared_ptr<SystemMonitor::LidarTimeoutManager> lidar_timeout_manager_ptr;

        private:
            void loadParam();
            void syncCallback(const sensor_msgs::PointCloud2::ConstPtr& msgs1, const sensor_msgs::PointCloud2::ConstPtr& msgs2);

            // 主过滤函数（X轴+车体，用于rslidar1）
            pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
            // 独立的车体滤波函数（用于rslidar2，也被filterCloud调用）
            pcl::PointCloud<pcl::PointXYZI>::Ptr filterVehicle(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
            // 点云变换函数
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const tf2::Transform& tf);

            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;

            message_filters::Subscriber<sensor_msgs::PointCloud2> sub1_;
            message_filters::Subscriber<sensor_msgs::PointCloud2> sub2_;

            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
            typedef message_filters::Synchronizer<SyncPolicy> Sync;
            std::shared_ptr<Sync> sync_;

            ros::Publisher filter_pub_;
            ros::Publisher concat_pub_;

            std::string rslidar1_topic_;
            std::string rslidar2_topic_;
            std::string filter_topic_;
            std::string concat_topic_;

            double filter_x_min_;
            tf2::Transform extrinsic_;
            
    };
} // namespace lidar_fusion
