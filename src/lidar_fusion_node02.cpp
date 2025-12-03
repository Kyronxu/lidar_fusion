#include "lidar_fusion_node.h"
#include "lidar_timeout_manager.h"
#include <pcl_conversions/pcl_conversions.h>
#include <limits>
#include <thread>
#include <vector>
#include <algorithm>

namespace lidar_fusion
{

    static std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
    splitCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn, int n)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> out(n);
        for (auto& pc : out) pc.reset(new pcl::PointCloud<pcl::PointXYZI>);

        float x_min = std::numeric_limits<float>::max();
        float x_max = std::numeric_limits<float>::lowest();
        for (const auto& p : cloudIn->points)
        {
            if (std::isnan(p.x)) continue;
            x_min = std::min(x_min, p.x);
            x_max = std::max(x_max, p.x);
        }
        if (x_max <= x_min) x_max = x_min + 1.0f;      // 保护
        const float bin = (x_max - x_min) / n;

        for (const auto& p : cloudIn->points)
        {
            if (std::isnan(p.x)) continue;
            int idx = static_cast<int>((p.x - x_min) / bin);
            if (idx < 0)      idx = 0;
            if (idx >= n)     idx = n - 1;
            out[idx]->push_back(p);
        }
        return out;
    }

    static pcl::PointCloud<pcl::PointXYZI>::Ptr
    mergeClouds(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clouds)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& c : clouds) *merged += *c;
        merged->width  = merged->points.size();
        merged->height = 1;
        merged->is_dense = false;
        return merged;
    }
    LidarFusionNode::LidarFusionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
    {
        loadParam();

        sub1_.subscribe(nh_, rslidar1_topic_, 10);
        sub2_.subscribe(nh_, rslidar2_topic_, 10);

        sync_ = std::make_shared<Sync>(SyncPolicy(20), sub1_, sub2_);
        sync_->registerCallback(boost::bind(&LidarFusionNode::syncCallback, this, _1, _2));

        filter_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(filter_topic_, 10);
        concat_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(concat_topic_, 10);

        ROS_INFO("Lidar fusion node initialized successfully!");
    }

    void LidarFusionNode::loadParam()
    {
        std::string config_path;
        if (!pnh_.getParam("config_path", config_path)) {
            ROS_WARN("!!! Config file path not specified! Example launch command:");
            ros::shutdown();
            return;
        }
        ROS_INFO("Successfully loaded config file: %s", config_path.c_str());

        YAML::Node config;
        try {
            config = YAML::LoadFile(config_path);
        } catch (const YAML::BadFile& e) {
            ROS_WARN("!!! Failed to read config file: %s", e.what());
            ros::shutdown();
            return;
        }

        rslidar1_topic_ = config["rslidar1_topic"].as<std::string>("/rslidar_points1");
        rslidar2_topic_ = config["rslidar2_topic"].as<std::string>("/rslidar_points2");
        filter_topic_ = config["filter_topic"].as<std::string>("/filter_cloud");
        concat_topic_ = config["concat_topic"].as<std::string>("/concat_cloud");
        filter_x_min_ = config["filter_x_min"].as<double>(0.0);

        ROS_INFO("=== Loaded Parameters ===");
        ROS_INFO("rslidar1_topic: %s", rslidar1_topic_.c_str());
        ROS_INFO("rslidar2_topic: %s", rslidar2_topic_.c_str());
        ROS_INFO("filter_topic:   %s", filter_topic_.c_str());
        ROS_INFO("concat_topic:   %s", concat_topic_.c_str());
        ROS_INFO("filter_x_min:   %.2f", filter_x_min_);

        try {
            std::vector<double> default_extrinsic = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::vector<double> ex = config["extrinsic"].as<std::vector<double>>(default_extrinsic);
            tf2::Vector3 origin(ex[0], ex[1], ex[2]);
            tf2::Quaternion q;
            q.setRPY(ex[3], ex[4], ex[5]);
            extrinsic_.setOrigin(origin);
            extrinsic_.setRotation(q);
            ROS_INFO("Extrinsic param: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f",
                    ex[0], ex[1], ex[2], ex[3], ex[4], ex[5]);
        } catch (const YAML::Exception& e) {
            ROS_WARN("Failed to read extrinsic parameters, using default values (all zeros): %s", e.what());
            extrinsic_.setIdentity();
        }
    }

    void LidarFusionNode::syncCallback(const sensor_msgs::PointCloud2::ConstPtr& msgs1, const sensor_msgs::PointCloud2::ConstPtr& msgs2)
    {
        std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

        // -------------------------- rslidar1 处理：X轴过滤 + 车体滤波 --------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msgs1, *cloud1);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered1 = filterCloud(cloud1); // 双重滤波

        // 发布rslidar1过滤后的点云
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered1, filtered_msg);
        filtered_msg.header = msgs1->header;
        filter_pub_.publish(filtered_msg);

        // -------------------------- rslidar2 处理：外参变换 + 车体滤波 --------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msgs2, *cloud2);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed2 = transformCloud(cloud2, extrinsic_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered2 = filterVehicle(transformed2);

        // -------------------------- 点云融合与发布 --------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr concat_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        *concat_cloud += *filtered1;
        *concat_cloud += *filtered2;

        sensor_msgs::PointCloud2 concat_msg;
        pcl::toROSMsg(*concat_cloud, concat_msg);
        concat_msg.header = msgs1->header;
        concat_pub_.publish(concat_msg);
        lidar_timeout_manager_ptr->Reset("/concat_cloud");

        // 打印回调耗时
        std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
        ROS_INFO("Fusion callback time: %f ms", std::chrono::duration<double, std::milli>(end_time - start_time).count());
    }

    // pcl::PointCloud<pcl::PointXYZI>::Ptr LidarFusionNode::filterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    // {
    //     // Step 1: X-axis filtering (X >= filter_x_min_)
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr x_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //     pcl::CropBox<pcl::PointXYZI> x_crop_filter;
    //     x_crop_filter.setInputCloud(cloud);
    //     x_crop_filter.setMin(Eigen::Vector4f(
    //         static_cast<float>(filter_x_min_),
    //         -std::numeric_limits<float>::max(),
    //         -std::numeric_limits<float>::max(),
    //         1.0f
    //     ));
    //     x_crop_filter.setMax(Eigen::Vector4f(
    //         std::numeric_limits<float>::max(),
    //         std::numeric_limits<float>::max(),
    //         std::numeric_limits<float>::max(),
    //         1.0f
    //     ));
    //     x_crop_filter.filter(*x_filtered_cloud);

    //     // Step 2: Vehicle filtering（调用独立函数，与rslidar2共用逻辑）
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr final_filtered_cloud = filterVehicle(x_filtered_cloud);

    //     return final_filtered_cloud;
    // }
    pcl::PointCloud<pcl::PointXYZI>::Ptr LidarFusionNode::filterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    {
        const int THREAD_N = 8;

        /* 1. 分块 */
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pieces = splitCloud(cloud, THREAD_N);

        /* 2. 并行滤波 */
        std::vector<std::thread> threads;
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> filteredPieces(THREAD_N);
        for (auto& pc : filteredPieces) pc.reset(new pcl::PointCloud<pcl::PointXYZI>);

        auto worker = [&](int id)
        {
            /* ---- X 轴裁剪 ---- */
            pcl::CropBox<pcl::PointXYZI> x_crop;
            x_crop.setInputCloud(pieces[id]);
            x_crop.setMin(Eigen::Vector4f(static_cast<float>(filter_x_min_),
                                        -std::numeric_limits<float>::max(),
                                        -std::numeric_limits<float>::max(), 1.0f));
            x_crop.setMax(Eigen::Vector4f(std::numeric_limits<float>::max(),
                                        std::numeric_limits<float>::max(),
                                        std::numeric_limits<float>::max(), 1.0f));
            pcl::PointCloud<pcl::PointXYZI>::Ptr x_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            x_crop.filter(*x_filtered);

            /* ---- 车体滤波 ---- */
            filteredPieces[id] = filterVehicle(x_filtered);
        };

        for (int i = 0; i < THREAD_N; ++i) threads.emplace_back(worker, i);
        for (auto& t : threads) t.join();

        /* 3. 合并 */
        return mergeClouds(filteredPieces);
    }

    // 独立的车体滤波函数（rslidar1和rslidar2共用）
    pcl::PointCloud<pcl::PointXYZI>::Ptr LidarFusionNode::filterVehicle(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::CropBox<pcl::PointXYZI> vehicle_crop;
        
        Eigen::Vector3f vehicle_translation = {0.0f, 0.5f, -1.0f};  // 相对于激光雷达帧的车体平移
        Eigen::Vector4f vehicle_max_pt = {5.0f, 2.0f, 2.0f, 1.0f};  // 车体半长（x±5, y±2, z±2）
        Eigen::Vector4f vehicle_min_pt = -vehicle_max_pt;           // 对称范围
        Eigen::Quaternionf vehicle_quat = {1.0f, 0.0f, 0.0f, 0.0f}; // 无旋转（单位四元数）
        Eigen::Vector3f vehicle_rotation = vehicle_quat.matrix().eulerAngles(0, 1, 2); // 滚转/俯仰/偏航

        // 设置车体滤波配置
        vehicle_crop.setInputCloud(cloud);
        vehicle_crop.setMin(vehicle_min_pt);
        vehicle_crop.setMax(vehicle_max_pt);
        vehicle_crop.setTranslation(vehicle_translation);
        vehicle_crop.setRotation(vehicle_rotation);
        vehicle_crop.setNegative(true);          // 剔除车体内的点（保留外部点）
        vehicle_crop.setKeepOrganized(false);    // 不保留有序结构（节省内存）
        vehicle_crop.filter(*filtered_cloud);

        return filtered_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr LidarFusionNode::transformCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const tf2::Transform& tf)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
        transformed->reserve(cloud->size());

        const tf2::Vector3& origin = tf.getOrigin();
        const tf2::Matrix3x3 rot(tf.getRotation());

        for (const auto& p : cloud->points) {
            if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
                continue;
            }
            tf2::Vector3 v(p.x, p.y, p.z);
            v = rot * v + origin;

            pcl::PointXYZI np;
            np.x = v.x();
            np.y = v.y();
            np.z = v.z();
            np.intensity = p.intensity;
            transformed->points.push_back(np);
        }

        transformed->width = transformed->points.size();
        transformed->height = 1;
        transformed->is_dense = false;
        transformed->header = cloud->header;

        return transformed;
    }
} // namespace lidar_fusion

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "lidar_fusion_node");
//     ros::NodeHandle nh;
//     ros::NodeHandle pnh("~");
//     lidar_timeout_manager_ptr = std::make_shared<SystemMonitor::LidarTimeoutManager>(nh);
//     lidar_timeout_manager_ptr->Init();
//     lidar_timeout_manager_ptr->Start();

//     try {
//         lidar_fusion::LidarFusionNode node(nh, pnh);
//         std::cout << "\033[1;32m Fusion lidars for running ======>>>  \033[0m" << std::endl;
//         ros::spin();
//     } catch (const std::exception& e) {
//         ROS_FATAL("Node running exception: %s", e.what());
//         return 1;
//     }

//     return 0;
// }
