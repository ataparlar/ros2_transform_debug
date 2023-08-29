//
// Created by ataparlar on 21.08.2023.
//

#ifndef BUILD_LIO_SAM_IMU_TRANSFORM_HPP
#define BUILD_LIO_SAM_IMU_TRANSFORM_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "message_filters/subscriber.h"
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"

class LioSamImuTransform : public rclcpp::Node
{
public:
    LioSamImuTransform();
private:
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    typedef message_filters::sync_policies::ApproximateTime<
            nav_msgs::msg::Odometry,
            sensor_msgs::msg::Imu>
            approximate_policy;

    // made a synchronizer type which uses approximate policy
    typedef message_filters::Synchronizer<approximate_policy> Sync;

    std::shared_ptr<Sync> sync_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subscription_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscription_;
//    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_subscription_;


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher_;
//    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
//    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscribtion_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> base_link_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> lidar_link_tf_broadcaster_;

    void map_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
                      const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg);
    void lidar_callback(const sensor_msgs::msg::PointCloud2 ::ConstSharedPtr & lidar_msg);
};




#endif //BUILD_LIO_SAM_IMU_TRANSFORM_HPP
