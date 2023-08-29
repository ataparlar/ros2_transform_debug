//
// Created by ataparlar on 03.08.2023.
//

#ifndef BUILD_DATA_PROVIDER_NODE_HPP
#define BUILD_DATA_PROVIDER_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"

class TfPublisher : public rclcpp::Node
{
public:
    TfPublisher();
private:
    typedef message_filters::sync_policies::ApproximateTime<
            geometry_msgs::msg::PoseWithCovarianceStamped, geometry_msgs::msg::PoseWithCovarianceStamped>
            approximate_policy;

    // made a synchronizer type which uses approximate policy
    typedef message_filters::Synchronizer<approximate_policy> Sync;

    std::shared_ptr<Sync> sync_;

    void tf_callback(const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg,
                     const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
                     const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg1,
                     const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg2);

    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subscription_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscription_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_subscription_;
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> pose1_subscription_;
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> pose2_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};



#endif //BUILD_DATA_PROVIDER_NODE_HPP
