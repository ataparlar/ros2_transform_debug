#include <chrono>
#include <functional>
#include <memory>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "message_filters/subscriber.h"
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "data_provider_pkg/data_provider_node.hpp"
#include <Eigen/Geometry>

TfPublisher::TfPublisher() : Node("tf_publisher")
{
    odom_subscription_.subscribe(this, "/applanix/lvx_client/odom");
    imu_subscription_.subscribe(this, "/applanix/lvx_client/imu_raw");
    lidar_subscription_.subscribe(this, "/hesai/pandar");
    pose1_subscription_.subscribe(this, "/localization/pose_twist_fusion_filter/biased_pose_with_covariance");
    pose2_subscription_.subscribe(this, "/sensing/gnss/pose_with_covariance");
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose_difference", 10);

    sync_.reset(new Sync(approximate_policy(100),
                    TfPublisher::pose1_subscription_, TfPublisher::pose2_subscription_));

    sync_->registerCallback(
            std::bind(&TfPublisher::pose_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
//    tf_broadcaster_ =
//            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
};

void TfPublisher::tf_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
                              const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                              const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_msg) {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = imu_msg->header.stamp;
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "map";


    tf2::Quaternion q(
            imu_msg->orientation.x,
            imu_msg->orientation.y,
            imu_msg->orientation.z,
            imu_msg->orientation.w);
//    tf2::Quaternion q_inv = q.inverse();
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    Eigen::AngleAxisd angle_axis_x(roll*M_PI/180, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd angle_axis_y(pitch*M_PI/180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd angle_axis_z((yaw*M_PI/180), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_ = (angle_axis_x * angle_axis_y * angle_axis_z);
    // take the inverse of the quaternion
    Eigen::Quaterniond q_inv = q_.inverse();

    transformStamped.transform.translation.x = odom_msg->pose.pose.position.x;
    transformStamped.transform.translation.y = odom_msg->pose.pose.position.y;
    transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = imu_msg->orientation.x;
    transformStamped.transform.rotation.y = imu_msg->orientation.y;
    transformStamped.transform.rotation.z = imu_msg->orientation.z;
    transformStamped.transform.rotation.w = imu_msg->orientation.w;
//    transformStamped.transform.rotation.set__x(q_.x());
//    transformStamped.transform.rotation.set__y(q_.y());
//    transformStamped.transform.rotation.set__z(q_.z());
//    transformStamped.transform.rotation.set__w(q_.w());
    tf_broadcaster_->sendTransform(transformStamped);

    std::cout << "Time difference betweeen imu_msg and lidar_msg in seconds: " <<
    imu_msg->header.stamp.sec - lidar_msg->header.stamp.sec << std::endl;
}

void TfPublisher::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg1,
                   const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg2) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = pose_msg1->header.stamp;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.pose.position.x = pose_msg1->pose.pose.position.x - pose_msg2->pose.pose.position.x;
    pose_msg.pose.pose.position.y = pose_msg1->pose.pose.position.y - pose_msg2->pose.pose.position.y;
    pose_msg.pose.pose.position.z = pose_msg1->pose.pose.position.z - pose_msg2->pose.pose.position.z;
    pose_msg.pose.pose.orientation.x = pose_msg1->pose.pose.orientation.x;
    pose_msg.pose.pose.orientation.y = pose_msg1->pose.pose.orientation.y;
    pose_msg.pose.pose.orientation.z = pose_msg1->pose.pose.orientation.z;
    pose_msg.pose.pose.orientation.w = pose_msg1->pose.pose.orientation.w;
//    std::cout << "eucladean distance between two poses: " << sqrt(pow(pose_msg.pose.pose.position.x, 2) +
//            pow(pose_msg.pose.pose.position.y, 2) + pow(pose_msg.pose.pose.position.z, 2)) << std::endl;
    std::cout << "eucladean distance between two poses in x and y: " << sqrt(pow(pose_msg.pose.pose.position.x, 2) +
            pow(pose_msg.pose.pose.position.y, 2)) << std::endl;
    publisher_->publish(pose_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfPublisher>());
    rclcpp::shutdown();
    return 0;
}