//
// Created by ataparlar on 21.08.2023.
//

#include "data_provider_pkg/lio_sam_imu_transform.hpp"
//#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Matrix3x3.h>

LioSamImuTransform::LioSamImuTransform() :
    Node("lio_sam_imu_transform") {
//    auto qos = rclcpp::QoS(
//            rclcpp::QoSInitialization(
//                    qos_profile.history,
//                    qos_profile.depth
//            ),
//            qos_profile);


//    lidar_subscription_.subscribe(this, "/hesai/pandar");
//    odom_subscription_.subscribe(this, "/applanix/lvx_client/odom");
//    odom_subscription_.subscribe(this, "/applanix/lvx_client/odom");
    odom_subscription_.subscribe(this, "/sensing/lidar/top/mirror_cropped/pointcloud_ex", qos_profile);
    imu_subscription_.subscribe(this, "/sensing/gnss/clap/ros/imu");
    navsatfix_subscription_.subscribe(this, "/sensing/gnss/clap/ros/gps_nav_sat_fix");
    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/hesai/pandar", 10, std::bind(&LioSamImuTransform::lidar_callback,
                                   this, std::placeholders::_1));
    navsatfix_subscription_x = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/sensing/gnss/clap/ros/gps_nav_sat_fix", 10, std::bind(&LioSamImuTransform::navsatfix_callback,
                                   this, std::placeholders::_1));

    lidar_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_transform", 10);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_transform", 10);
    odom_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("odom_path", 10);
    navsatfix_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("navsatfix_path", 10);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("navsatfix_odom", 10);

    sync_.reset(new Sync(approximate_policy(100),
                         odom_subscription_,imu_subscription_, navsatfix_subscription_));

    sync_->registerCallback(
            std::bind(&LioSamImuTransform::map_callback, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    base_link_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    lidar_link_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
};


void LioSamImuTransform::map_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
                                      const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg,
                                      const sensor_msgs::msg::NavSatFix::ConstSharedPtr & navsatfix_msg) {
    if (!locart_init) {
        localCartesian.Reset(
                navsatfix_msg->latitude, navsatfix_msg->longitude, navsatfix_msg->altitude);
        locart_init = true;
    }
    else {
        double local_x, local_y, local_z;
        localCartesian.Forward(
                navsatfix_msg->latitude, navsatfix_msg->longitude, navsatfix_msg->altitude,
                local_x, local_y, local_z
        );
        geometry_msgs::msg::PoseStamped pose_;
        pose_.header.frame_id = "map";
        pose_.header.stamp = LioSamImuTransform::get_clock()->now();
        pose_.pose.position.x = local_x;
        pose_.pose.position.y = local_y;
        pose_.pose.position.z = local_z;
        pose_.pose.orientation.x = imu_msg->orientation.x;
        pose_.pose.orientation.y = imu_msg->orientation.y;
        pose_.pose.orientation.z = imu_msg->orientation.z;
        pose_.pose.orientation.w = imu_msg->orientation.w;
        navsatfix_poses_.header.frame_id = "map";
        navsatfix_poses_.header.stamp = LioSamImuTransform::get_clock()->now();
        navsatfix_poses_.poses.push_back(pose_);
        navsatfix_path_publisher_->publish(navsatfix_poses_);




        geometry_msgs::msg::TransformStamped transformStamped;

        tf2::Quaternion q_orig, q_rot, q_new;
        q_orig.setX(imu_msg->orientation.x);
        q_orig.setY(imu_msg->orientation.y);
        q_orig.setZ(imu_msg->orientation.z);
        q_orig.setW(imu_msg->orientation.w);
//        q_orig = q_orig.inverse();

        q_rot.setRPY(0.0, 0.0, 0.0);

        q_new = q_orig * q_rot;
        tf2::Matrix3x3 m(q_new);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
//        q_new.setRPY(pitch, roll, -yaw);
        q_new.setRPY(roll, pitch, yaw);
        q_new.normalize();

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = LioSamImuTransform::get_clock()->now();
        pose.pose.position = odom_msg->pose.pose.position;
        pose.pose.orientation = imu_msg->orientation;

        odom_poses_.header.frame_id = "map";
        odom_poses_.header.stamp = LioSamImuTransform::get_clock()->now();
        odom_poses_.poses.push_back(pose);
        odom_path_publisher_->publish(odom_poses_);


        sensor_msgs::msg::Imu imu;
        imu.header.frame_id = "base_link";
        imu.header.stamp = this->get_clock()->now();
        imu.orientation.x = q_new.x();
        imu.orientation.y = q_new.y();
        imu.orientation.z = q_new.z();
        imu.orientation.w = q_new.w();
        imu.linear_acceleration.x = imu_msg->linear_acceleration.y;
        imu.linear_acceleration.y = imu_msg->linear_acceleration.x;
        imu.linear_acceleration.z = -imu_msg->linear_acceleration.z;
        imu.angular_velocity.x = imu_msg->angular_velocity.y;
        imu.angular_velocity.y = imu_msg->angular_velocity.x;
        imu.angular_velocity.z = -imu_msg->angular_velocity.z;
        imu.orientation_covariance = imu_msg->orientation_covariance;
        imu_publisher_->publish(imu);

        nav_msgs::msg::Odometry odom_msg_;
        odom_msg_.header.stamp = this->get_clock()->now();
        odom_msg_.header.frame_id = "map";
        odom_msg_.child_frame_id = "";
        odom_msg_.pose.pose.position = pose_.pose.position;
        odom_msg_.pose.pose.orientation = imu.orientation;
        odom_publisher_->publish(odom_msg_);

//    imu.angular_velocity.x = imu_msg->angular_velocity.y;
//    imu.angular_velocity.y = imu_msg->angular_velocity.x;
//    imu.angular_velocity.z = - imu_msg->angular_velocity.z;
//    imu.angular_velocity_covariance = msg->angular_velocity_covariance;

//    imu.linear_acceleration.x = imu_msg->linear_acceleration.y;
//    imu.linear_acceleration.y = imu_msg->linear_acceleration.x;
//    imu.linear_acceleration.z = - imu_msg->linear_acceleration.z;
//    imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;

        transformStamped.header.stamp = LioSamImuTransform::get_clock()->now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.rotation.x = q_new.x();
        transformStamped.transform.rotation.y = q_new.y();
        transformStamped.transform.rotation.z = q_new.z();
        transformStamped.transform.rotation.w = q_new.w();
        transformStamped.transform.translation.x = local_x;
        transformStamped.transform.translation.y = local_y;
        transformStamped.transform.translation.z = local_z;
//        base_link_tf_broadcaster_->sendTransform(transformStamped);
    }
}


void LioSamImuTransform::lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_msg) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = LioSamImuTransform::get_clock()->now();
    cloud.header.frame_id = "lidar_link";
    cloud.data = lidar_msg->data;
    cloud.fields = lidar_msg->fields;
    cloud.height = lidar_msg->height;
    cloud.width = lidar_msg->width;
    cloud.is_bigendian = lidar_msg->is_bigendian;
    cloud.is_dense = lidar_msg->is_dense;
    cloud.point_step = lidar_msg->point_step;
    cloud.row_step = lidar_msg->row_step;


    tf2::Quaternion q_lidar;
//    q_lidar.setRPY(3.14159265, 0.0, 3.14159265);
    q_lidar.setRPY(0.0, 0.0, -1.57079633);

    geometry_msgs::msg::TransformStamped lidar_transform_stamped;
    lidar_transform_stamped.header.stamp = LioSamImuTransform::get_clock()->now();
    lidar_transform_stamped.header.frame_id = "base_link";
    lidar_transform_stamped.child_frame_id = "lidar_link";
    lidar_transform_stamped.transform.rotation.x = q_lidar.x();
    lidar_transform_stamped.transform.rotation.y = q_lidar.y();
    lidar_transform_stamped.transform.rotation.z = q_lidar.z();
    lidar_transform_stamped.transform.rotation.w = q_lidar.w();
    lidar_transform_stamped.transform.translation.x = 0.0;
    lidar_transform_stamped.transform.translation.y = 0.0;
    lidar_transform_stamped.transform.translation.z = -1.0;
//    lidar_link_tf_broadcaster_->sendTransform(lidar_transform_stamped);
    lidar_publisher_->publish(cloud);
}


void LioSamImuTransform::navsatfix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &navsatfix_msg) {
    nav_msgs::msg::Odometry odom_msg;
    if (!locart_init) {
        localCartesian.Reset(
                navsatfix_msg->latitude, navsatfix_msg->longitude, navsatfix_msg->altitude);
        locart_init = true;
    }
    else {
        double local_x, local_y, local_z;
        localCartesian.Forward(
                navsatfix_msg->latitude, navsatfix_msg->longitude, navsatfix_msg->altitude,
                local_x, local_y, local_z
                );
        odom_msg.header.frame_id = "map";
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.pose.pose.position.x = local_x;
        odom_msg.pose.pose.position.y = local_y;
        odom_msg.pose.pose.position.z = local_z;
        odom_publisher_->publish(odom_msg);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = LioSamImuTransform::get_clock()->now();
        pose.pose.position.x = local_x;
        pose.pose.position.y = local_y;
        pose.pose.position.z = local_z;
        navsatfix_poses_.header.frame_id = "map";
        navsatfix_poses_.header.stamp = LioSamImuTransform::get_clock()->now();
        navsatfix_poses_.poses.push_back(pose);
//        std::cout << "navsatfix_odom published" << std::endl;
        navsatfix_path_publisher_->publish(navsatfix_poses_);
    }
}



int main(int argc, char * argv[])
{
    rclcpp::QoS qos_profile(rmw_qos_profile_s rmw_qos_profile_sensor_data);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LioSamImuTransform>());
    rclcpp::shutdown();
    return 0;
}