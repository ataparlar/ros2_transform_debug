//
// Created by ataparlar on 21.08.2023.
//

#include "data_provider_pkg/lio_sam_imu_transform.hpp"
//#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>

LioSamImuTransform::LioSamImuTransform() :
    Node("lio_sam_imu_transform") {
//    auto qos = rclcpp::QoS(
//            rclcpp::QoSInitialization(
//                    qos_profile.history,
//                    qos_profile.depth
//            ),
//            qos_profile);


//    lidar_subscription_.subscribe(this, "/hesai/pandar");
    odom_subscription_.subscribe(this, "/applanix/lvx_client/odom");
    imu_subscription_.subscribe(this, "/applanix/lvx_client/imu_raw");
    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/hesai/pandar", 10, std::bind(&LioSamImuTransform::lidar_callback,
                                   this, std::placeholders::_1));

    lidar_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_transform", 10);

    sync_.reset(new Sync(approximate_policy(100),
                         odom_subscription_,imu_subscription_));

    sync_->registerCallback(
            std::bind(&LioSamImuTransform::map_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

    base_link_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    lidar_link_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
};


void LioSamImuTransform::map_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
                                      const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg) {
    geometry_msgs::msg::TransformStamped transformStamped;

    tf2::Quaternion q_orig, q_rot, q_rot_x, q_rot_y, q_rot_z, q_rot2, q_new;
    q_orig.setX(imu_msg->orientation.x);
    q_orig.setY(imu_msg->orientation.y);
    q_orig.setZ(imu_msg->orientation.z);
    q_orig.setW(imu_msg->orientation.w);
//    q_orig = q_orig.inverse();

    q_rot.setRPY(3.14159265, 0.0, 0.0);
//    q_rot.setRPY(3.14159265, 0.0, -1.57079633);
//    q_rot_x.setRPY(3.14159265, 0.0, 0.0); // NED -> ENU
//    q_rot_y.setRPY(0.0, 0.0, 0.0); // NED -> ENU
//    q_rot_z.setRPY(0.0, 0.0, -1.57079633); // NED -> ENU
//    q_rot.setRPY(0.0, 0.0, -1.57079633);

//    q_rot2.setRPY(0.0, 0.0,1.57079633);
//    q_rot2.setRPY(3.14159265, 0.0,0.0);

    q_new = q_orig * q_rot; // * q_rot2;
//    q_new = q_orig * q_rot_z; // * q_rot2;
//    q_new = q_new * q_rot_y;
//    q_new = q_new * q_rot_x;
    q_new.normalize();



//    Eigen::Quaterniond imu_quat(imu_msg->orientation.w, imu_msg->orientation.x,
//                                       imu_msg->orientation.y, imu_msg->orientation.z);
//    Eigen::Affine3d ned2enu(Eigen::Affine3d::Identity());
//    ned2enu.matrix().topLeftCorner<3, 3>() =
//            Eigen::AngleAxisd(-1.57079633, Eigen::Vector3d::UnitZ())
//                    .toRotationMatrix() *
//            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
//                    .toRotationMatrix() *
//            Eigen::AngleAxisd(14159265, Eigen::Vector3d::UnitX())
//                    .toRotationMatrix();
//    Eigen::Affine3d new_rot(Eigen::Affine3d::Identity());
//    new_rot = imu_quat.toRotationMatrix() * ned2enu.rotation();




    sensor_msgs::msg::Imu imu;
    imu.orientation.x = q_new.x();
    imu.orientation.y = q_new.y();
    imu.orientation.z = q_new.z();
    imu.orientation.w = q_new.w();
    imu.orientation_covariance = imu_msg->orientation_covariance;

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
    transformStamped.transform.translation.x = odom_msg->pose.pose.position.x;
    transformStamped.transform.translation.y = odom_msg->pose.pose.position.y;
    transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
    base_link_tf_broadcaster_->sendTransform(transformStamped);
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
    q_lidar.setRPY(3.14159265, 0.0, 3.14159265);

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
    lidar_link_tf_broadcaster_->sendTransform(lidar_transform_stamped);

    lidar_publisher_->publish(cloud);


}



int main(int argc, char * argv[])
{
    rclcpp::QoS qos_profile(rmw_qos_profile_t rmw_qos_profile_default);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LioSamImuTransform>());
    rclcpp::shutdown();
    return 0;
}