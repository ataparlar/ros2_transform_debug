//
// Created by ataparlar on 06.09.2023.
//

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "data_provider_pkg/transform_visualizer.hpp"

TransformVisualizer::TransformVisualizer() :
        Node("transform_visualizer") {

    this->declare_parameter("imu_r_x", 0.0);
    this->declare_parameter("imu_r_y", 0.0);
    this->declare_parameter("imu_r_z", 0.0);
    this->declare_parameter("imu2lidar_x", 0.0);
    this->declare_parameter("imu2lidar_y", 0.0);
    this->declare_parameter("imu2lidar_z", 0.0);
    this->declare_parameter("imu_topic", "");
    this->declare_parameter("odom_topic", "");
    this->declare_parameter("lidar_topic", "");
    this->declare_parameter("navsatfix_topic", "");
    this->declare_parameter("publish_tf", true);
    this->declare_parameter("enable_ned2enu", false);
    this->declare_parameter("approximate_time", false);

    imu_roll = this->get_parameter("imu_r_x").as_double();
    imu_pitch = this->get_parameter("imu_r_y").as_double();
    imu_yaw = this->get_parameter("imu_r_z").as_double();
    imu2lidar_roll = this->get_parameter("imu2lidar_x").as_double();
    imu2lidar_pitch = this->get_parameter("imu2lidar_y").as_double();
    imu2lidar_yaw = this->get_parameter("imu2lidar_z").as_double();
    imu_topic = this->get_parameter("imu_topic").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    lidar_topic = this->get_parameter("lidar_topic").as_string();
    navsatfix_topic = this->get_parameter("navsatfix_topic").as_string();
    publish_tf = this->get_parameter("publish_tf").as_bool();
    enable_ned2enu = this->get_parameter("enable_ned2enu").as_bool();
    approximate_time = this->get_parameter("approximate_time").as_bool();


    if (approximate_time) {
        approx_imu_subscription_.subscribe(this, imu_topic);
        approx_odom_subscription_.subscribe(this, odom_topic);
        approx_pointcloud_subscription_.subscribe(this, lidar_topic);
        approx_navsatfix_subscription_.subscribe(this, navsatfix_topic);
    } else {
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic, 10, std::bind(&TransformVisualizer::imu_callback,
                                         this, std::placeholders::_1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic, 10, std::bind(&TransformVisualizer::odom_callback,
                                          this, std::placeholders::_1));
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lidar_topic, 10, std::bind(&TransformVisualizer::lidar_callback,
                                           this, std::placeholders::_1));
        navsatfix_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                navsatfix_topic, 10, std::bind(&TransformVisualizer::navsatfix_callback,
                                               this, std::placeholders::_1));
    }


    if (approximate_time) {
        sync_.reset(new Sync(approximate_policy(100),
                             approx_imu_subscription_,approx_odom_subscription_,
                             approx_pointcloud_subscription_, approx_navsatfix_subscription_));
        sync_->registerCallback(
                std::bind(&TransformVisualizer::approx_callback, this,
                          std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3, std::placeholders::_4));
    }

    lidar_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transform_visualizer/pointcloud", 10);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/transform_visualizer/imu", 10);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/transform_visualizer/odom/navsatfix", 10);
    odom_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/transform_visualizer/path/odom", 10);
    navsatfix_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/transform_visualizer/path/navsatfix", 10);

    base_link_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    lidar_link_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
};


void TransformVisualizer::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
    tf2::Quaternion q_orig, q_rot, q_new;
    q_orig.setX(msg->orientation.x);
    q_orig.setY(msg->orientation.y);
    q_orig.setZ(msg->orientation.z);
    q_orig.setW(msg->orientation.w);

    q_rot.setRPY(imu_roll*M_PI/180, imu_pitch*M_PI/180, imu_yaw*M_PI/180);
    q_new = q_orig * q_rot;

    // get roll pitch yaw values of the msg
    tf2::Matrix3x3 m(q_orig);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if (enable_ned2enu) {
        q_new.setRPY(pitch, roll, -yaw);
    } else {
        q_new.setRPY(roll, pitch, yaw);
    }
    q_new.normalize();

    odom.pose.pose.orientation.x = q_new.x();
    odom.pose.pose.orientation.y = q_new.y();
    odom.pose.pose.orientation.z = q_new.z();
    odom.pose.pose.orientation.w = q_new.w();

    base_link_tf.transform.rotation = odom.pose.pose.orientation;

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "base_link";
    imu_msg.header.stamp = this->get_clock()->now();
    if (enable_ned2enu) {
        imu_msg.linear_acceleration.x = msg->linear_acceleration.y;
        imu_msg.linear_acceleration.y = msg->linear_acceleration.x;
        imu_msg.linear_acceleration.z = -msg->linear_acceleration.z;
        imu_msg.angular_velocity.x = msg->angular_velocity.y;
        imu_msg.angular_velocity.y = msg->angular_velocity.x;
        imu_msg.angular_velocity.z = -msg->angular_velocity.z;
    } else {
        imu_msg.linear_acceleration = msg->linear_acceleration;
        imu_msg.angular_velocity = msg->angular_velocity;
    }
    imu_msg.orientation = odom.pose.pose.orientation;
    imu_publisher_->publish(imu_msg);
}


void TransformVisualizer::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->get_clock()->now();
    pose.pose = msg->pose.pose;
    odom_poses_.header.frame_id = "map";
    odom_poses_.header.stamp = this->get_clock()->now();
    odom_poses_.poses.push_back(pose);
    odom_path_publisher_->publish(odom_poses_);
}


void TransformVisualizer::lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    tf2::Quaternion q;
    q.setRPY(imu2lidar_roll*M_PI/180, imu2lidar_pitch*M_PI/180, imu2lidar_yaw*M_PI/180);

    RCLCPP_INFO(this->get_logger(), "POINTCLOUD IS IN");
    if (publish_tf){
        geometry_msgs::msg::TransformStamped lidar_link_tf;
        lidar_link_tf.header.stamp = this->get_clock()->now();
        lidar_link_tf.header.frame_id = "base_link";
        lidar_link_tf.child_frame_id = "lidar_link";
        lidar_link_tf.transform.translation.x = 0.0;
        lidar_link_tf.transform.translation.x = 0.0;
        lidar_link_tf.transform.translation.z = 1.5;
        lidar_link_tf.transform.rotation.x = q.x();
        lidar_link_tf.transform.rotation.y = q.y();
        lidar_link_tf.transform.rotation.z = q.z();
        lidar_link_tf.transform.rotation.w = q.w();
        lidar_link_tf_broadcaster_->sendTransform(lidar_link_tf);
    }

    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pointcloud_msg.header.frame_id = "lidar_link";
    pointcloud_msg.header.stamp = this->get_clock()->now();
    pointcloud_msg.row_step = msg->row_step;
    pointcloud_msg.point_step = msg->point_step;
    pointcloud_msg.is_dense = msg->is_dense;
    pointcloud_msg.is_bigendian = msg->is_bigendian;
    pointcloud_msg.width = msg->width;
    pointcloud_msg.height = msg->height;
    pointcloud_msg.fields = msg->fields;
    pointcloud_msg.data = msg->data;
    lidar_publisher_->publish(pointcloud_msg);
}


void TransformVisualizer::navsatfix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg) {
    // Make the first NavSatFix data the local cartesian origin
    if (!locart_init) {
        localCartesian.Reset(
                msg->latitude, msg->longitude, msg->altitude);
        locart_init = true;
    }
    // Get the position of other NavSatFix data in local cartesian cs
    else {
        double local_x, local_y, local_z;
        localCartesian.Forward(
                msg->latitude, msg->longitude, msg->altitude,
                local_x, local_y, local_z
        );
        // publish nav msg
        geometry_msgs::msg::PoseStamped approx_pose;
        approx_pose.header.frame_id = "map";
        approx_pose.header.stamp = this->get_clock()->now();
        approx_pose.pose.position.x = local_x;
        approx_pose.pose.position.y = local_y;
        approx_pose.pose.position.z = local_z;
        navsatfix_poses_.header.frame_id = "map";
        navsatfix_poses_.header.stamp = this->get_clock()->now();
        navsatfix_poses_.poses.push_back(approx_pose);
        navsatfix_path_publisher_->publish(navsatfix_poses_);

        // publish odom msg
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = local_x;
        odom.pose.pose.position.y = local_y;
        odom.pose.pose.position.z = local_z;
//        approx_odom.pose.pose.orientation.x
        odom_publisher_->publish(odom);

        // publish tf
        if (publish_tf) {
            base_link_tf.header.frame_id = "map";
            base_link_tf.header.stamp = this->get_clock()->now();
            base_link_tf.child_frame_id = "base_link";
            base_link_tf.transform.translation.x = odom.pose.pose.position.x;
            base_link_tf.transform.translation.y = odom.pose.pose.position.y;
            base_link_tf.transform.translation.z = odom.pose.pose.position.z;
            base_link_tf.transform.rotation = odom.pose.pose.orientation;
            base_link_tf_broadcaster_->sendTransform(base_link_tf);
        }
    }
}


void TransformVisualizer::approx_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
                                          const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                                          const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg,
                                          const sensor_msgs::msg::NavSatFix::ConstSharedPtr &navsatfix_msg) {
    // publish imu msg
    tf2::Quaternion q_orig, q_rot, q_new;
    q_orig.setX(imu_msg->orientation.x);
    q_orig.setY(imu_msg->orientation.y);
    q_orig.setZ(imu_msg->orientation.z);
    q_orig.setW(imu_msg->orientation.w);

    q_rot.setRPY(imu_roll*M_PI/180, imu_pitch*M_PI/180, imu_yaw*M_PI/180);
    q_new = q_orig * q_rot;

    // get roll pitch yaw values of the msg
    tf2::Matrix3x3 m(q_orig);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if (enable_ned2enu) {
        q_new.setRPY(pitch, roll, -yaw);
    } else {
        q_new.setRPY(roll, pitch, yaw);
    }
    q_new.normalize();

    sensor_msgs::msg::Imu imu_msg_;
    imu_msg_.header.frame_id = "base_link";
    imu_msg_.header.stamp = this->get_clock()->now();
    if (enable_ned2enu) {
        imu_msg_.linear_acceleration.x = imu_msg->linear_acceleration.y;
        imu_msg_.linear_acceleration.y = imu_msg->linear_acceleration.x;
        imu_msg_.linear_acceleration.z = -imu_msg->linear_acceleration.z;
        imu_msg_.angular_velocity.x = imu_msg->angular_velocity.y;
        imu_msg_.angular_velocity.y = imu_msg->angular_velocity.x;
        imu_msg_.angular_velocity.z = -imu_msg->angular_velocity.z;
    } else {
        imu_msg_.linear_acceleration = imu_msg->linear_acceleration;
        imu_msg_.angular_velocity = imu_msg->angular_velocity;
    }
    imu_msg_.orientation = odom.pose.pose.orientation;
    imu_msg_.angular_velocity_covariance = imu_msg->angular_velocity_covariance;
    imu_msg_.linear_acceleration_covariance = imu_msg->linear_acceleration_covariance;
    imu_publisher_->publish(imu_msg_);


    // init coordinate system
    if (!locart_init) {
        localCartesian.Reset(
                navsatfix_msg->latitude, navsatfix_msg->longitude, navsatfix_msg->altitude);
        locart_init = true;
    }
        // Get the position of other NavSatFix data in local cartesian cs
    else {
        double local_x, local_y, local_z;
        localCartesian.Forward(
                navsatfix_msg->latitude, navsatfix_msg->longitude, navsatfix_msg->altitude,
                local_x, local_y, local_z
        );
        // publish nav msgs
        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = local_x;
        pose.pose.position.y = local_y;
        pose.pose.position.z = local_z;
        navsatfix_poses_.header.frame_id = "map";
        navsatfix_poses_.header.stamp = this->get_clock()->now();
        navsatfix_poses_.poses.push_back(pose);
        navsatfix_path_publisher_->publish(navsatfix_poses_);

        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position = odom_msg->pose.pose.position;
        pose.pose.orientation = odom_msg->pose.pose.orientation;
        odom_poses_.header.frame_id = "map";
        odom_poses_.header.stamp = this->get_clock()->now();
        odom_poses_.poses.push_back(pose);
        odom_path_publisher_->publish(odom_poses_);

        // publish odom msg
        nav_msgs::msg::Odometry approx_odom;
        approx_odom.header.stamp = this->get_clock()->now();
        approx_odom.header.frame_id = "map";
        approx_odom.child_frame_id = "base_link";
        approx_odom.pose.pose.position.x = local_x;
        approx_odom.pose.pose.position.y = local_y;
        approx_odom.pose.pose.position.z = local_z;
        approx_odom.pose.pose.orientation.x = q_new.x();
        approx_odom.pose.pose.orientation.y = q_new.y();
        approx_odom.pose.pose.orientation.z = q_new.z();
        approx_odom.pose.pose.orientation.w = q_new.w();
        odom_publisher_->publish(approx_odom);

        // publish pointcloud msg
        sensor_msgs::msg::PointCloud2 pointcloud_msg_;
        pointcloud_msg_.header.frame_id = "lidar_link";
        pointcloud_msg_.header.stamp = this->get_clock()->now();
        pointcloud_msg_.row_step = pointcloud_msg->row_step;
        pointcloud_msg_.point_step = pointcloud_msg->point_step;
        pointcloud_msg_.is_dense = pointcloud_msg->is_dense;
        pointcloud_msg_.is_bigendian = pointcloud_msg->is_bigendian;
        pointcloud_msg_.width = pointcloud_msg->width;
        pointcloud_msg_.height = pointcloud_msg->height;
        pointcloud_msg_.fields = pointcloud_msg->fields;
        pointcloud_msg_.data = pointcloud_msg->data;
        lidar_publisher_->publish(pointcloud_msg_);

        // publish tf
        if (publish_tf) {
            base_link_tf.header.frame_id = "map";
            base_link_tf.header.stamp = this->get_clock()->now();
            base_link_tf.child_frame_id = "base_link";
            base_link_tf.transform.translation.x = odom.pose.pose.position.x;
            base_link_tf.transform.translation.y = odom.pose.pose.position.y;
            base_link_tf.transform.translation.z = odom.pose.pose.position.z;
            base_link_tf.transform.rotation = odom.pose.pose.orientation;
            base_link_tf_broadcaster_->sendTransform(base_link_tf);

            tf2::Quaternion q;
            q.setRPY(imu2lidar_roll*M_PI/180, imu2lidar_pitch*M_PI/180, imu2lidar_yaw*M_PI/180);
            geometry_msgs::msg::TransformStamped lidar_link_tf;
            lidar_link_tf.header.stamp = this->get_clock()->now();
            lidar_link_tf.header.frame_id = "base_link";
            lidar_link_tf.child_frame_id = "lidar_link";
            lidar_link_tf.transform.translation.x = 0.0;
            lidar_link_tf.transform.translation.x = 0.0;
            lidar_link_tf.transform.translation.z = 1.5;
            lidar_link_tf.transform.rotation.x = q.x();
            lidar_link_tf.transform.rotation.y = q.y();
            lidar_link_tf.transform.rotation.z = q.z();
            lidar_link_tf.transform.rotation.w = q.w();
            lidar_link_tf_broadcaster_->sendTransform(lidar_link_tf);
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformVisualizer>());
    rclcpp::shutdown();
    return 0;
}