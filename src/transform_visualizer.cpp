////
//// Created by ataparlar on 06.09.2023.
////
//
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2/LinearMath/Matrix3x3.h>
//#include "data_provider_pkg/transform_visualizer.hpp"
//#include "autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp"
//
//TransformVisualizer::TransformVisualizer() :
//        Node("transform_visualizer") {
//
//    this->declare_parameter("imu_r_x", 0.0);
//    this->declare_parameter("imu_r_y", 0.0);
//    this->declare_parameter("imu_r_z", 0.0);
//    this->declare_parameter("imu2lidar_x", 0.0);
//    this->declare_parameter("imu2lidar_y", 0.0);
//    this->declare_parameter("imu2lidar_z", 0.0);
//    this->declare_parameter("imu_topic", "");
//    this->declare_parameter("odom_topic", "");
//    this->declare_parameter("lidar_topic", "");
//    this->declare_parameter("autoware_orientation_topic", "");
////    this->declare_parameter("ins_solution_topic", "");
//    this->declare_parameter("navsatfix_topic", "");
//    this->declare_parameter("publish_tf", true);
//    this->declare_parameter("enable_ned2enu", false);
//
//    imu_roll = this->get_parameter("imu_r_x").as_double();
//    imu_pitch = this->get_parameter("imu_r_y").as_double();
//    imu_yaw = this->get_parameter("imu_r_z").as_double();
//    imu2lidar_roll = this->get_parameter("imu2lidar_x").as_double();
//    imu2lidar_pitch = this->get_parameter("imu2lidar_y").as_double();
//    imu2lidar_yaw = this->get_parameter("imu2lidar_z").as_double();
//    imu_topic = this->get_parameter("imu_topic").as_string();
//    odom_topic = this->get_parameter("odom_topic").as_string();
//    lidar_topic = this->get_parameter("lidar_topic").as_string();
//    autoware_orientation_topic_ = this->get_parameter("autoware_orientation_topic").as_string();
//    navsatfix_topic = this->get_parameter("navsatfix_topic").as_string();
//    publish_tf = this->get_parameter("publish_tf").as_bool();
//    enable_ned2enu = this->get_parameter("enable_ned2enu").as_bool();
//
//
//    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
//            imu_topic, 10, std::bind(&TransformVisualizer::imu_callback,
//                                       this, std::placeholders::_1));
//    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
//            odom_topic, 10, std::bind(&TransformVisualizer::odom_callback,
//                                       this, std::placeholders::_1));
//    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//            lidar_topic, rclcpp::SensorDataQoS(), std::bind(&TransformVisualizer::lidar_callback,
//                                           this, std::placeholders::_1));
////    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
////            lidar_topic, 10, std::bind(&TransformVisualizer::lidar_callback,
////                                           this, std::placeholders::_1));
//    navsatfix_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
//            navsatfix_topic, 10, std::bind(&TransformVisualizer::navsatfix_callback,
//                                           this, std::placeholders::_1));
//    autoware_orientation_subscription_ = this->create_subscription<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
//            autoware_orientation_topic_, 10, std::bind(&TransformVisualizer::autoware_orientation_callback,
//                                           this, std::placeholders::_1));
////    ins_solution_subscription = this->create_subscription<applanix_msgs::msg::NavigationSolutionGsof49>(
////            ins_solution_topic, 10, std::bind(&TransformVisualizer::ins_solution_callback,
////                                           this, std::placeholders::_1));
//
//
//    lidar_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transform_visualizer/pointcloud", 10);
//    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/transform_visualizer/imu", 10);
//    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/transform_visualizer/odom", 10);
//    odom_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/transform_visualizer/path/odom", 10);
//    navsatfix_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/transform_visualizer/path/navsatfix", 10);
//
//
//    base_link_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
//    lidar_link_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
//
//    odom_init = false;
//};
//
//
//void TransformVisualizer::autoware_orientation_callback(
//        const autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr &msg) {
//
//    tf2::Quaternion q_orig, q_rot, q_new;
//    q_orig.setX(msg->orientation.orientation.x);
//    q_orig.setY(msg->orientation.orientation.y);
//    q_orig.setZ(msg->orientation.orientation.z);
//    q_orig.setW(msg->orientation.orientation.w);
//
//    q_rot.setRPY(imu_roll*M_PI/180, imu_pitch*M_PI/180, imu_yaw*M_PI/180);
//    q_new = q_orig * q_rot;
//
//    // get roll pitch yaw values of the msg
//    tf2::Matrix3x3 m(q_orig);
//    double roll, pitch, yaw;
//    m.getRPY(roll, pitch, yaw);
//
//    if (enable_ned2enu) {
//        q_new.setRPY(pitch, roll, -yaw);
//    } else {
//        q_new.setRPY(roll, pitch, yaw);
//    }
//    q_new.normalize();
//
//    base_link_tf.transform.rotation.x = q_new.x();
//    base_link_tf.transform.rotation.y = q_new.y();
//    base_link_tf.transform.rotation.z = q_new.z();
//    base_link_tf.transform.rotation.w = q_new.w();
//
//}
//
//
//void TransformVisualizer::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
////    tf2::Quaternion q_orig, q_rot, q_new;
////    q_orig.setX(msg->orientation.x);
////    q_orig.setY(msg->orientation.y);
////    q_orig.setZ(msg->orientation.z);
////    q_orig.setW(msg->orientation.w);
////
////    q_rot.setRPY(imu_roll*M_PI/180, imu_pitch*M_PI/180, imu_yaw*M_PI/180);
////    q_new = q_orig * q_rot;
////
////    // get roll pitch yaw values of the msg
////    tf2::Matrix3x3 m(q_orig);
////    double roll, pitch, yaw;
////    m.getRPY(roll, pitch, yaw);
////
////    if (enable_ned2enu) {
////        q_new.setRPY(pitch*5, roll, -yaw);
////    } else {
////        q_new.setRPY(roll, pitch*5, yaw);
////    }
////    q_new.normalize();
////
////    odom.pose.pose.orientation.x = q_new.x();
////    odom.pose.pose.orientation.y = q_new.y();
////    odom.pose.pose.orientation.z = q_new.z();
////    odom.pose.pose.orientation.w = q_new.w();
////
////    base_link_tf.transform.rotation.x = q_new.x();
////    base_link_tf.transform.rotation.y = q_new.y();
////    base_link_tf.transform.rotation.z = q_new.z();
////    base_link_tf.transform.rotation.w = q_new.w();
////
////    sensor_msgs::msg::Imu imu_msg;
////    imu_msg.header.frame_id = "base_link";
////    imu_msg.header.stamp = this->get_clock()->now();
////    if (enable_ned2enu) {
////        imu_msg.linear_acceleration.x = msg->linear_acceleration.y;
////        imu_msg.linear_acceleration.y = msg->linear_acceleration.x;
////        imu_msg.linear_acceleration.z = -msg->linear_acceleration.z;
////        imu_msg.angular_velocity.x = msg->angular_velocity.y;
////        imu_msg.angular_velocity.y = msg->angular_velocity.x;
////        imu_msg.angular_velocity.z = -msg->angular_velocity.z;
////    } else {
////        imu_msg.linear_acceleration = msg->linear_acceleration;
////        imu_msg.angular_velocity = msg->angular_velocity;
////    }
////    imu_msg.orientation = odom.pose.pose.orientation;
////    imu_publisher_->publish(imu_msg);
//}
//
//
//void TransformVisualizer::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
//    if (!odom_init) {
//        odom_x = msg->pose.pose.position.x;
//        odom_y = msg->pose.pose.position.y;
//        odom_z = msg->pose.pose.position.z;
//        odom_init = true;
//    } else {
//        geometry_msgs::msg::PoseStamped pose;
//        pose.header.frame_id = "map";
//        pose.header.stamp = this->get_clock()->now();
//        pose.pose = msg->pose.pose;
//        odom.header.stamp = this->get_clock()->now();
//        odom.header.frame_id = "map";
//        odom.pose.pose.position.x = msg->pose.pose.position.x - odom_x;
//        odom.pose.pose.position.y = msg->pose.pose.position.y - odom_y;
//        odom.pose.pose.position.z = msg->pose.pose.position.z - odom_z;
//        odom_publisher_->publish(odom);
//        odom_poses_.header.frame_id = "map";
//        odom_poses_.header.stamp = this->get_clock()->now();
//        odom_poses_.poses.push_back(pose);
//        odom_path_publisher_->publish(odom_poses_);
//    }
//}
//
//
//void TransformVisualizer::lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
//    tf2::Quaternion q;
//    q.setRPY(imu2lidar_roll*M_PI/180, imu2lidar_pitch*M_PI/180, imu2lidar_yaw*M_PI/180);
//
//    tf2::Matrix3x3 matrix3X3;
//    matrix3X3.setRPY(imu2lidar_roll*M_PI/180, imu2lidar_pitch*M_PI/180, imu2lidar_yaw*M_PI/180);
//    tf2::Matrix3x3 inverse_matrix3X3;
//    inverse_matrix3X3 = matrix3X3.inverse();
//    double roll, pitch, yaw;
//    inverse_matrix3X3.getRPY(roll, pitch, yaw);
////    std::cout << "roll: " << roll << " ------- pitch: " << pitch << " -------- yaw: " << yaw << "\n\n" << std::endl;
//    RCLCPP_INFO(this->get_logger(), "roll: %f -------- pitch: %f --------- yaw: %f \n",
//                roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
//
//
//    if (publish_tf){
//        geometry_msgs::msg::TransformStamped lidar_link_tf;
//        lidar_link_tf.header.stamp = this->get_clock()->now();
//        lidar_link_tf.header.frame_id = "base_link";
//        lidar_link_tf.child_frame_id = "lidar_link";
//        lidar_link_tf.transform.translation.x = 0.0;
//        lidar_link_tf.transform.translation.x = 0.0;
//        lidar_link_tf.transform.translation.z = 1.5;
//        lidar_link_tf.transform.rotation.x = q.x();
//        lidar_link_tf.transform.rotation.y = q.y();
//        lidar_link_tf.transform.rotation.z = q.z();
//        lidar_link_tf.transform.rotation.w = q.w();
//        lidar_link_tf_broadcaster_->sendTransform(lidar_link_tf);
//    }
//
//    sensor_msgs::msg::PointCloud2 pointcloud_msg;
//    pointcloud_msg.header.frame_id = "lidar_link";
////    pointcloud_msg.header.frame_id = "hesai_base_link";
//    pointcloud_msg.header.stamp = this->get_clock()->now();
//    pointcloud_msg.row_step = msg->row_step;
//    pointcloud_msg.point_step = msg->point_step;
//    pointcloud_msg.is_dense = msg->is_dense;
//    pointcloud_msg.is_bigendian = msg->is_bigendian;
//    pointcloud_msg.width = msg->width;
//    pointcloud_msg.height = msg->height;
//    pointcloud_msg.fields = msg->fields;
//    pointcloud_msg.data = msg->data;
//    lidar_publisher_->publish(pointcloud_msg);
//}
//
//
//void TransformVisualizer::navsatfix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg) {
//    // Make the first NavSatFix data the local cartesian origin
//    if (!locart_init) {
//        localCartesian.Reset(
//                msg->latitude, msg->longitude, msg->altitude);
//        locart_init = true;
//    }
//    // Get the position of other NavSatFix data in local cartesian cs
//    else {
//        double local_x, local_y, local_z;
//        localCartesian.Forward(
//                msg->latitude, msg->longitude, msg->altitude,
//                local_x, local_y, local_z
//        );
//        // publish nav msg
//        pose.header.frame_id = "map";
//        pose.header.stamp = this->get_clock()->now();
//        pose.pose.position.x = local_x;
//        pose.pose.position.y = local_y;
//        pose.pose.position.z = local_z;
//        navsatfix_poses_.header.frame_id = "map";
//        navsatfix_poses_.header.stamp = this->get_clock()->now();
//        navsatfix_poses_.poses.push_back(pose);
//        navsatfix_path_publisher_->publish(navsatfix_poses_);
//
//        // publish odom msg
////        odom.header.stamp = this->get_clock()->now();
////        odom.header.stamp = msg->header.stamp;
////        odom.header.frame_id = "map";
////        odom.child_frame_id = "base_link";
////        odom.pose.pose.position.x = local_x;
////        odom.pose.pose.position.y = local_y;
////        odom.pose.pose.position.z = local_z;
////        odom_publisher_->publish(odom);
//
//        // publish tf
//        if (publish_tf) {
//            base_link_tf.header.frame_id = "map";
//            base_link_tf.header.stamp = this->get_clock()->now();
//            base_link_tf.child_frame_id = "base_link";
//            base_link_tf.transform.translation.x = local_x;
//            base_link_tf.transform.translation.y = local_y;
//            base_link_tf.transform.translation.z = local_z;
////            base_link_tf.transform.rotation = odom.pose.pose.orientation;
//            base_link_tf_broadcaster_->sendTransform(base_link_tf);
//        }
//    }
//}
//
//
//int main(int argc, char * argv[])
//{
//    rclcpp::init(argc, argv);
//    rclcpp::spin(std::make_shared<TransformVisualizer>());
//    rclcpp::shutdown();
//    return 0;
//}