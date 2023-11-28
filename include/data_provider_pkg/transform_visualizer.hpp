//
// Created by ataparlar on 06.09.2023.
//

#ifndef BUILD_TRANSFORM_VISUALIZER_HPP
#define BUILD_TRANSFORM_VISUALIZER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "message_filters/subscriber.h"
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "GeographicLib/LocalCartesian.hpp"
#include "applanix_msgs/msg/navigation_solution_gsof49.hpp"
#include "autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp"


class TransformVisualizer : public rclcpp::Node
{
public:
    TransformVisualizer();
private:
    // Parameters
    double imu_roll;
    double imu_pitch;
    double imu_yaw;
    double imu2lidar_roll;
    double imu2lidar_pitch;
    double imu2lidar_yaw;
    std::string imu_topic;
    std::string odom_topic;
    std::string lidar_topic;
    std::string navsatfix_topic;
    std::string autoware_orientation_topic_;
//    std::string ins_solution_topic;
    bool publish_tf;
    bool enable_ned2enu;


    struct PositionPacket
    {
        uint8_t udp_header[42];
        uint8_t reserved_01[187];
        uint8_t temp_top_board;
        uint8_t temp_bot_board;
        uint8_t reserved_02[9];
        uint32_t timestamp_microseconds_since_hour;
        uint8_t status_pps;
        uint8_t status_thermal;
        uint8_t temp_when_last_shut_from_overheat;
        uint8_t temp_when_booted;
        char nmea_sentence[128];
        uint8_t reserved_03[178];
    } __attribute__((packed));

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_subscription_;
    rclcpp::Subscription<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr
        autoware_orientation_subscription_;
//    rclcpp::Subscription<applanix_msgs::msg::NavigationSolutionGsof49>::SharedPtr ins_solution_subscription;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr navsatfix_path_publisher_;

    // Callbacks
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
    void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
    void navsatfix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr & msg);
    void autoware_orientation_callback(const autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr & msg);
    void ins_solution_callback(const applanix_msgs::msg::NavigationSolutionGsof49::ConstSharedPtr & msg);

    // Transform Frames
    geometry_msgs::msg::TransformStamped base_link_tf;
    std::unique_ptr<tf2_ros::TransformBroadcaster> base_link_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> lidar_link_tf_broadcaster_;


    bool locart_init = false;
    GeographicLib::LocalCartesian localCartesian;

    nav_msgs::msg::Path odom_poses_;
    nav_msgs::msg::Path navsatfix_poses_;

    geometry_msgs::msg::PoseStamped pose;
    nav_msgs::msg::Odometry odom;

    bool odom_init;
    double odom_x;
    double odom_y;
    double odom_z;


};


#endif //BUILD_TRANSFORM_VISUALIZER_HPP
