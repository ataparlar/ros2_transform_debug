//
// Created by ataparlar on 08.09.2023.
//

#ifndef BUILD_POINTCLOUD_TRANSLATOR_HPP
#define BUILD_POINTCLOUD_TRANSLATOR_HPP

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
#include "GeographicLib/UTMUPS.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



class PointCloudTranslator : public rclcpp::Node {
public:
    PointCloudTranslator();
private:

};



#endif //BUILD_POINTCLOUD_TRANSLATOR_HPP
