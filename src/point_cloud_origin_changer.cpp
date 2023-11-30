//
// Created by ataparlar on 30.11.2023.
//

#include "data_provider_pkg/point_cloud_origin_changer.hpp"
#include "GeographicLib/UTMUPS.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

PointCloudOriginChanger::PointCloudOriginChanger() :
        Node("point_cloud_origin_changer") {

    this->declare_parameter("pcd_map_full_path", "");
    this->declare_parameter("new_pcd_full_path", "");
    this->declare_parameter("old_origin_lat", 41.019141875);
    this->declare_parameter("old_origin_lon", 28.88822681013889);
    this->declare_parameter("old_origin_height", 116.250886);
    this->declare_parameter("new_origin_lat", 41.018738);
    this->declare_parameter("new_origin_lon", 139.349550);
    this->declare_parameter("new_origin_height", 28.88742);

    pcd_map_full_path_ = this->get_parameter("pcd_map_full_path").as_string();
    new_pcd_full_path_ = this->get_parameter("new_pcd_full_path").as_string();
    old_origin_lat_ = this->get_parameter("old_origin_lat").as_double();
    old_origin_lon_ = this->get_parameter("old_origin_lon").as_double();
    old_origin_height_ = this->get_parameter("old_origin_height").as_double();
    new_origin_lat_ = this->get_parameter("new_origin_lat").as_double();
    new_origin_lon_ = this->get_parameter("new_origin_lon").as_double();
    new_origin_height_ = this->get_parameter("new_origin_height").as_double();


    int old_zone;
    bool old_northp;
    double old_origin_x, old_origin_y, old_gamma, old_k;
    GeographicLib::UTMUPS::Forward(
            old_origin_lat_, old_origin_lon_, old_zone, old_northp,
            old_origin_x, old_origin_y, old_gamma, old_k);


    int new_zone;
    bool new_northp;
    double new_origin_x, new_origin_y, new_gamma, new_k;
    GeographicLib::UTMUPS::Forward(
            new_origin_lat_, new_origin_lon_, new_zone, new_northp,
            new_origin_x, new_origin_y, new_gamma, new_k);


    double delta_x = new_origin_x - old_origin_x;
    double delta_y = new_origin_y - old_origin_y;
    double delta_z = new_origin_height_ - old_origin_height_;



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  // cloud to be read
    pcl::PointCloud<pcl::PointXYZ> new_cloud;  // cloud to be written

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_map_full_path_, *cloud) == -1) //* load the file
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

    new_cloud.resize(cloud->size());
    size_t point_counter = 0;
    for (const auto& point: *cloud){
        // edit coordinates of all the points
        new_cloud[point_counter].x = point.x - delta_x;
        new_cloud[point_counter].y = point.y - delta_y;
        new_cloud[point_counter].z = point.z - delta_z;
        point_counter++;
    }

    pcl::io::savePCDFileASCII(new_pcd_full_path_, new_cloud);

    RCLCPP_INFO(this->get_logger(), "New point cloud saved to %s.", new_pcd_full_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "New point cloud origin:\n"
                                    "Latitude:\t %f\n"
                                    "Longitude:\t %f\n"
                                    "Height:\t %f\n", new_origin_lat_, new_origin_lon_, new_origin_height_);
    RCLCPP_INFO(this->get_logger(), "New point cloud has %zu points.", point_counter);

    rclcpp::shutdown();
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudOriginChanger>());
    rclcpp::shutdown();
    return 0;
}