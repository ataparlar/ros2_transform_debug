//
// Created by ataparlar on 30.11.2023.
//

#ifndef BUILD_POINT_CLOUD_ORIGIN_CHANGER_HPP
#define BUILD_POINT_CLOUD_ORIGIN_CHANGER_HPP


#include <rclcpp/rclcpp.hpp>

class PointCloudOriginChanger : public rclcpp::Node {
public:
    PointCloudOriginChanger();

    // Parameters
    std::string pcd_map_full_path_;
    std::string new_pcd_full_path_;
    double old_origin_lat_;
    double old_origin_lon_;
    double old_origin_height_;
    double new_origin_lat_;
    double new_origin_lon_;
    double new_origin_height_;

private:

};



#endif //BUILD_POINT_CLOUD_ORIGIN_CHANGER_HPP