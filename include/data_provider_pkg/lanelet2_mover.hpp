//
// Created by ataparlar on 28.11.2023.
//

#ifndef BUILD_LANELET2_MOVER_HPP
#define BUILD_LANELET2_MOVER_HPP

#include <rclcpp/rclcpp.hpp>

class Lanelet2Mover : public rclcpp::Node {
public:
    Lanelet2Mover();


    // Parameters
    double origin_lat_;
    double origin_lon_;
    std::string lanelet2_file_full_path_;
    std::string lanelet2_file_full_path_output_;


private:

};




#endif //BUILD_LANELET2_MOVER_HPP
