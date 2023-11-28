//
// Created by ataparlar on 28.11.2023.
//

#include "data_provider_pkg/lanelet2_mover.hpp"
#include <GeographicLib/UTMUPS.hpp>

#include <fstream>

Lanelet2Mover::Lanelet2Mover() :
        Node("lanelet2_mover") {

    this->declare_parameter("origin_lat", 35.324064);
    this->declare_parameter("origin_lon", 139.349550);
    this->declare_parameter("lanelet2_file_full_path", "");
    this->declare_parameter("output_lanelet2_file_full_path", "");

    origin_lat_ = this->get_parameter("origin_lat").as_double();
    origin_lon_ = this->get_parameter("origin_lon").as_double();
    lanelet2_file_full_path_ = this->get_parameter("lanelet2_file_full_path").as_string();
    lanelet2_file_full_path_output_ = this->get_parameter("output_lanelet2_file_full_path").as_string();


    int zone;
    bool northp;
    double origin_x, origin_y, gamma, k;
    GeographicLib::UTMUPS::Forward(
            origin_lat_, origin_lon_, zone, northp, origin_x, origin_y, gamma, k);


    std::ofstream file_writer;
    std::string txt_name = lanelet2_file_full_path_output_;
    file_writer.open( txt_name);
    file_writer << "<?xml version='1.0' encoding='UTF-8'?>\n<osm version=\"0.6\" upload=\"true\" generator=\"commonroad-scenario-designer\">" << std::endl;



    std::fstream lanelet2_file;
    lanelet2_file.open(lanelet2_file_full_path_, std::ios::in | std::ios::out);

    if (lanelet2_file.is_open()) {
        std::string line;
        // Read data from the file object and put it into a string.
        while (getline(lanelet2_file, line)) {
            bool line_written = false;
            // Print the data of the string.
//            std::cout << line << std::endl;
            size_t lat_string_start = line.find("lat=");
            size_t lon_string_start = line.find("lon=");

            if (lat_string_start != std::string::npos || lon_string_start != std::string::npos) {
                std::string lat_string = line.substr(lat_string_start, line.length()-lat_string_start);

                size_t find_quote = lat_string.find("\"");
                std::string lat_delete = lat_string.substr(0, find_quote+1);
                lat_string.erase(0, lat_delete.length());
//                std::cout << lat_string << std::endl;

                find_quote = lat_string.find("\"");
                std::string old_lat_str = lat_string.substr(0, find_quote);
                double old_lat = std::stod(old_lat_str);



                std::string lon_string = line.substr(lon_string_start, line.length()-lon_string_start);
                find_quote = lon_string.find("\"");
                std::string lon_delete = lat_string.substr(0, find_quote+1);
                lon_string.erase(0, lon_delete.length());

                find_quote = lon_string.find("\"");
                std::string old_lon_str = lat_string.substr(0, find_quote);
                double old_lon = std::stod(old_lon_str);




                int zone2;
                bool northp2;
                double x, y, gamma2, k2;
                GeographicLib::UTMUPS::Forward(
                        old_lat, old_lon, zone2, northp2, x, y, gamma2, k2);


                double new_x = origin_x + x;
                double new_y = origin_y + y;

                double new_lat, new_lon;
                GeographicLib::UTMUPS::Reverse(
                        zone, northp, new_x, new_y, new_lat, new_lon, gamma, k);

                std::cout << "new_lat: " << new_lat << std::endl;
                std::cout << "new_lon: " << new_lon << "\n" << std::endl;


                file_writer << line.substr(0, line.find("lat=")+5) << std::setprecision(12) << std::to_string(new_lat) <<
                    "\" lon=\"" << std::to_string(new_lon) << "\"/>" << std::endl;
                line_written = true;

            }

            if (!line_written) {
                file_writer << line << std::endl;
            }

        }
        lanelet2_file.close();
    }
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Lanelet2Mover>());
    rclcpp::shutdown();
    return 0;
}