//
// Created by ataparlar on 08.09.2023.
//

#include <data_provider_pkg/pointcloud_translator.hpp>


PointCloudTranslator::PointCloudTranslator() :
        Node("transform_visualizer") {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (
            "/home/ataparlar/Downloads/service_LOAM_last/GlobalMap.pcd", *cloud);
//    pcl::PCLPointCloud2 cloud_blob;
//    pcl::io::loadPCDFile ("test_pcd.pcd", cloud_blob);
//    pcl::fromPCLPointCloud2 (cloud_blob, *cloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>

    double lat = 41.024007930319044;
    double lon = 28.896191868814206;
    int zone;
    bool northp;
    double x, y;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);

    pcl::PointCloud<pcl::PointXYZ> new_cloud;
    new_cloud.resize(cloud->size());
    int i = 0;
    for (const auto& point: *cloud){
        new_cloud[i].x = point.x + x;
        new_cloud[i].y = point.y + y;
        new_cloud[i].z = point.z + 111.195; //37.43;
        i++;
    };
    pcl::io::savePCDFileASCII (
            "/home/ataparlar/Downloads/service_LOAM_last/GlobalCoordinatesMap.pcd", new_cloud);

    std::cout << "SAVED SAVED SAVED" << std::endl;
    std::cout << "SAVED SAVED SAVED" << std::endl;
    std::cout << "SAVED SAVED SAVED" << std::endl;
    std::cout << "SAVED SAVED SAVED" << std::endl;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTranslator>());
    rclcpp::shutdown();
    return 0;
}