#include <iostream>
#include <thread>
#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include "../include/TileDBPCLReader.hpp"



using namespace std;

int main() {
    std::string path = "../../../Downloads/autzen_tiledb";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    TileDBPCLReader r;
//    r.set_dim_fields(DimFields::XYZRGB);
    r.read<pcl::PointXYZ>(path, cloud);
    cout << cloud->width << endl;
    cout << cloud->height << endl;
    cout << cloud->size() << endl;
    cout << cloud->sensor_orientation_ << endl;
//    pcl::io::loadPCDFile(path, *cloud);
    pcl::visualization::PCLVisualizer viewer("vis");
    viewer.addPointCloud(cloud);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::nanoseconds(100000));
    }
//    std::string out_path = "../../../Downloads/autzen_crop.pcd";



    return 0;
}