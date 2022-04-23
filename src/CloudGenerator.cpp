
#include "../include/CloudGenerator.hpp"
#include <pcl/conversions.h>


void pclXYZCloud2Gen::toCloud2(DomainIterable &dom, pcl::PCLPointCloud2 &cloud2)
{
    auto cloud = new pcl::PointCloud<pcl::PointXYZ>();
    builder->build(dom, *cloud);
    pcl::toPCLPointCloud2(*cloud, cloud2);
    delete cloud;
}


void pclXYZRGBCloud2Gen::toCloud2(DomainIterable &dom, pcl::PCLPointCloud2 &cloud2)
{
    auto cloud = new pcl::PointCloud<pcl::PointXYZRGB>();
    builder->build(dom, *cloud);
    pcl::toPCLPointCloud2(*cloud, cloud2);
    delete cloud;
}


void pclXYZICloud2Gen::toCloud2(DomainIterable &dom, pcl::PCLPointCloud2 &cloud2)
{
    auto cloud = new pcl::PointCloud<pcl::PointXYZI>();
    builder->build(dom, *cloud);
    pcl::toPCLPointCloud2(*cloud, cloud2);
    delete cloud;
}


template <typename T>
void pclPtsCloudGen::toCloud(DomainIterable &dom, pcl::PointCloud<T>& cloud)
{
    cloud.clear();
    builder->build(dom, cloud);
}