
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "DomainIterable.hpp"


class DomainIterable;


class CloudBuilder
{
public:
    void build(DomainIterable& dom, pcl::PointCloud<pcl::PointXYZ>& cloud);
    void build(DomainIterable& dom, pcl::PointCloud<pcl::PointXYZRGB>& cloud);
    void build(DomainIterable& dom, pcl::PointCloud<pcl::PointXYZI>& cloud);
};