
#include "../include/CloudBuilder.hpp"


void CloudBuilder::build(DomainIterable& dom, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    for (auto it = dom.begin(); it != dom.end(); ++it)
        cloud.push_back({
                                static_cast<float>(it.dim(0)),
                                static_cast<float>(it.dim(1)),
                                static_cast<float>(it.dim(2))
                        });
}

void CloudBuilder::build(DomainIterable& dom, pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    for (auto it = dom.begin(); it != dom.end(); ++it)
        cloud.push_back({
                                static_cast<float>(it.dim(0)),
                                static_cast<float>(it.dim(1)),
                                static_cast<float>(it.dim(2)),
                                static_cast<uint8_t>(it.dim(3)),
                                static_cast<uint8_t>(it.dim(4)),
                                static_cast<uint8_t>(it.dim(5))
                        });
}

void CloudBuilder::build(DomainIterable& dom, pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    for (auto it = dom.begin(); it != dom.end(); ++it)
        cloud.push_back({
                                static_cast<float>(it.dim(0)),
                                static_cast<float>(it.dim(1)),
                                static_cast<float>(it.dim(2)),
                                static_cast<float>(it.dim(3))
                        });
}