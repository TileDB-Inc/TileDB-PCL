
#pragma once

#include <memory>
#include <pcl/PCLPointCloud2.h>
#include "DimFields.hpp"
#include "CloudBuilder.hpp"


class DomainIterable;


class pclCloudGen
{
protected:
    std::unique_ptr<CloudBuilder> builder;
public:
    pclCloudGen()
    {
        builder = std::make_unique<CloudBuilder>();
    }
    virtual void toCloud2(DomainIterable& dom, pcl::PCLPointCloud2& cloud2) = 0;
    virtual ~pclCloudGen() = default;
};


class pclXYZCloud2Gen : public pclCloudGen
{
public:
    void toCloud2(DomainIterable& dom, pcl::PCLPointCloud2& cloud2) override;
};


class pclXYZRGBCloud2Gen : public pclCloudGen
{
public:
    void toCloud2(DomainIterable& dom, pcl::PCLPointCloud2& cloud2) override;
};


class pclXYZICloud2Gen : public pclCloudGen
{
public:
    void toCloud2(DomainIterable& dom, pcl::PCLPointCloud2& cloud2) override;
};


class pclPtsCloudGen
{
protected:
    std::unique_ptr<CloudBuilder> builder;
public:
    pclPtsCloudGen()
    {
        builder = std::make_unique<CloudBuilder>();
    }
    template <typename T>
    void toCloud(DomainIterable& dom, pcl::PointCloud<T>& cloud);
};

