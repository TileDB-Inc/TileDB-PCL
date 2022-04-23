
#pragma once

#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/file_io.h>
#include "DimFields.hpp"
#include "BufferBuilder.hpp"
#include "CloudGeneratorFactory.hpp"
#include "DomainIterable.hpp"


class TileDBPCLReader: public pcl::FileReader
{
public:
    TileDBPCLReader(){}

    TileDBPCLReader(DimFields _fields);

    TileDBPCLReader(DimFields _fields, bool includeAttrs);

    TileDBPCLReader(DimFields _fields, pclCloudGenFactory* cgf, bool includeAttrs = false);

    int readHeader(const std::string &file_name, pcl::PCLPointCloud2 &cloud, Eigen::Vector4f &origin,
                   Eigen::Quaternionf &orientation, int &file_version, int &data_type, unsigned int &data_idx,
                   const int offset) override;

    int read(const std::string &file_name, pcl::PCLPointCloud2 &cloud, Eigen::Vector4f &origin,
             Eigen::Quaternionf &orientation, int &file_version, const int offset) override;

    int read(const std::string& file_name, pcl::PCLPointCloud2& cloud);

    template <typename PointT> inline
    int read(const std::string &file_name, typename pcl::PointCloud<PointT>::Ptr cloud, const int offset = 0)
    {
        auto bufferBuilder = new TileDBArrayBufferBuilder;
        auto dim_names = fields.to_vector();
        DomainIterable iDom(file_name, bufferBuilder, dim_names);
        auto cloudBuilder = new CloudBuilder();
        cloudBuilder->build(iDom, *cloud);
        delete cloudBuilder;
        delete bufferBuilder;
        return 0;
    }

    void set_dim_fields(DimFields _fields) { fields = _fields; }

    void set_include_attrs(bool include) { include_attrs = include; }

private:
    DimFields fields{DimFields::XYZ};
    bool include_attrs{false};
    pclCloudGenFactory* cloud_gen_factory{new pclPtsCloud2GenFactory()};
};


