#include <iostream>
#include <cstdio>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <tiledb/tiledb>
#include "../include/main.hpp"
#include "../include/tiledb_write.hpp"


RangeSetter& RangeSetter::addRange(const tiledb::Dimension &dim)
{
    switch (dim.type())
    {
        case TILEDB_UINT8:
            q->add_range(dim.name(), dim.domain<uint8_t>().first, dim.domain<uint8_t>().second);
            break;
        case TILEDB_UINT16:
            q->add_range(dim.name(), dim.domain<uint16_t>().first, dim.domain<uint16_t>().second);
            break;
        case TILEDB_UINT32:
            q->add_range(dim.name(), dim.domain<uint32_t>().first, dim.domain<uint32_t>().second);
            break;
        case TILEDB_UINT64:
            q->add_range(dim.name(), dim.domain<uint64_t>().first, dim.domain<uint64_t>().second);
            break;
        case TILEDB_INT8:
            q->add_range(dim.name(), dim.domain<int8_t>().first, dim.domain<int8_t>().second);
            break;
        case TILEDB_INT16:
            q->add_range(dim.name(), dim.domain<int16_t>().first, dim.domain<int16_t>().second);
            break;
        case TILEDB_INT32:
            q->add_range(dim.name(), dim.domain<int32_t>().first, dim.domain<int32_t>().second);
            break;
        case TILEDB_INT64:
            q->add_range(dim.name(), dim.domain<int64_t>().first, dim.domain<int64_t>().second);
            break;
        case TILEDB_FLOAT32:
            q->add_range(dim.name(), dim.domain<float>().first, dim.domain<float>().second);
            break;
        case TILEDB_FLOAT64:
        default:
            q->add_range(dim.name(), dim.domain<double>().first, dim.domain<double>().second);
    }
    return *this;
}


RangeSetter& RangeSetter::addBuffer(const tiledb::Dimension& dim, std::vector<double>& buff)
{
    q->set_data_buffer(dim.name(), buff);
    return *this;
}


void TileDBArrayBufferBuilder::build_ibuff(const std::string &filename, const std::vector<std::string>& dim_names)
{
    tiledb::Context ctx;
    tiledb::Array array(ctx, filename, TILEDB_READ);
    tiledb::Query* q = new tiledb::Query(ctx, array);
    for (const auto& dim: array.schema().domain().dimensions())
    {
        if (dim_names.empty() || DimNameIdxFinder()(dim_names, dim.name()) != -1)
            if (dim.type() == TILEDB_FLOAT64) {   // TO FIX: generalize to take all dim types
                std::vector<double> dim_buff(array.schema().capacity());
                i_buff.push_back(dim_buff);
                RangeSetter(q).addRange(dim).addBuffer(dim, i_buff.at(i_buff.size() - 1));
            }
    }
    q->submit();
}


void TileDBArrayBufferBuilder::build(const std::string &filename, std::vector<double> &buff, const std::vector<std::string>& dim_names)
{
    build_ibuff(filename, dim_names);
    std::vector<double> temp;
    size_t last_nonempty = 0;
    double val;
    size_t len = i_buff.at(0).size();
    size_t dims = i_buff.size();
    for (size_t i = 0; i < len; i++) {
        for (size_t j = 0; j < dims; j++) {
            val = i_buff.at(j).at(i);
            if (val != m_null_val) last_nonempty = (dims * i) + j;
            temp.push_back(val);
        }
    }
    std::vector<double> data(&temp[0], &temp[last_nonempty + 1]);
    buff.swap(data);
}


double DomainIterator::dim(size_t idx) { return *(m_ptr + idx); }


double DomainIterator::dim(const std::string &name)
{
    if (m_dim_names.empty()) return double(-1);
    size_t idx = DimNameIdxFinder()(m_dim_names, name);
    if (idx < 0) return double(-1);
    return dim(idx);
}



double& DomainIterator::operator*() { return *m_ptr; }
double* DomainIterator::operator->() { return m_ptr; }


DomainIterator& DomainIterator::operator++()
{
    m_ptr += m_pt_size;
    return *this;
}


DomainIterator DomainIterator::operator++(int)
{
    DomainIterator temp = *this;
    ++(*this);
    return temp;
}


std::vector<std::string> DimFields::to_vector()
{
    switch (fields) {
        case XYZRGB:
            return dim_strings["XYZRGB"];
        case XYZI:
            return dim_strings["XYZI"];
        case XYZ:
        default:
            return dim_strings["XYZ"];
    }
}


DomainIterable::DomainIterable(const std::string &array_uri,
                               TileDBArrayBufferBuilder *builder, tiledb::Domain dom,
                               std::vector<std::string> dim_names)
                               : tiledb::Domain(dom), dim_names(dim_names)
{
    builder->build(array_uri, buffer, dim_names);
    dims = tiledb::Domain::dimensions();
}


DomainIterable::DomainIterable(const std::string &array_uri, TileDBArrayBufferBuilder *builder, std::vector<std::string> dim_names)
 : DomainIterable(array_uri, builder, tiledb::Array(tiledb::Context(), array_uri, TILEDB_READ).schema().domain(), dim_names)
{}


DomainIterator DomainIterable::begin()
{
    size_t pt_size = dim_names.empty() ? dims.size() : dim_names.size();
    return { &buffer[0], pt_size, dim_names };
}


DomainIterator DomainIterable::end()
{
    return { &buffer[buffer.size()] };
}


pclCloudGen* pclPtsCloudGenFactory::toCloudGen(DimFields &fields)
{
    switch (fields) {
        case DimFields::XYZ:
            return new pclPtXYZGen();
        case DimFields::XYZRGB:
            return new pclPtXYZRGBGen();
        case DimFields::XYZI:
            return new pclPtXYZIGen();
    }
}


void pclPtXYZGen::toCloud2(DomainIterable &dom, pcl::PCLPointCloud2 &cloud)
{
    auto cloud_xyz = pcl::PointCloud<pcl::PointXYZ>();
    for (auto it = dom.begin(); it != dom.end(); ++it)
    {
        cloud_xyz.push_back({static_cast<float>(it.dim(0)), static_cast<float>(it.dim(1)), static_cast<float>(it.dim(2))});
    }
    pcl::toPCLPointCloud2(cloud_xyz, cloud);
}


void pclPtXYZRGBGen::toCloud2(DomainIterable &dom, pcl::PCLPointCloud2 &cloud)
{
    auto cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>();
    for (auto it = dom.begin(); it != dom.end(); ++it)
    {
        cloud_xyzrgb.push_back(
                {
                        static_cast<float>(it.dim(0)),
                        static_cast<float>(it.dim(1)),
                        static_cast<float>(it.dim(2)),
                        static_cast<uint8_t>(it.dim(3)),
                        static_cast<uint8_t>(it.dim(4)),
                        static_cast<uint8_t>(it.dim(5))
                }
        );
    }
    pcl::toPCLPointCloud2(cloud_xyzrgb, cloud);
}


void pclPtXYZIGen::toCloud2(DomainIterable &dom, pcl::PCLPointCloud2 &cloud)
{
    auto cloud_xyzi = pcl::PointCloud<pcl::PointXYZI>();
    for (auto it = dom.begin(); it != dom.end(); ++it)
    {
        cloud_xyzi.push_back({static_cast<float>(it.dim(0)), static_cast<float>(it.dim(1)), static_cast<float>(it.dim(2)), static_cast<float>(it.dim(3))});
    }
    pcl::toPCLPointCloud2(cloud_xyzi, cloud);
}


int TileDBPCLReader::readHeader(const std::string &file_name, pcl::PCLPointCloud2 &cloud, Eigen::Vector4f &origin,
                                Eigen::Quaternionf &orientation, int &file_version, int &data_type,
                                unsigned int &data_idx, const int offset)
{
    tiledb::Context ctx;
    tiledb::Array array(ctx, file_name, TILEDB_READ);
    auto dim_names = fields.to_vector();
    int field_offset = 0;
    for (const auto& dim: array.schema().domain().dimensions())
    {
        if (DimNameIdxFinder()(dim_names, dim.name()) != -1)
        {
            pcl::PCLPointField f;
            f.name = dim.name();
            f.datatype = pcl::traits::asEnum<double>::value;
            f.offset = field_offset;
            field_offset += sizeof(double);
            f.count = static_cast<uint32_t>(1);
            cloud.fields.push_back(f);
        }
    }
    return 0;
}


int TileDBPCLReader::read(const std::string &file_name, pcl::PCLPointCloud2 &cloud, Eigen::Vector4f &origin,
                          Eigen::Quaternionf &orientation, int &file_version, const int offset)
{
    int data_type = sizeof(double); // TO FIX: this gets determined in readheader and keep as dummy variable
    unsigned int data_idx = 0; // same here;
    readHeader(file_name, cloud, origin, orientation, file_version, data_type, data_idx, offset);
    auto builder = new TileDBArrayBufferBuilder;
    std::vector<std::string> dim_names = fields.to_vector();
    DomainIterable iDom(file_name, builder, dim_names);
    cloud_gen_factory->toCloudGen(fields)->toCloud2(iDom, cloud);
    delete builder;
    return 0;
}


int TileDBPCLReader::read(const std::string &file_name, pcl::PCLPointCloud2 &cloud)
{
    Eigen::Vector4f origin = Eigen::Vector4f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    int file_version = 0;
    int offset = 0;
    return read(file_name, cloud, origin, orientation, file_version, offset);
}







int main() {
    std::string tdb_array = "<input_path>";
    std::string ply_path = "<output_path.ply>";
    tiledb::Context ctx;
    tiledb::Array array(ctx, tdb_array, TILEDB_READ);
    std::cout << array.schema() << std::endl;
    pcl::PCLPointCloud2* cloud2_ = new pcl::PCLPointCloud2;
    TileDBPCLReader r;
    r.set_dim_fields(DimFields::Fields::XYZ); // default: XYZ
    r.read(tdb_array, *cloud2_);
    pcl::io::savePLYFile(ply_path, *cloud2_);
    return 0;
}
