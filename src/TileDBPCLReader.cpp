
#include "../include/TileDBPCLReader.hpp"


TileDBPCLReader::TileDBPCLReader(DimFields _fields) : fields(_fields) {}


TileDBPCLReader::TileDBPCLReader(DimFields _fields, bool includeAttrs) : fields(_fields), include_attrs(includeAttrs) {}


TileDBPCLReader::TileDBPCLReader(DimFields _fields, pclCloudGenFactory *cgf, bool includeAttrs) : fields(_fields), cloud_gen_factory(cgf), include_attrs(includeAttrs) {}


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