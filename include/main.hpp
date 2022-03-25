//
// Created by Chloe T on 2/14/22.
//

#ifndef UNTITLED1_MAIN_HPP
#define UNTITLED1_MAIN_HPP


#include <unordered_map>


class Printer
{
public:
    template<typename T>
    void print_all(const std::vector<T>& vec)
    {
        std::cout << "[ ";
        for (size_t i = 0; i < vec.size() - 1; i++)
            std::cout << vec.at(i) << ", ";
        std::cout << vec.at(vec.size() - 1) << " ]" << std::endl;
    }
    template<typename T>
    void print_all(const std::pair<T, T>& pair)
    {
        std::cout << "[ "
                  << pair.first << ", " << pair.second
                  << " ]" << std::endl;
    }
};


struct DimNameIdxFinder
{
    int operator()(const std::vector<std::string>& options, const std::string& input)
    {
        for (int i = 0; i < options.size(); i++)
            if (options.at(i).find(input) != std::string::npos)
                return i;
        return -1;
    }
};


class RangeSetter
{
private:
    tiledb::Query* q;

public:
    RangeSetter(tiledb::Query* q) : q(q) {}
    RangeSetter& addRange(const tiledb::Dimension& dim);
    RangeSetter& addBuffer(const tiledb::Dimension& dim, std::vector<double>& buff);
};


class BufferBuilder
{
public:
    virtual ~BufferBuilder() = default;
    virtual void build(const std::string& filename, std::vector<double>& buffer, const std::vector<std::string>& dim_names = {});
    virtual void build(const std::string& filename, std::vector<uint8_t>& buffer, const std::vector<std::string>& dim_names = {});
    virtual void build(const std::string& filename, std::vector<int64_t>& buffer, const std::vector<std::string>& dim_names = {});
};


class TileDBArrayBufferBuilder
{
private:
    std::vector<std::vector<double>> i_buff;
    int m_null_val;
    void build_ibuff(const std::string& filename, const std::vector<std::string>& dim_names = {});

public:
    TileDBArrayBufferBuilder() : m_null_val(0) {}
    TileDBArrayBufferBuilder(int m_null_val) : m_null_val(m_null_val) {}
    void build(const std::string& filename, std::vector<double>& buff, const std::vector<std::string>& dim_names = {});
};


class DomainIterator
{
public:
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = double;
    using pointer = double*;
    using reference = double&;

private:
    pointer m_ptr;
    size_t m_pt_size;
    std::vector<std::string> m_dim_names;

public:
    DomainIterator(pointer ptr, size_t pt_size = 3, const std::vector<std::string> dim_names = {}) : m_ptr(ptr), m_pt_size(pt_size), m_dim_names(dim_names) {}
    double dim(size_t idx);
    double dim(const std::string& name);
    reference operator*();
    pointer operator->();
    DomainIterator& operator++();
    DomainIterator operator++(int);
    friend bool operator==(const DomainIterator& a, const DomainIterator& b) { return a.m_ptr == b.m_ptr; }
    friend bool operator!=(const DomainIterator& a, const DomainIterator& b) { return a.m_ptr != b.m_ptr; };
};


class DomainIterable;
class DimFields;


class pclCloudGen
{
public:
   virtual void toCloud2(DomainIterable&, pcl::PCLPointCloud2&) = 0;
   virtual ~pclCloudGen() = default;
};


class pclPtsCloudGen : public pclCloudGen
{};


class pclPtXYZGen : public pclPtsCloudGen
{
public:
    void toCloud2(DomainIterable &iterable, pcl::PCLPointCloud2 &cloud2) override;
};


class pclPtXYZRGBGen : public pclPtsCloudGen
{
public:
    void toCloud2(DomainIterable &iterable, pcl::PCLPointCloud2 &cloud2) override;
};


class pclPtXYZIGen : public pclPtsCloudGen
{
public:
    void toCloud2(DomainIterable &iterable, pcl::PCLPointCloud2 &cloud2) override;
};


class pclCloudGenFactory
{
public:
    virtual pclCloudGen* toCloudGen(DimFields&) = 0;
    virtual ~pclCloudGenFactory() = default;
};


class pclPtsCloudGenFactory : public pclCloudGenFactory
{
public:
    pclCloudGen* toCloudGen(DimFields &fields) override;
};


class DimFields
{
    std::unordered_map<std::string, std::vector<std::string>> dim_strings // this can be extracted into another class;
    {
        {"XYZ", {" X x ", " Y y ", " Z z "}},
        {"XYZRGB", {" X x ", " Y y ", " Z z ", " R r RED red Red ", " G g GREEN green Green GR gr Gr ", " B b BLUE blue Blue BL bl Bl "}},
        {"XYZI", {" X x ", " Y y ", " Z z ", " I i INTENSITY intensity Intensity "}}
    };
public:
    enum Fields
    {
        XYZ,
        XYZRGB,
        XYZI
    } fields;

    DimFields() : fields(XYZ) {}

    DimFields(Fields f) : fields(f) {}

    operator Fields() const { return fields; }

    std::vector<std::string> to_vector();

    constexpr bool operator==(DimFields &rhs) const {
        return fields == rhs.fields;
    }

    constexpr bool operator!=(DimFields &rhs) const {
        return rhs != *this;
    }
};


class DomainIterable: public tiledb::Domain
{
private:
    std::vector<double> buffer;
    std::vector<tiledb::Dimension> dims;
    std::vector<std::string> dim_names;

public:
    DomainIterable(const std::string& array_uri, TileDBArrayBufferBuilder* builder, tiledb::Domain dom, std::vector<std::string> dim_names = {});
    DomainIterable(const std::string& array_uri, TileDBArrayBufferBuilder* builder, std::vector<std::string> dim_names = {});
    DomainIterator begin();
    DomainIterator end();
    int ndims() { return dims.size(); }
    std::vector<tiledb::Dimension> dimensions() { return dims; }
};


class TileDBPCLReader: public pcl::FileReader
{
public:
    TileDBPCLReader() = default;

    explicit TileDBPCLReader(DimFields _fields) : fields(_fields) {}

    TileDBPCLReader(DimFields _fields, bool includeAttrs) : fields(_fields), include_attrs(includeAttrs) {}

    TileDBPCLReader(DimFields _fields, pclCloudGenFactory* cgf, bool includeAttrs = false) : fields(_fields), cloud_gen_factory(cgf), include_attrs(includeAttrs) {}

    int readHeader(const std::string &file_name, pcl::PCLPointCloud2 &cloud, Eigen::Vector4f &origin,
                   Eigen::Quaternionf &orientation, int &file_version, int &data_type, unsigned int &data_idx,
                   const int offset) override;

    int read(const std::string &file_name, pcl::PCLPointCloud2 &cloud, Eigen::Vector4f &origin,
             Eigen::Quaternionf &orientation, int &file_version, const int offset) override;

    int read(const std::string& file_name, pcl::PCLPointCloud2& cloud);

    void set_dim_fields(DimFields _fields) { fields = _fields; }

    void set_include_attrs(bool include) { include_attrs = include; }

private:
    DimFields fields{DimFields::XYZ};
    bool include_attrs{false};
    pclCloudGenFactory* cloud_gen_factory{new pclPtsCloudGenFactory()};
};


class TileDBPCLWriter: public pcl::FileWriter
{

};

#endif //UNTITLED1_MAIN_HPP
