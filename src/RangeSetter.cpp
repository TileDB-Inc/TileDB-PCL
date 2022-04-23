
#include "../include/RangeSetter.hpp"


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