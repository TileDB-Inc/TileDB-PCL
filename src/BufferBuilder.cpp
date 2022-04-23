
#include "../include/BufferBuilder.hpp"


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