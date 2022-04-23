
#pragma once

#include <vector>
#include <string>
#include <tiledb/array.h>
#include <tiledb/query.h>
#include "DimNameIdxFinder.hpp"
#include "RangeSetter.hpp"


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


