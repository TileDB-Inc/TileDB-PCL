#pragma once

#include <vector>
#include <string>
#include <tiledb/array.h>
#include "BufferBuilder.hpp"
#include "DomainIterator.hpp"
#include "DimNameIdxFinder.hpp"


class DomainIterable: public tiledb::Domain
{
private:
    std::vector<double> buffer;
    std::vector<tiledb::Dimension> dims;
    std::vector<std::string> dim_names;
    std::vector<tiledb_datatype_t> dim_types;

public:
    DomainIterable(const std::string& array_uri, TileDBArrayBufferBuilder* builder, tiledb::Domain dom, std::vector<std::string> dim_names = {});
    DomainIterable(const std::string& array_uri, TileDBArrayBufferBuilder* builder, std::vector<std::string> dim_names = {});
    DomainIterator begin();
    DomainIterator end();
    size_t ndims() { return dim_names.empty() ? dims.size() : dim_names.size(); }
    std::vector<tiledb::Dimension> dimensions() { return dims; }
};


