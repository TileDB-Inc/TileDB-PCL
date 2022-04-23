
#include "../include/DomainIterable.hpp"


DomainIterable::DomainIterable(const std::string &array_uri,
                               TileDBArrayBufferBuilder *builder, tiledb::Domain dom,
                               std::vector<std::string> dim_names)
        : tiledb::Domain(dom), dim_names(dim_names)
{
    builder->build(array_uri, buffer, dim_names);
    dims = tiledb::Domain::dimensions();
    for (const auto& dim: dims)
    {
        if (this->dim_names.empty() || DimNameIdxFinder()(this->dim_names, dim.name()) != -1)
            dim_types.push_back(dim.type());
    }
}


DomainIterable::DomainIterable(const std::string &array_uri, TileDBArrayBufferBuilder *builder, std::vector<std::string> dim_names)
        : DomainIterable(array_uri, builder, tiledb::Array(tiledb::Context(), array_uri, TILEDB_READ).schema().domain(), dim_names)
{}


DomainIterator DomainIterable::begin()
{
    return { &buffer[0], ndims(), dim_names, dim_types };
}


DomainIterator DomainIterable::end()
{
    return { &buffer[buffer.size()] };
}
