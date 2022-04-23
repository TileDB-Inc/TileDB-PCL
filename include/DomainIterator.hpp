
#pragma once

#include <vector>
#include <string>
#include <tiledb/type.h>
#include "DimNameIdxFinder.hpp"


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
    std::vector<tiledb_datatype_t> m_dim_types;

public:
    DomainIterator(pointer ptr, size_t pt_size = 3, const std::vector<std::string> dim_names = {}, const std::vector<tiledb_datatype_t> dim_types = { TILEDB_FLOAT64, TILEDB_FLOAT64, TILEDB_FLOAT64 }) : m_ptr(ptr), m_pt_size(pt_size), m_dim_names(dim_names), m_dim_types(dim_types) {}
    double dim(size_t idx);
    double dim(const std::string& name);
    tiledb_datatype_t dim_type(size_t idx);
    reference operator*();
    pointer operator->();
    DomainIterator& operator++();
    DomainIterator operator++(int);
    friend bool operator==(const DomainIterator& a, const DomainIterator& b) { return a.m_ptr == b.m_ptr; }
    friend bool operator!=(const DomainIterator& a, const DomainIterator& b) { return a.m_ptr != b.m_ptr; };
};


