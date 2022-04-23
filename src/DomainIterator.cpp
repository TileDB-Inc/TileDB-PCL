
#include "../include/DomainIterator.hpp"


double DomainIterator::dim(size_t idx) { return *(m_ptr + idx); }


double DomainIterator::dim(const std::string &name)
{
    if (m_dim_names.empty()) return double(-1);
    size_t idx = DimNameIdxFinder()(m_dim_names, name);
    if (idx < 0) return double(-1);
    return dim(idx);
}


tiledb_datatype_t DomainIterator::dim_type(size_t idx)
{
    return m_dim_types[idx];
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