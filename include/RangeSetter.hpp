
#pragma once

#include <vector>
#include <string>
#include <tiledb/array.h>
#include <tiledb/query.h>


class RangeSetter
{
private:
    tiledb::Query* q;

public:
    RangeSetter(tiledb::Query* q) : q(q) {}
    RangeSetter& addRange(const tiledb::Dimension& dim);
    RangeSetter& addBuffer(const tiledb::Dimension& dim, std::vector<double>& buff);
};


