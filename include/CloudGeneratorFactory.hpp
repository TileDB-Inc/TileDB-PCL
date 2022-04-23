
#pragma once

#include "../include/DimFields.hpp"
#include "../include/CloudGenerator.hpp"


class pclCloudGenFactory
{
public:
    virtual pclCloudGen* toCloudGen(DimFields&) = 0;
    virtual ~pclCloudGenFactory() = default;
};


class pclPtsCloud2GenFactory : public pclCloudGenFactory
{
public:
    pclCloudGen* toCloudGen(DimFields &fields) override;
};


