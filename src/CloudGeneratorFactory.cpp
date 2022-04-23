
#include "../include/CloudGeneratorFactory.hpp"


pclCloudGen* pclPtsCloud2GenFactory::toCloudGen(DimFields &fields)
{
    switch (fields) {
        case DimFields::XYZ:
            return new pclXYZCloud2Gen();
        case DimFields::XYZRGB:
            return new pclXYZRGBCloud2Gen();
        case DimFields::XYZI:
            return new pclXYZICloud2Gen();
    }
}