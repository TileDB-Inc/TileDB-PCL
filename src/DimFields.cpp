
#include "../include/DimFields.hpp"


std::vector<std::string> DimFields::to_vector()
{
    switch (fields) {
        case XYZRGB:
            return dim_strings["XYZRGB"];
        case XYZI:
            return dim_strings["XYZI"];
        case XYZ:
        default:
            return dim_strings["XYZ"];
    }
}