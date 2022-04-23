
#pragma once

#include <vector>
#include <string>
#include <unordered_map>


class DimFields
{
    std::unordered_map<std::string, std::vector<std::string>> dim_strings // this can be extracted into another class;
    {
            {"XYZ", {" X x ", " Y y ", " Z z "}},
            {"XYZRGB", {" X x ", " Y y ", " Z z ", " R r RED red Red ", " G g GREEN green Green GR gr Gr ", " B b BLUE blue Blue BL bl Bl "}},
            {"XYZI", {" X x ", " Y y ", " Z z ", " I i INTENSITY intensity Intensity "}}
    };
public:
    enum Fields
    {
        XYZ,
        XYZRGB,
        XYZI
    } fields;

    DimFields() : fields(XYZ) {}

    DimFields(Fields f) : fields(f) {}

    operator Fields() const { return fields; }

    std::vector<std::string> to_vector();

    constexpr bool operator==(DimFields &rhs) const {
        return fields == rhs.fields;
    }

    constexpr bool operator!=(DimFields &rhs) const {
        return rhs != *this;
    }
};



