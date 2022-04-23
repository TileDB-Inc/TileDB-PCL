
#pragma once

#include <vector>
#include <string>


struct DimNameIdxFinder
{
    int operator()(const std::vector<std::string>& options, const std::string& input)
    {
        for (int i = 0; i < options.size(); i++)
            if (options.at(i).find(input) != std::string::npos)
                return i;
        return -1;
    }
};