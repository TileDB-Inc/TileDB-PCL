//
// Created by Chloe T on 2/14/22.
//

#ifndef UNTITLED1_MAIN_HPP
#define UNTITLED1_MAIN_HPP


#include <unordered_map>


class Printer
{
public:
    template<typename T>
    void print_all(const std::vector<T>& vec)
    {
        std::cout << "[ ";
        for (size_t i = 0; i < vec.size() - 1; i++)
            std::cout << vec.at(i) << ", ";
        std::cout << vec.at(vec.size() - 1) << " ]" << std::endl;
    }
    template<typename T>
    void print_all(const std::pair<T, T>& pair)
    {
        std::cout << "[ "
                  << pair.first << ", " << pair.second
                  << " ]" << std::endl;
    }
};





class TileDBPCLWriter: public pcl::FileWriter
{

};

#endif //UNTITLED1_MAIN_HPP
