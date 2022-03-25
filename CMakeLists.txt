cmake_minimum_required(VERSION 3.18)
project(TileDB_PCL)

set(CMAKE_CXX_STANDARD 14)
set(PCL_DIR /usr/local/Cellar/pcl/1.12.1_1/share/pcl-1.12/)
set(TileDB_DIR /usr/local/Cellar/tiledb/2.5.0/lib/cmake/TileDB/)
find_package(TileDB REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES src/main.cpp src/tiledb_write.cpp include/main.hpp include/tiledb_write.hpp)
add_executable(TileDB_PCL ${SOURCES})
target_include_directories(TileDB_PCL PRIVATE src include)

target_link_libraries(TileDB_PCL PRIVATE ${PCL_LIBRARIES} TileDB::tiledb_shared)
