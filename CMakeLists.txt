cmake_minimum_required(VERSION 3.18)
project(TileDB_PCL)

set(CMAKE_CXX_STANDARD 17)
set(PCL_DIR /usr/local/Cellar/pcl/1.12.1_2/share/pcl-1.12/)
set(Qt5_DIR /usr/local/Cellar/qt@5/5.15.3/lib/cmake/Qt5/)
set(TileDB_DIR /usr/local/Cellar/tiledb/2.5.0/lib/cmake/TileDB/)
find_package(TileDB REQUIRED)
find_package(VTK COMPONENTS REQUIRED ViewsQt GUISupportQt)
find_package(PCL 1.3 REQUIRED COMPONENTS common io VISUALIZATION)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES src/main.cpp src/tiledb_write.cpp src/visualization.cpp include/main.hpp include/tiledb_write.hpp include/visualization.hpp src/DomainIterable.cpp include/DomainIterable.hpp src/DomainIterator.cpp include/DomainIterator.hpp src/RangeSetter.cpp include/RangeSetter.hpp src/BufferBuilder.cpp include/BufferBuilder.hpp include/DimNameIdxFinder.hpp src/CloudGenerator.cpp include/CloudGenerator.hpp src/CloudGeneratorFactory.cpp include/CloudGeneratorFactory.hpp src/DimFields.cpp include/DimFields.hpp include/CloudBuilder.hpp src/TileDBPCLReader.cpp include/TileDBPCLReader.hpp src/CloudBuilder.cpp)
add_executable(TileDB_PCL ${SOURCES})
target_include_directories(TileDB_PCL PRIVATE src include)

target_link_libraries(TileDB_PCL PRIVATE ${PCL_LIBRARIES} TileDB::tiledb_shared)
