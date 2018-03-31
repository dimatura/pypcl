cmake_minimum_required(VERSION 2.8.12)
project(pypcl)

#find_package(NUMPY REQUIRED)
find_package(PCL REQUIRED
             common
             features
             filters
             io
             registration
             sample_consensus
             segmentation
             surface
             )

add_subdirectory(pybind11)

pybind11_add_module(libpypcl
    src/module.cpp
    src/pointcloud.cpp
    src/io.cpp
    src/filters.cpp
    src/transform.cpp
    src/depth.cpp
    src/features.cpp
    #src/registration.cpp
)

include_directories(
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(libpypcl
    PRIVATE
    ${PCL_LIBRARIES}
)
