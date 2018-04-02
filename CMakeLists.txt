cmake_minimum_required(VERSION 2.8.12)
project(pypcl)

#find_package(NUMPY REQUIRED)
find_package(PCL REQUIRED
             common
             features
             filters
             kdtree
             io
             registration
             search
             sample_consensus
             segmentation
             surface
             )

#find_package(PCL REQUIRED)

add_subdirectory(pybind11)

pybind11_add_module(libpypcl
    src/depth.cpp
    src/features.cpp
    src/filters.cpp
    src/io.cpp
    src/module.cpp
    src/pointcloud.cpp
    src/transform.cpp
    src/registration.cpp
)

#message("pcl_libraries: " ${PCL_LIBRARIES})

link_directories(${PCL_LIBRARY_DIRS})

include_directories(
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(libpypcl
    PRIVATE
    ${PCL_LIBRARIES}
)
