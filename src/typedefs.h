#ifndef TYPEDEFS_H_6L21M3XB
#define TYPEDEFS_H_6L21M3XB

#include <cstdint>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pybind11/numpy.h>
#include <boost/shared_ptr.hpp>

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);

namespace pypcl {

namespace py = pybind11;

using PCXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PCXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using PCXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PCXYZL = pcl::PointCloud<pcl::PointXYZL>;
using PCN = pcl::PointCloud<pcl::Normal>;
using PCLPC2 = pcl::PCLPointCloud2;

// PointXYZRGBNormal
// PointSurfel

using ndarrayXf = py::array_t<float>;
using ndarray1f = py::array_t<float, 1>;
using ndarray2f = py::array_t<float, 2>;
using ndarray3f = py::array_t<float, 3>;

using ndarray1i = py::array_t<int, 1>;
using ndarray2i = py::array_t<int, 2>;
using ndarray3i = py::array_t<int, 3>;

using ndarray2u1 = py::array_t<std::uint8_t, 2>;
using ndarray3u1 = py::array_t<std::uint8_t, 3>;

using ndarray2u2 = py::array_t<std::uint16_t, 2>;
using ndarray3u2 = py::array_t<std::uint16_t, 3>;

}

#endif /* end of include guard: TYPEDEFS_H_6L21M3XB */
