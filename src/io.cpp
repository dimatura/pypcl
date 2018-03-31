#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include "typedefs.h"
#include "tinyformat.h"


namespace pypcl {

namespace py = pybind11;

//PC_XYZ::Ptr load_pcd_pcxyz(const std::string& fname) {

template<class PointCloudT>
typename PointCloudT::Ptr load_pcd(const std::string& fname) {
  typename PointCloudT::Ptr cloud(new PointCloudT);
  using PointT = typename PointCloudT::PointType;
  if (pcl::io::loadPCDFile<PointT>(fname, *cloud) == -1) {
    std::string errmsg = tfm::format("error loading %s", fname);
    throw std::runtime_error(errmsg);
  }
  return cloud;
}

template<class PointCloudT>
void save_pcd(const typename PointCloudT::Ptr cloud,
              const std::string& fname,
              const std::string& mode) {

  using PointT = typename PointCloudT::PointType;
  int ret = 0;
  if (mode == "ascii") {
    ret = pcl::io::savePCDFile<PointT>(fname, *cloud, false);
  } else if (mode == "binary") {
    ret = pcl::io::savePCDFile<PointT>(fname, *cloud, true);
  } else if (mode == "binary_compressed") {
    ret = pcl::io::savePCDFileBinaryCompressed<PointT>(fname, *cloud);
  } else {
    throw std::runtime_error("unknown save mode");
  }

  if (ret == -1) {
    std::string errmsg = tfm::format("error loading %s", fname);
    throw std::runtime_error(errmsg);
  }
}

py::object load_pcd_pclpc2(const std::string& fname, bool load_viewpoint) {
  PCLPC2::Ptr cloud(new PCLPC2);

  int ret;
  Eigen::Vector4f translation;
  Eigen::Quaternionf orientation;
  py::object output;
  if (load_viewpoint) {
    ret = pcl::io::loadPCDFile(fname, *cloud, translation, orientation);
    output = py::make_tuple(cloud, translation, orientation);
  } else {
    ret = pcl::io::loadPCDFile(fname, *cloud);
    output = py::cast(cloud);
  }
  if (ret < 0) {
    std::string errmsg = tfm::format("error loading %s", fname);
    throw std::runtime_error(errmsg);
  }
  return output;
}

void save_pcd_pclpc2(const PCLPC2::Ptr cloud,
                     const std::string& fname,
                     const std::string& mode,
                     const Eigen::Vector4f& origin,
                     const Eigen::Quaternionf& orientation) {
  int ret = 0;
  pcl::PCDWriter writer;

  if (mode == "ascii") {
    ret = writer.writeASCII(fname, *cloud, origin, orientation);
  } else if (mode == "binary") {
    ret = writer.writeBinary(fname, *cloud, origin, orientation);
  } else if (mode == "binary_compressed") {
    ret = writer.writeBinaryCompressed(fname, *cloud, origin, orientation);
  } else {
    throw std::runtime_error("unknown save mode");
  }

  if (ret == -1) {
    std::string errmsg = tfm::format("error loading %s", fname);
    throw std::runtime_error(errmsg);
  }
}

void export_io(py::module& m) {
  m.def("load_pcd_pcxyz",
        &load_pcd<PCXYZ>,
        py::arg("fname"));

  m.def("save_pcd_pcxyz",
        &save_pcd<PCXYZ>,
        py::arg("cloud"),
        py::arg("fname"),
        py::arg("mode") = "binary_compressed");

  m.def("load_pcd_pclpc2",
        &load_pcd_pclpc2,
        py::arg("fname"),
        py::arg("load_viewpoint")=false);

  m.def("save_pcd_pclpc2",
        &save_pcd_pclpc2,
        py::arg("cloud"),
        py::arg("fname"),
        py::arg("mode") = "binary_compressed",
        py::arg("origin") = Eigen::Vector4f::Zero(),
        py::arg("orientation") = Eigen::Quaternionf::Identity());
}

}
