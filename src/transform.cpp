#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "typedefs.h"
#include "point_cloud2_iterator.h"

namespace pypcl {

namespace py = pybind11;

template <typename PointT>
py::object transform_pc_mat(typename pcl::PointCloud<PointT>::Ptr pc,
                            const ndarray2f& M,
                            bool inplace=false) {
  using PointCloudT = typename pcl::PointCloud<PointT>;
  if (!((M.shape(0) == 3 && M.shape(1) == 3) ||
        (M.shape(0) == 4 && M.shape(1) == 4))) {
    throw std::runtime_error("M must be 3x3 or 4x4");
  }

  Eigen::Matrix4f M2 = Eigen::Matrix4f::Identity();
  auto Mbuf = M.unchecked();
  for (int i=0; i < M.shape(0); ++i) {
    for (int j=0; j < M.shape(1); ++j) {
      M2(i, j) = Mbuf(i, j);
    }
  }

  // TODO there's also transfrompointcloudwithnormals
  if (inplace) {
    // not all filters allow in-place, transform does
    pcl::transformPointCloud(*pc, *pc, M2);
    return py::none();
  }

  typename PointCloudT::Ptr outpc(new PointCloudT);
  pcl::transformPointCloud(*pc, *outpc, M2);
  return py::cast(outpc);
}

template <typename PointT>
py::object transform_pc_rt(typename pcl::PointCloud<PointT>::Ptr pc,
                           const Eigen::Quaternionf& rotation,
                           const Eigen::Vector3f& translation,
                           bool inplace=false) {

  //Eigen::AngleAxisd( M_PI/4., Vector3d(0, 0, 1));

  using PointCloudT = typename pcl::PointCloud<PointT>;

  // TODO there's also transfrompointcloudwithnormals
  if (inplace) {
    // not all filters allow in-place, transform does
    pcl::transformPointCloud(*pc, *pc, translation, rotation);
    return py::none();
  }

  typename PointCloudT::Ptr outpc(new PointCloudT);
  pcl::transformPointCloud(*pc, *outpc, translation, rotation);
  return py::cast(outpc);
}


// TODO inplace for now
// TODO only float xyz for now
PCLPC2::Ptr transform_pclpc2_mat(pcl::PCLPointCloud2::Ptr pc,
                                 const ndarray2f& M) {
  if (!((M.shape(0) == 3 && M.shape(1) == 3) ||
        (M.shape(0) == 4 && M.shape(1) == 4))) {
    throw std::runtime_error("M must be 3x3 or 4x4");
  }

  PCLPC2::Ptr outpc(new PCLPC2(*pc));

  Eigen::Matrix4f M2 = Eigen::Matrix4f::Identity();
  // in case input is 3x3 rot matrix
  auto Mbuf = M.unchecked();
  for (int i=0; i < M.shape(0); ++i) {
    for (int j=0; j < M.shape(1); ++j) {
      M2(i, j) = Mbuf(i, j);
    }
  }
  //py::print("foo");

  PointCloud2Iterator<float> iter_x(*outpc, "x");
  while (iter_x != iter_x.end()) {
    Eigen::Vector4f p(iter_x[0],
                      iter_x[1],
                      iter_x[2],
                      1.);
    Eigen::Vector4f p2 = M2*p;
    iter_x[0] = p2.x();
    iter_x[1] = p2.y();
    iter_x[2] = p2.z();
    ++iter_x;
  }
  return outpc;
}

// TODO only float xyz for now
PCLPC2::Ptr transform_pclpc2_rt(pcl::PCLPointCloud2::Ptr pc,
                         const Eigen::Quaternionf& rotation,
                         const Eigen::Vector3f& translation) {
  //AngleAxisd( M_PI/4., Vector3d(0, 0, 1));

  Eigen::Affine3f M = Eigen::Translation3f(translation)*rotation;
  PCLPC2::Ptr outpc(new PCLPC2(*pc));

  PointCloud2Iterator<float> iter_x(*outpc, "x");
  while (iter_x != iter_x.end()) {
    Eigen::Vector3f p(iter_x[0],
                      iter_x[1],
                      iter_x[2]);
    Eigen::Vector3f p2 = M*p;
    iter_x[0] = p2.x();
    iter_x[1] = p2.y();
    iter_x[2] = p2.z();
    ++iter_x;
  }
  return outpc;
}


void export_transform(py::module& m) {
  m.def("transform_pcxyz_rt",
        &transform_pc_mat<pcl::PointXYZ>,
        py::arg("pc"),
        py::arg("M"),
        py::arg("inplace")=false);

  m.def("transform_pcxyz_mat",
        &transform_pc_rt<pcl::PointXYZ>,
        py::arg("pc"),
        py::arg("rotation"),
        py::arg("translation"),
        py::arg("inplace")=false);

  m.def("transform_pclpc2_mat",
        &transform_pclpc2_mat,
        py::arg("pc"),
        py::arg("M"));

  m.def("transform_pclpc2_rt",
        &transform_pclpc2_rt,
        py::arg("pc"),
        py::arg("rotation"),
        py::arg("translation"));

}

}
