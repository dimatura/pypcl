#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <pcl/common/common.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/kdtree/kdtree.h>

#include "typedefs.h"

namespace pypcl {

namespace py = pybind11;

PCLPC2::Ptr integral_image_normals_estimation(const PCLPC2::Ptr organized_pc,
                                              const std::string method,
                                              float max_depth_change_factor,
                                              float normal_smoothing_size,
                                              bool depth_dependent_smoothing) {

  PCXYZ::Ptr xyz(new PCXYZ);
  pcl::fromPCLPointCloud2(*organized_pc, *xyz);

  pcl::PointCloud<pcl::Normal> normals;
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  if (method=="average_3d_gradient") {
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
  } else if (method=="covariance_matrix") {
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  } else if (method=="average_depth_change") {
    ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
  } else {
    throw py::value_error("method not understood");
  }
  ne.setNormalSmoothingSize(normal_smoothing_size);
  ne.setMaxDepthChangeFactor(max_depth_change_factor);
  //ne.setRectSize(8, 8); // width height
  ne.setDepthDependentSmoothing(depth_dependent_smoothing); // not sure what this does
  //ne.useSensorOriginAsViewPoint();
  ne.setInputCloud(xyz);
  ne.compute(normals);

  PCLPC2::Ptr output(new PCLPC2);
  PCLPC2 output_normals;
  pcl::toPCLPointCloud2(normals, output_normals);
  pcl::concatenateFields(*organized_pc, output_normals, *output);
  return output;
}


PCLPC2::Ptr normals_estimation(const PCLPC2::Ptr pc, float radius) {
  PCXYZ::Ptr xyz(new PCXYZ);
  pcl::fromPCLPointCloud2(*pc, *xyz);
  //float radius = 2.0f;
  pcl::PointCloud<pcl::Normal> normals;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(radius);
  ne.setInputCloud(xyz);
  ne.compute(normals);

  PCLPC2::Ptr output(new PCLPC2);
  PCLPC2 output_normals;
  pcl::toPCLPointCloud2(normals, output_normals);
  pcl::concatenateFields(*pc, output_normals, *output);
  // TODO normal refinement, see ref/normref.cpp
  return output;
}

void export_features(py::module& m) {
  m.def("integral_image_normals_estimation",
        &integral_image_normals_estimation,
        py::arg("organized_pc"),
        py::arg("method") = "average_3d_gradient",
        py::arg("max_depth_change_factor") = 0.02f,
        py::arg("normal_smoothing_size") = 0.2f,
        py::arg("depth_dependent_smoothing") = true);

  m.def("normals_estimation",
        &normals_estimation,
        py::arg("pc"),
        py::arg("radius") = 2.0);
}

}
