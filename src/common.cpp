#include <cstdint>
#include <cmath>
#include <limits>

#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <boost/shared_ptr.hpp>

#include "point_cloud2_iterator.h"
#include "typedefs.h"
#include "util.h"

namespace pypcl {

namespace py = pybind11;

// computemeanandcovariancematrix
// demeanpointcloud
// getminmax3d
// getPointCloudAsEigen(PCLPointCloud2 in, Eigen::MatrixXf out)
// getEigenAsPointCloud(Eigen::MatrixXf in, PCLPointCloud2 out)
// PCA
// posesfrommatches

// removenan can be done with passthrough_filter

#if 0
// disparity to xyz
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new
                                              pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::RGB>::Ptr left_image (new
                                             pcl::PointCloud<pcl::RGB>);
// Fill left image cloud.
// pcl::DisparityMapConverter<pcl::PointXYZI> dmc;
// dmc.setBaseline (0.8387445f);
// dmc.setFocalLength (368.534700f);
// dmc.setImageCenterX (318.112200f);
// dmc.setImageCenterY (224.334900f);
// dmc.setDisparityThresholdMin(15.0f);
// // Left view of the scene.
// dmc.setImage (left_image);
// // Disparity map of the scene.
// dmc.loadDisparityMap ("disparity_map.txt", 640, 480);
// dmc.compute(*cloud);
#endif

#if 0
// disparity to dem
pcl::PointCloud<pcl::PointDEM>::Ptr cloud (new
                                               pcl::PointCloud<pcl::PointDEM>);
pcl::PointCloud<pcl::RGB>::Ptr left_image (new
                                             pcl::PointCloud<pcl::RGB>);
// Fill left image cloud.
// pcl::DigitalElevationMapBuilder demb;
// demb.setBaseline (0.8387445f);
// demb.setFocalLength (368.534700f);
// demb.setImageCenterX (318.112200f);
// demb.setImageCenterY (224.334900f);
// demb.setDisparityThresholdMin (15.0f);
// demb.setDisparityThresholdMax (80.0f);
// demb.setResolution (64, 32);
// // Left view of the scene.
// demb.loadImage (left_image);
// // Disparity map of the scene.
// demb.loadDisparityMap ("disparity_map.txt", 640, 480);
// demb.compute(*cloud);
#endif

// GroundPlaneComparator to beused with OrganizedConnectedComponentSegmentation
// TSDFVolume
// generalized icp
// voxelgridlabel(pointxyzrgbl)
// voxelgridcovariance (PointT) used for ndt3d
// medianfilter for organized xyz
// covariancesampling(pointT, pointNT)
// organizededgefronormals??
// PointRepresentation<PointT>::copyToFloatArray
// pclCorrespondence(int,int)
// getApproximateIndices(PointT)
// pointToPlaneDistance, projectPoint(to plane)
// extractEuclideanClusters(PointT)

#if 0
py::tuple pclpc2_minmax(const PCLPC2::Ptr pc) {
  std::vector<int> valid_ix;
  get_valid_xyz_indices(*pc, valid_ix);
  if (valid_ix.empty()) {
    return py::make_tuple(py::none, py::none);
  }

  PointCloud2ConstIterator<float> iter_x0(*pc, "x");
  auto iter_x = iter_x0 + valid_ix;
  Eigen::Vector4f minpt;
  Eigen::Vector4f maxpt;
  for (int i : valid_ix) {
    auto iter_x = iter_x0;
    iter_x += i;
    //py::print("i=", i, "iter_x[0] = ", iter_x[0]);
    Eigen::Vector4f p(iter_x[0],
                      iter_x[1],
                      iter_x[2],
                      0.);

  }
}
#endif

Eigen::Vector3f pclpc2_centroid(const PCLPC2::Ptr pc) {
  std::vector<int> valid_ix;
  get_valid_xyz_indices(*pc, valid_ix);
  if (valid_ix.empty()) {
    return Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(),
                           std::numeric_limits<float>::quiet_NaN(),
                           std::numeric_limits<float>::quiet_NaN());

  }
  float n = static_cast<float>(valid_ix.size());

  PointCloud2ConstIterator<float> iter_x0(*pc, "x");
  Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
  for (int i : valid_ix) {
    auto iter_x = iter_x0;
    iter_x += i;
    // TODO use Map
    Eigen::Vector4f p(iter_x[0],
                      iter_x[1],
                      iter_x[2],
                      0.);
    centroid += (p/n);
  }
  return centroid.head<3>();
}

void export_common(py::module& m) {
  m.def("pclpc2_centroid",
        &pclpc2_centroid,
        py::arg("pc"));
}

}
