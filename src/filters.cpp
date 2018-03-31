
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

//#include <pcl/filters/project_inliers.h>

#include <boost/shared_ptr.hpp>

#include "typedefs.h"

namespace pypcl {

PCLPC2::Ptr passthrough_filter(const PCLPC2::Ptr pc,
                               std::string field,
                               float minval,
                               float maxval,
                               bool keep_organized) {
  // unintuitive name; it can also crop cloud.
  PCLPC2::Ptr pc_out(new PCLPC2());
  pcl::PassThrough<PCLPC2> pass;
  pass.setInputCloud(pc);
  pass.setFilterFieldName(field);
  pass.setFilterLimits(minval, maxval);
  pass.setKeepOrganized(keep_organized);
  pass.filter(*pc_out);
  return pc_out;
}

PCLPC2::Ptr extract_indices(const PCLPC2::Ptr pc,
                            const ndarray1i& indices,
                            bool set_negative,
                            bool keep_organized) {
  auto indices_buf = indices.unchecked();
  std::vector<int> indices2(indices_buf.data(0),
                            indices_buf.data(0)+indices_buf.size());
  pcl::ExtractIndices<PCLPC2> extract;
  extract.setInputCloud(pc);
  PCLPC2::Ptr pc_out(new PCLPC2());
  extract.setNegative(set_negative);
  extract.setKeepOrganized(keep_organized);
  //extract.filterDirectly
  extract.filter(*pc_out);
  return pc_out;
}

// see tutorial pcl_tutorial.pdf in dropbox
PCLPC2::Ptr voxel_grid_filter(const PCLPC2::Ptr pc,
                              float leaf_size) {
  // TODO setDownsampleAllData
  PCLPC2::Ptr pc_out(new PCLPC2());
  pcl::VoxelGrid<PCLPC2> filter;
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  //filter.setMinimumPointsPerVoxel(min_points_per_voxel);
  filter.setInputCloud(pc);
  filter.filter(*pc_out);
  return pc_out;
}

PCLPC2::Ptr statistical_outlier_removal(const PCLPC2::Ptr pc,
                                        int meank,
                                        float stddev,
                                        bool keep_organized) {
  // for radius, params. radius 0.05, minneighbors 800

  PCXYZ::Ptr xyz_cloud_pre(new PCXYZ()), xyz_cloud(new PCXYZ());
  PCLPC2::Ptr output(new PCLPC2());
  pcl::fromPCLPointCloud2(*pc, *xyz_cloud_pre);
  pcl::PointIndices::Ptr removed_indices(new pcl::PointIndices),
                         indices(new pcl::PointIndices);
  std::vector<int> valid_indices;
  if (keep_organized) {
    xyz_cloud = xyz_cloud_pre;
    for (int i = 0; i < int (xyz_cloud->size()); ++i) {
      valid_indices.push_back(i);
    }
  } else {
    pcl::removeNaNFromPointCloud<pcl::PointXYZ>(*xyz_cloud_pre, *xyz_cloud, valid_indices);
  }

  PCXYZ::Ptr xyz_cloud_filtered(new PCXYZ());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter(true);
  filter.setInputCloud(xyz_cloud);
  filter.setMeanK(meank);
  filter.setStddevMulThresh(stddev);
  filter.setNegative(false);
  filter.setKeepOrganized(keep_organized);
  filter.filter(*xyz_cloud_filtered);
  filter.getRemovedIndices(*removed_indices);

  if (keep_organized) {
    pcl::PCLPointCloud2 output_filtered;
    pcl::toPCLPointCloud2(*xyz_cloud_filtered, output_filtered);
    pcl::concatenateFields(*pc, output_filtered, *output);
  } else {
    for (size_t i = 0; i < removed_indices->indices.size(); ++i) {
      indices->indices.push_back(valid_indices[removed_indices->indices[i]]);
    }
    pcl::ExtractIndices<pcl::PCLPointCloud2> ei;
    ei.setInputCloud(pc);
    ei.setIndices(indices);
    ei.setNegative(true);
    ei.filter(*output);
  }

  return output;
}

PCLPC2::Ptr crop_box(PCLPC2::Ptr pc,
                     const Eigen::Vector4f& minpt,
                     const Eigen::Vector4f& maxpt,
                     const Eigen::Vector3f& translation,
                     const Eigen::Vector3f& orientation,
                     bool keep_organized) {
  pcl::CropBox<PCLPC2> filter;
  filter.setInputCloud(pc);
  filter.setMin(minpt);
  filter.setMax(maxpt);
  filter.setTranslation(translation);
  filter.setRotation(orientation);
  filter.setKeepOrganized(keep_organized);
  PCLPC2::Ptr pc_out(new PCLPC2());
  filter.filter(*pc_out);
  return pc_out;
}


PCLPC2::Ptr radius_outlier_removal(PCLPC2::Ptr pc,
                                   float radius,
                                   int min_pts) {
  pcl::RadiusOutlierRemoval<PCLPC2> filter;
  filter.setInputCloud(pc);
  filter.setRadiusSearch(radius);
  filter.setMinNeighborsInRadius(min_pts);
  //filter.setKeepOrganized(keep_organized);
  PCLPC2::Ptr pc_out(new PCLPC2());
  filter.filter(*pc_out);
  return pc_out;
}


void export_filters(py::module& m) {

#if 0
  // TODO to overload filter, we need to specify signature explicitly
  py::class_<pcl::PassThrough<PCLPC2>>(m, "PassThrough")
      .def("set_input_cloud", &pcl::PassThrough<PCLPC2>::setInputCloud)
      .def("set_filter_field_name", &pcl::PassThrough<PCLPC2>::setFilterFieldName)
      .def("set_filter_limits", &pcl::PassThrough<PCLPC2>::setFilterLimits)
      .def("set_keep_organized", &pcl::PassThrough<PCLPC2>::setKeepOrganized)
      .def("filter", &pcl::PassThrough<PCLPC2>::filter)
      ;

  py::class_<pcl::ExtractIndices<PCLPC2>>(m, "ExtractIndices")
      .def("set_input_cloud", &pcl::ExtractIndices<PCLPC2>::setInputCloud)
      .def("set_negative", &pcl::ExtractIndices<PCLPC2>::setNegative)
      ;
#endif

  // TODO
  // shadow point removal
  // sampling surface normal
  // radius outlier removal - see outlier_removal.cpp

  m.def("passthrough_filter",
        &passthrough_filter,
        py::arg("pc"),
        py::arg("field"),
        py::arg("minval"),
        py::arg("maxval"),
        py::arg("keep_organized")=true);

  m.def("extract_indices",
        &extract_indices,
        py::arg("pc"),
        py::arg("indices"),
        py::arg("set_negative")=false,
        py::arg("keep_organized")=true);

  m.def("voxel_grid_filter",
        &voxel_grid_filter,
        py::arg("pc"),
        py::arg("leaf_size"));

  m.def("statistical_outlier_removal",
        &statistical_outlier_removal,
        py::arg("pc"),
        py::arg("meank")=50,
        py::arg("stddev")=1.0,
        py::arg("keep_organized")=true);

  m.def("radius_outlier_removal",
        &radius_outlier_removal,
        py::arg("pc"),
        py::arg("radius")=1.,
        py::arg("min_pts")=20);

  m.def("crop_box",
        &crop_box,
        py::arg("pc"),
        py::arg("minpt"),
        py::arg("maxpt"),
        py::arg("translation")=Eigen::Vector3f::Zero(),
        py::arg("orientation")=Eigen::Vector3f::Zero(),
        py::arg("keep_organized")=true);

}

}
