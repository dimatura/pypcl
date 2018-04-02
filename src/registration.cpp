#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <pcl/common/common.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_organized_projection.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>

//#include <pcl/registration/correspondence_rejection_median_distance.h>
//#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>

#include "typedefs.h"

namespace pypcl {

namespace py = pybind11;


Eigen::Matrix4f icp(const PCLPC2::Ptr input_pc,
                    const PCLPC2::Ptr target_pc,
                    int iter,
                    float max_dist,
                    float outlier_thresh,
                    const Eigen::Matrix4f& guess) {
  PCXYZ::Ptr xyz_src(new PCXYZ());
  PCXYZ::Ptr xyz_tgt(new PCXYZ());
  pcl::fromPCLPointCloud2(*input_pc, *xyz_src);
  pcl::fromPCLPointCloud2(*target_pc,*xyz_tgt);

  using ICP = pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;
  ICP icp;
  icp.setMaximumIterations(iter);
  icp.setMaxCorrespondenceDistance(max_dist);
  icp.setRANSACOutlierRejectionThreshold(outlier_thresh);
  icp.setInputSource(xyz_src);
  icp.setInputTarget(xyz_tgt);
  PCXYZ::Ptr out_pc(new PCXYZ());
  // align(*out_pc, Matrix4f guess)
  icp.align(*out_pc, guess);
  Eigen::Matrix4f M = icp.getFinalTransformation();
  //py::object out_pc_o = py::cast(out_pc);
  //return py::make_tuple(out_pc_o, M);
  return M;
}

Eigen::Matrix4f organized_pc_registration(PCXYZ::Ptr xyz_src,
                                          PCXYZ::Ptr xyz_tgt,
                                          float fx,
                                          float fy,
                                          float threshold,
                                          float max_dist) {
  // I don't even know.
  pcl::registration::CorrespondenceEstimationOrganizedProjection<pcl::PointXYZ, pcl::PointXYZ> organized_projection;
  organized_projection.setInputSource(xyz_src);
  organized_projection.setInputTarget(xyz_tgt);
  organized_projection.setFocalLengths(fx, fy);
  organized_projection.setDepthThreshold(threshold); // 0.03
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
  organized_projection.determineCorrespondences(*correspondences, max_dist); //0.05
  pcl::registration::CorrespondenceRejectorDistance rejector_d;
  //pcl::PointCloud<pcl::PointXYZ>::ConstPtr cxyz_src(xyz_src);
  //pcl::PointCloud<pcl::PointXYZ>::ConstPtr cxyz_tgt(xyz_tgt);
  rejector_d.setInputSource<pcl::PointXYZ>(xyz_src);
  rejector_d.setInputTarget<pcl::PointXYZ>(xyz_tgt);
  rejector_d.setInputCorrespondences(correspondences);
  rejector_d.setMaximumDistance(0.5);
  pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences);
  rejector_d.getCorrespondences(*correspondences_filtered);
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> te;
  Eigen::Matrix4f tf;
  te.estimateRigidTransformation(*xyz_src, *xyz_tgt, *correspondences_filtered, tf);
  //pcl::registration::TransformationEstimationPointToPlaneWeighted<pcl::PointXYZ, pcl::PointNormal, double> te;
  //te.setWeights(correspondence_weights);
  //te.estimateRigidTransformation(*xyz_src, *cloud_normals_tgt, correspondences_filtered, transform);
  return tf;
}

Eigen::Matrix4f icp2(const PCLPC2::Ptr pc2_src,
                     const PCLPC2::Ptr pc2_tgt,
                     float inlier_threshold,
                     int sac_max_iter,
                     int icp_max_iter,
                     float icp_eps,
                     const Eigen::Matrix4f& guess) {
  // assumes clouds have normals
  // not even sure if they are used though
  using namespace pcl;
  using namespace pcl::registration;
  using Scalar = float;
  PointCloud<PointNormal>::Ptr src(new PointCloud<PointNormal>);
  PointCloud<PointNormal>::Ptr tgt(new PointCloud<PointNormal>);
  fromPCLPointCloud2(*pc2_src, *src);
  fromPCLPointCloud2(*pc2_tgt, *tgt);
  TransformationEstimationLM<PointNormal, PointNormal, Scalar>::Ptr te(new TransformationEstimationLM<PointNormal, PointNormal, Scalar>);
  CorrespondenceEstimation<PointNormal, PointNormal, Scalar>::Ptr cens(new CorrespondenceEstimation<PointNormal, PointNormal, Scalar>);
  cens->setInputSource(src);
  cens->setInputTarget(tgt);
  //cens->setSourceNormals (src);
  CorrespondenceRejectorOneToOne::Ptr cor_rej_o2o(new CorrespondenceRejectorOneToOne);
  CorrespondenceRejectorMedianDistance::Ptr cor_rej_med(new CorrespondenceRejectorMedianDistance);
  cor_rej_med->setInputSource<PointNormal>(src);
  cor_rej_med->setInputTarget<PointNormal>(tgt);
  CorrespondenceRejectorSampleConsensus<PointNormal>::Ptr cor_rej_sac(new CorrespondenceRejectorSampleConsensus<PointNormal>);
  cor_rej_sac->setInputSource(src);
  cor_rej_sac->setInputTarget(tgt);
  cor_rej_sac->setInlierThreshold(inlier_threshold); // 0.005
  cor_rej_sac->setMaximumIterations(sac_max_iter); // 100000
  CorrespondenceRejectorVarTrimmed::Ptr cor_rej_var(new CorrespondenceRejectorVarTrimmed);
  IterativeClosestPoint<PointNormal, PointNormal, Scalar> icp;
  icp.setCorrespondenceEstimation(cens);
  icp.setTransformationEstimation(te);
  icp.addCorrespondenceRejector(cor_rej_o2o);
  //icp.addCorrespondenceRejector(cor_rej_var);
  //icp.addCorrespondenceRejector(cor_rej_med);
  //icp.addCorrespondenceRejector(cor_rej_tri);
  //icp.addCorrespondenceRejector(cor_rej_sac);
  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  icp.setMaximumIterations(icp_max_iter); // 100
  icp.setTransformationEpsilon(icp_eps); // 1e-10
  PointCloud<PointNormal> output;
  icp.align(output, guess);
  Eigen::Matrix4f transformation = icp.getFinalTransformation();
  return transformation;
  // Convert data back
  //pcl::PCLPointCloud2 output_source;
  //toPCLPointCloud2(output, output_source);
  //concatenateFields(*xyz_src, output_source, transformed_source);
}


void export_registration(py::module& m) {
  m.def("icp",
        &icp,
        py::arg("input_pc"),
        py::arg("target_pc"),
        py::arg("iter") = 100,
        py::arg("max_dist") = 0.1f,
        py::arg("outlier_thresh") = 0.1f,
        py::arg("guess")=Eigen::Matrix4f::Identity());

  m.def("organized_pc_registration",
        &organized_pc_registration,
        py::arg("xyz_src"),
        py::arg("xyz_tgt"),
        py::arg("fx"),
        py::arg("fy"),
        py::arg("threshold") = 1.0f,
        py::arg("max_dist") = 2.0f);

  m.def("icp2",
        &icp2,
        py::arg("pc2_src"),
        py::arg("pc2_tgt"),
        py::arg("inlier_threshold")=0.1,
        py::arg("sac_max_iter")=100,
        py::arg("icp_max_iter")=100,
        py::arg("icp_eps")=1e-3,
        py::arg("guess")=Eigen::Matrix4f::Identity());

}

}
