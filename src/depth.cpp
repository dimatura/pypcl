#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include "tinyformat.h"
#include "typedefs.h"

namespace pypcl {

namespace py = pybind11;

PCXYZ::Ptr depth_img_to_pcxyz(const ndarray2f& depth_img,
                              float fx,
                              float fy,
                              float cx,
                              float cy,
                              float min_valid_depth,
                              float max_valid_depth,
                              bool organized = true) {
  size_t height = depth_img.shape(0);
  size_t width = depth_img.shape(1);
  PCXYZ::Ptr xyz_pc(new PCXYZ());

  auto depth_img_buf = depth_img.unchecked();

  if (organized) {
    for (int v = 0; v < height; ++v) {
      for (int u = 0; u < width; ++u) {
        pcl::PointXYZ p;
        float d = depth_img_buf(v, u);
        if (!std::isfinite(d) || (d < min_valid_depth) ||
            (d > max_valid_depth)) {
          p.x = std::numeric_limits<float>::quiet_NaN();
          p.y = std::numeric_limits<float>::quiet_NaN();
          p.z = std::numeric_limits<float>::quiet_NaN();
        } else {
          p.x = (u - cx) * d / fx;
          p.y = (v - cy) * d / fy;
          p.z = d;
        }
        xyz_pc->push_back(p);
      }
    }
    xyz_pc->width = width;
    xyz_pc->height = height;
  } else {
    for (int v = 0; v < height; ++v) {
      for (int u = 0; u < width; ++u) {
        float d = depth_img_buf(v, u);
        if (!std::isfinite(d) || (d < min_valid_depth) ||
            (d > max_valid_depth)) {
          continue;
        }
        pcl::PointXYZ p;
        p.x = (u - cx) * d / fx;
        p.y = (v - cy) * d / fy;
        p.z = d;
        xyz_pc->push_back(p);
      }
    }
    xyz_pc->width = xyz_pc->size();
    xyz_pc->height = 1;
  }
  return xyz_pc;
}

ndarray2f disp_u2_img_to_depth_img(const ndarray2u2& disp_img,
                                   int subpx_factor,
                                   float Tx,
                                   bool mask_invalid,
                                   uint16_t invalid_value) {
  size_t height = disp_img.shape(0);
  size_t width = disp_img.shape(1);
  ndarray2f depth_img({height, width});
  auto disp_img_buf = disp_img.unchecked();
  auto depth_img_buf = depth_img.mutable_unchecked();
  float constant = -Tx * subpx_factor;
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      uint16_t disp = disp_img_buf(v, u);
      if (mask_invalid && (disp == invalid_value)) {
        depth_img_buf(v, u) = std::numeric_limits<float>::quiet_NaN();
      } else {
        depth_img_buf(v, u) = constant / disp;
      }
    }
  }
  return depth_img;
}

ndarray2f disp_f4_img_to_depth_img(const ndarray2f& disp_img,
                                   int subpx_factor,
                                   float Tx) {
  size_t height = disp_img.shape(0);
  size_t width = disp_img.shape(1);
  ndarray2f depth_img({height, width});
  auto disp_img_buf = disp_img.unchecked();
  auto depth_img_buf = depth_img.mutable_unchecked();
  float constant = -Tx * subpx_factor;
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      float disp = disp_img_buf(v, u);
      if (!std::isfinite(disp)) {
        depth_img_buf(v, u) = std::numeric_limits<float>::quiet_NaN();
      } else {
        depth_img_buf(v, u) = constant / disp;
      }
    }
  }
  return depth_img;
}

void export_depth(py::module& m) {
  m.def("depth_img_to_pcxyz",
        &depth_img_to_pcxyz,
        py::arg("depth_img"),
        py::arg("fx"),
        py::arg("fy"),
        py::arg("cx"),
        py::arg("cy"),
        py::arg("min_valid_depth"),
        py::arg("max_valid_depth"),
        py::arg("organized") = false);

  m.def("disp_u2_img_to_depth_img",
        &disp_u2_img_to_depth_img,
        py::arg("disp_img"),
        py::arg("subpx_factor"),
        py::arg("Tx"),
        py::arg("mask_invalid"),
        py::arg("invalid_value"));

  m.def("disp_f4_img_to_depth_img",
        &disp_f4_img_to_depth_img,
        py::arg("disp_img"),
        py::arg("subpx_factor"),
        py::arg("Tx"));
}
}
