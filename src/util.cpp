#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "point_cloud2_iterator.h"
#include "typedefs.h"
#include "util.h"

namespace pypcl {
namespace py = pybind11;

void get_valid_xyz_indices(const PCLPC2& pc, std::vector<int>& valid_ix) {
  PointCloud2ConstIterator<float> iter_x(pc, "x");
  valid_ix.clear();
  valid_ix.reserve(pc.width * pc.height);

  int ix = 0;
  while (iter_x != iter_x.end()) {
    if (!std::isfinite(iter_x[0]) || !std::isfinite(iter_x[1]) ||
        !std::isfinite(iter_x[2])) {
      // nothing
    } else {
      valid_ix.push_back(ix);
    }
    ++iter_x;
    ++ix;
  }
}

void export_util(py::module& m) {
  // m.def("demean_pcxyz", pcl::demeanPointCloud(
  m.def("get_valid_xyz_indices",
        [](PCLPC2::Ptr pc) {
          std::vector<int> indices;
          get_valid_xyz_indices(*pc, indices);
          return indices;
        },
        py::arg("pc"));
}
}
