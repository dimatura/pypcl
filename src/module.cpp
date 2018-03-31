#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace pypcl {
void export_pointcloud(py::module& m);
void export_depth(py::module& m);
void export_features(py::module& m);
void export_filters(py::module& m);
void export_io(py::module& m);
void export_transform(py::module& m);
#if 0
void export_misc(py::module& m);
void export_pinhole_camera_model(py::module& m);
void export_point_projector(py::module& m);
void export_stereo_camera_model(py::module& m);
void export_icp(py::module& m);
//void exportPcRasterizer();
//void exportTriangulation();
//void exportDistortion();
#endif
}

PYBIND11_MODULE(libpypcl, m) {
  m.doc() = "pypcl";
  pypcl::export_pointcloud(m);
  pypcl::export_depth(m);
  pypcl::export_features(m);
  pypcl::export_filters(m);
  pypcl::export_io(m);
  pypcl::export_transform(m);
#if 0
  pypc::export_misc(m);
  pypc::export_pinhole_camera_model(m);
  pypc::export_point_projector(m);
  pypc::export_stereo_camera_model(m);
  pypc::export_icp(m);
  //pypc::exportPcRasterizer(m);
  //pypc::exportTriangulation(m);
  //pypc::exportDistortion(m);
#endif
}
