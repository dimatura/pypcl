#include <cstdint>
#include <cmath>
#include <limits>

#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <boost/shared_ptr.hpp>

#include "typedefs.h"
#include "tinyformat.h"

namespace pypcl {

py::array pclpc2_as_ndarray(const PCLPC2& pc) {
  py::list names, formats, offsets;

  ssize_t itemsize = 0;
  for (const auto& field : pc.fields) {
    names.append(field.name);
    switch (field.datatype) {
      case pcl::PCLPointField::UINT8:
        formats.append("u1");
        itemsize += 1;
        break;
      case pcl::PCLPointField::UINT16:
        formats.append("u2");
        itemsize += 2;
        break;
      case pcl::PCLPointField::INT32:
        formats.append("i4");
        itemsize += 4;
        break;
      case pcl::PCLPointField::UINT32:
        formats.append("u4");
        itemsize += 4;
        break;
      case pcl::PCLPointField::FLOAT32:
        formats.append("f4");
        itemsize += 4;
        break;
      case pcl::PCLPointField::FLOAT64:
        formats.append("f8");
        itemsize += 8;
        break;
      default:
        throw std::runtime_error("unsupported data type");
        break;
    }
    offsets.append(field.offset);
  }
  // TODO not quite the same due to padding -
  // however, the 'correct' itemsize seems to give wrong result
  //ssize_t itemsize = pc.point_step;
  py::dtype dt(names, formats, offsets, itemsize);

  std::vector<ssize_t> shape, strides;
  // two cases - structured vs unstructured
  if (pc.height == 1) {
    shape.push_back(pc.width);
    strides.push_back(pc.point_step);
  } else {
    shape.push_back(pc.height);
    shape.push_back(pc.width);
    strides.push_back(pc.row_step);
    strides.push_back(pc.point_step);
  }
  // TODO figure out object/handle for refcounting
  py::object obj(py::cast(pc));
  py::array arr(dt, shape, strides, &pc.data[0], obj);
  return arr;
}

PCLPC2::Ptr _pclpc2_from_ndarray(const py::array& arr,
                                const py::list& fields) {
  PCLPC2::Ptr pc(new PCLPC2);
  int n = py::len(fields);
  for (int i=0; i < n; ++i) {
    pcl::PCLPointField pf;
    py::tuple name_fmt_offset = fields[i].cast<py::tuple>();
    pf.name = name_fmt_offset[0].cast<std::string>();
    pf.offset = name_fmt_offset[2].cast<int>();
    // TODO assumption
    pf.count = 1;
    std::string fmt = name_fmt_offset[1].attr("name").cast<std::string>();
    //py::print(fmt);
    if (fmt == "float32") {
      pf.datatype = pcl::PCLPointField::FLOAT32;
    } else if (fmt == "float64") {
      pf.datatype = pcl::PCLPointField::FLOAT64;
    } else if (fmt == "uint8") {
      pf.datatype = pcl::PCLPointField::UINT8;
    } else if (fmt == "uint16") {
      pf.datatype = pcl::PCLPointField::UINT16;
    } else if (fmt == "int32") {
      pf.datatype = pcl::PCLPointField::INT32;
    } else {
      throw std::runtime_error("unsupported data type");
    }
    pc->fields.push_back(pf);
  }
  pc->point_step = arr.itemsize();
  if (arr.ndim() == 1) {
    pc->row_step = static_cast<uint32_t>(arr.itemsize()*arr.shape(0));
    pc->width = arr.shape(0);
    pc->height = 1;
  } else {
    pc->row_step = static_cast<uint32_t>(arr.strides(0));
    pc->width = arr.shape(0);
    pc->height = arr.shape(1);
  }
  return pc;
}


template<typename PointT>
struct FieldAdder {
  FieldAdder(py::list& names,
             py::list& formats,
             py::list& offsets,
             std::vector<int>& sizes) :
      names_(names),
      formats_(formats),
      offsets_(offsets),
      sizes_(sizes) { }

  template<typename U> void operator() () {
    names_.append(pcl::traits::name<PointT, U>::value);
    offsets_.append(pcl::traits::offset<PointT, U>::value);
    int datatype = pcl::traits::datatype<PointT, U>::value;
    switch (datatype) {
      case pcl::PCLPointField::UINT8:
        formats_.append("u1");
        sizes_.push_back(1);
        break;
      case pcl::PCLPointField::UINT16:
        formats_.append("u2");
        sizes_.push_back(2);
        break;
      case pcl::PCLPointField::INT32:
        formats_.append("i4");
        sizes_.push_back(4);
        break;
      case pcl::PCLPointField::UINT32:
        formats_.append("u4");
        sizes_.push_back(4);
        break;
      case pcl::PCLPointField::FLOAT32:
        formats_.append("f4");
        sizes_.push_back(4);
        break;
      case pcl::PCLPointField::FLOAT64:
        formats_.append("f8");
        sizes_.push_back(8);
        break;
      default:
        throw std::runtime_error("unsupported data type");
        break;
    }
  }
  py::list& names_, formats_, offsets_;
  std::vector<int>& sizes_;
};

template <class PointT>
py::array pointcloud_to_ndarray(pcl::PointCloud<PointT>& pc) {
  //using PointT = PointCloudT::PointType;

  py::list names, formats, offsets;
  std::vector<int> sizes;
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(FieldAdder<PointT>(names,
                                                                                       formats,
                                                                                       offsets,
                                                                                       sizes));
  int itemsize = 0;
  for (int it : sizes) {
    itemsize += it;
  }

  py::dtype dt(names,
               formats,
               offsets,
               itemsize);

  std::vector<ssize_t> shape, strides;
  // two cases - structured vs unstructured
  if (pc.height == 1) {
    shape.push_back(pc.width);
    strides.push_back(sizeof(PointT));
  } else {
    shape.push_back(pc.height);
    shape.push_back(pc.width);
    strides.push_back(static_cast<uint32_t>(sizeof(PointT) * pc.width));
    strides.push_back(sizeof(PointT));
  }
  // TODO figure out object/handle for refcounting
  py::object obj(py::cast(pc));
  py::array arr(dt,
                shape,
                strides,
                reinterpret_cast<uint8_t*>(&pc.points[0]),
                obj);
  return arr;

}

template <class PointCloudT>
void export_pointcloud(py::module& m, const char* name) {
  py::class_<PointCloudT, typename PointCloudT::Ptr>(m, name)
      .def(py::init<>())
      .def(py::init<PointCloudT>())
      .def_readwrite("width", &PointCloudT::width)
      .def_readwrite("height", &PointCloudT::height)
      .def_readwrite("is_dense", &PointCloudT::is_dense)
      .def_readwrite("sensor_origin", &PointCloudT::sensor_origin_)
      .def_readwrite("sensor_orientation", &PointCloudT::sensor_orientation_)
      .def("size", &PointCloudT::size)
      .def("clear", &PointCloudT::clear)
      .def("is_organized", &PointCloudT::isOrganized)
      .def("get_MatrixXfMap", [](const PointCloudT& self) { return self.getMatrixXfMap(); })
      .def("to_pclpc2", [](const PointCloudT& self) {
        pcl::PCLPointCloud2::Ptr out(new PCLPC2);
        pcl::toPCLPointCloud2(self, *out);
        return out;
      })
      .def("to_ndarray", [](PointCloudT& self) {
        return pointcloud_to_ndarray<typename PointCloudT::PointType>(self);
      })
      .def("empty", &PointCloudT::empty)
      .def("cat", [](PointCloudT& self, const PointCloudT& other) {
           return self + other;
      })
      .def("swap", &PointCloudT::swap)
      .def("__repr__", [](const PointCloudT& self) {
           std::stringstream ss;
           ss << "PointCloudT: " << "\n";
           ss << self;
           return ss.str();
      })
      .def("copy", [](const PointCloudT& self) {
           return PointCloudT(self);
      })
      //.def("set_from_ndarray", &set_pc_from_ndarray<PointCloudT>)
      //.def_buffer(&pcxyz_to_buffer_info);
      ;
}


void export_pclpointcloud2(py::module& m) {
  py::class_<pcl::PCLPointField> PCLPointField(m, "PCLPointField");
  PCLPointField.def(py::init<>())
      .def_readwrite("name", &pcl::PCLPointField::name)
      .def_readwrite("offset", &pcl::PCLPointField::offset)
      .def_readwrite("datatype", &pcl::PCLPointField::datatype)
      .def_readwrite("count", &pcl::PCLPointField::count)
      ;

  py::enum_<pcl::PCLPointField::PointFieldTypes>(PCLPointField, "PointFieldTypes")
      .value("INT8", pcl::PCLPointField::INT8)
      .value("UINT8", pcl::PCLPointField::UINT8)
      .value("INT16", pcl::PCLPointField::INT16)
      .value("UINT16", pcl::PCLPointField::UINT16)
      .value("INT32", pcl::PCLPointField::INT32)
      .value("UINT32", pcl::PCLPointField::UINT32)
      .value("FLOAT32", pcl::PCLPointField::FLOAT32)
      .value("FLOAT64", pcl::PCLPointField::FLOAT64)
      .export_values()
      ;

  py::class_<PCLPC2, PCLPC2::Ptr>(m, "PCLPointCloud2")
      .def(py::init<>())
      .def_readwrite("width", &PCLPC2::width)
      .def_readwrite("height", &PCLPC2::height)
      .def_readwrite("fields", &PCLPC2::fields)
      .def_readwrite("is_bigendian", &PCLPC2::is_bigendian)
      .def_readwrite("point_step", &PCLPC2::point_step)
      .def_readwrite("row_step", &PCLPC2::row_step)
      .def_readwrite("is_dense", &PCLPC2::is_dense)
      .def("to_ndarray", &pclpc2_as_ndarray)
      .def_static("from_ndarray", &_pclpc2_from_ndarray)
	    .def("info", [](const PCLPC2& pc) {
        std::stringstream s;
        s << "PCLPointCloud2:" << std::endl;
        s << "height: ";
        s << "  " << pc.height << std::endl;
        s << "width: ";
        s << "  " << pc.width << std::endl;
        s << "point_step: ";
        s << "  " << pc.point_step << std::endl;
        s << "row_step: ";
        s << "  " << pc.row_step << std::endl;
        s << "is_dense: ";
        s << "  " << pc.is_dense << std::endl;
        s << "fields[]" << std::endl;
        for (size_t i = 0; i < pc.fields.size (); ++i) {
          s << "  fields[" << i << "]: " << std::endl;
          s << "    name: " << pc.fields[i].name << std::endl;
          s << "    offset: " << pc.fields[i].offset << std::endl;
          s << "    datatype: " << int(pc.fields[i].datatype) << std::endl;
          s << "    count: " << pc.fields[i].count << std::endl;
        }
        return s.str();})
     ;
}

void export_quaternion(py::module& m) {
  using Quat = Eigen::Quaternionf;

  py::class_<Quat>(m, "Quaternionf")
      .def(py::init<float, float, float, float>(),
           py::arg("w")=1.0, py::arg("x")=0., py::arg("y")=0., py::arg("z")=0.)
      .def(py::init<Quat>())
      .def_property("w",
                    [](const Quat& q) { return q.w(); },
                    [](Quat& q, float w) { q.w() = w; })
      .def_property("x",
                    [](const Quat& q) { return q.x(); },
                    [](Quat& q, float x) { q.x() = x; })
      .def_property("y",
                    [](const Quat& q) { return q.y(); },
                    [](Quat& q, float y) { q.y() = y; })
      .def_property("z",
                    [](const Quat& q) { return q.z(); },
                    [](Quat& q, float z) { q.z() = z; })
      .def("as_tuple_wxyz",
           [](const Quat& q) { return py::make_tuple(q.w(), q.x(), q.y(), q.z()); })
      .def("as_tuple_xyzw",
           [](const Quat& q) { return py::make_tuple(q.x(), q.y(), q.z(), q.w()); })
      .def("normalize",
           [](Quat& q) { q.normalize(); })
      .def("inverse",
           [](const Quat& q) { return q.inverse(); })
      .def("norm",
           [](const Quat& q) { return q.norm(); })
      .def("set_identity",
           [](Quat& q) { q.setIdentity(); })
      .def("to_rotation_matrix",
           [](const Quat& q) { return q.toRotationMatrix(); })
      .def("coeffs",
           [](const Quat& q) { return q.coeffs(); })
      .def("angular_distance",
           [](const Quat& self, const Quat& other) { return self.angularDistance(other); })
      .def("__repr__",
           [](const Quat& q) {
            return tfm::format("Quaternionf(w=%.4f, x=%.4f, y=%.4f, z=%.4f)",
                               q.w(), q.x(), q.y(), q.z());})
      .def_static("unit_random",
           []() { return Eigen::Quaternionf::UnitRandom(); })
      .def_static("from_angle_axis",
           [](float angle, const Eigen::Vector3f& axis) { return Quat(Eigen::AngleAxisf(angle, axis)); })
      ;

}


PCXYZ::Ptr xyz_img_to_pc(const ndarray2f& xyz_img,
                         bool skip_nan) {
  size_t height = xyz_img.shape(0);
  size_t width = xyz_img.shape(1);
  PCXYZ::Ptr xyz_pc(new PCXYZ());
  auto xyz_img_buf = xyz_img.unchecked();
  for (int v=0; v < height; ++v) {
    for (int u=0; u < width; ++u) {
      float x = xyz_img_buf(v, u, 0);
      float y = xyz_img_buf(v, u, 1);
      float z = xyz_img_buf(v, u, 2);
      if (skip_nan && (std::isnan(x)||std::isnan(y)||std::isnan(z))) {
        continue;
      }
      pcl::PointXYZ p; p.x = x; p.y = y; p.z = z;
      xyz_pc->push_back(p);
    }
  }
  xyz_pc->width = xyz_pc->size();
  xyz_pc->height = 1;
  return xyz_pc;
}

PCXYZ::Ptr xyz_img_to_organized_pc(const ndarray2f& xyz_img) {
  size_t height = xyz_img.shape(0);
  size_t width = xyz_img.shape(1);
  PCXYZ::Ptr xyz_pc(new PCXYZ());
  auto xyz_img_buf = xyz_img.unchecked();
  for (int v=0; v < height; ++v) {
    for (int u=0; u < width; ++u) {
      pcl::PointXYZ p;
      p.x = xyz_img_buf(v, u, 0);
      p.y = xyz_img_buf(v, u, 1);
      p.z = xyz_img_buf(v, u, 2);
      xyz_pc->push_back(p);
    }
  }
  xyz_pc->width = width;
  xyz_pc->height = height;
  return xyz_pc;
}


PCXYZ::Ptr ndarray_to_pcxyz(ndarrayXf arr) {
  PCXYZ::Ptr pc(new PCXYZ());
  if (arr.ndim() == 2) {
    // assuming unorganized
    if (arr.shape(1) != 3) {
      throw py::value_error("unorganized arr must be Nx3");
    }
    auto arr_buf = arr.unchecked<2>();
    for (int i=0; i < arr.shape(0); ++i) {
      pcl::PointXYZ p;
      p.x = arr_buf(i, 0);
      p.y = arr_buf(i, 1);
      p.z = arr_buf(i, 2);
      pc->push_back(p);
    }
    pc->width = arr.shape(0);
    pc->height = 1;
  } else if (arr.ndim() == 3) {
    // assuming organized, ie xyz image
    if (arr.shape(2) != 3) {
      throw py::value_error("organized pc array must be HxWx3");
    }
    auto arr_buf = arr.unchecked<3>();
    for (int i=0; i < arr.shape(0); ++i) {
      for (int j=0; j < arr.shape(1); ++j) {
        pcl::PointXYZ p;
        p.x = arr_buf(i, j, 0);
        p.y = arr_buf(i, j, 1);
        p.z = arr_buf(i, j, 2);
        pc->push_back(p);
      }
    }
    pc->width = arr.shape(1);
    pc->height = arr.shape(0);
  }
}


void export_pointcloud(py::module& m) {
  export_quaternion(m);
  export_pointcloud<pcl::PointCloud<pcl::PointXYZ>>(m, "PointCloudXYZ");
  export_pclpointcloud2(m);

  m.def("xyz_img_to_pc",
        &xyz_img_to_pc,
        py::arg("xyz_img"),
        py::arg("skip_nan"));

  m.def("xyz_img_to_organized_pc",
        &xyz_img_to_organized_pc,
        py::arg("xyz_img"));

  m.def("ndarray_to_pcxyz",
        &ndarray_to_pcxyz,
        py::arg("arr"));

}

}
