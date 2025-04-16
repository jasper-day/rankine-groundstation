#include <drake/common/trajectories/piecewise_constant_curvature_trajectory.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "dubins_path.h"
#include "dubins_segment.h"
#include "dubins_solver.h"

namespace pympcc {

namespace py = pybind11;

using namespace mpcc::dubins;

PYBIND11_MODULE(pydubins, m) {
  m.doc() = "Python bindings for Dubins path planning";

  py::module::import("pydrake.trajectories");

  py::enum_<SegmentType>(m, "SegmentType")
      .value("SEGMENT", SegmentType::SEGMENT)
      .value("LINESEGMENT", SegmentType::LINESEGMENT)
      .value("CIRCULARSEGMENT", SegmentType::CIRCULARSEGMENT)
      .export_values();

  using T = double;
  // Bind the Segment base class
  py::class_<Segment<T>, std::shared_ptr<Segment<T>>>(m, "Segment")
      .def("path_coords", &Segment<T>::path_coords)
      .def("length", &Segment<T>::length)
      .def("start", &Segment<T>::start)
      .def("end", &Segment<T>::end)
      .def("heading_start", &Segment<T>::heading_start)
      .def("heading_end", &Segment<T>::heading_end)
      .def("eval", &Segment<T>::eval)
      .def("get_params", &Segment<T>::get_params)
      .def("set_params", &Segment<T>::set_params)
      .def_readonly("type", &Segment<T>::type);

  // Bind the LineSegment class
  py::class_<LineSegment<T>, Segment<T>, std::shared_ptr<LineSegment<T>>>(
      m, "LineSegment")
      .def(py::init([](const Eigen::Ref<const Eigen::Vector2d>& start,
                       const Eigen::Ref<const Eigen::Vector2d>& end) {
             return std::make_shared<LineSegment<T>>(drake::Vector2<T>(start),
                                                     drake::Vector2<T>(end));
           }),
           py::arg("start"), py::arg("end"))
      .def("path_coords",
           [](LineSegment<T>& self,
              const Eigen::Ref<const Eigen::Vector2d>& point) {
             return self.path_coords(point);
           })
      .def("length", &LineSegment<T>::length)
      .def("start", &LineSegment<T>::start)
      .def("end", &LineSegment<T>::end)
      .def("heading_start", &LineSegment<T>::heading_start)
      .def("heading_end", &LineSegment<T>::heading_end)
      .def("eval", &LineSegment<T>::eval)
      .def("get_params", &LineSegment<T>::get_params)
      .def("set_params", &LineSegment<T>::set_params)
      .def_readonly("type", &LineSegment<T>::type);

  // Bind the CircularSegment class
  py::class_<CircularSegment<T>, Segment<T>,
             std::shared_ptr<CircularSegment<T>>>(m, "CircularSegment")
      .def(py::init([](const Eigen::Ref<const Eigen::Vector2d>& centre,
                       T radius, T heading, T arclength) {
             return std::make_shared<CircularSegment<T>>(
                 drake::Vector2<T>(centre), radius, heading, arclength);
           }),
           py::arg("centre"), py::arg("radius"), py::arg("heading"),
           py::arg("arclength"))
      .def("path_coords",
           [](CircularSegment<T>& self,
              const Eigen::Ref<const Eigen::Vector2d>& point) {
             return self.path_coords(point);
           })
      .def("length", &CircularSegment<T>::length)
      .def("start", &CircularSegment<T>::start)
      .def("end", &CircularSegment<T>::end)
      .def("heading_start", &CircularSegment<T>::heading_start)
      .def("heading_end", &CircularSegment<T>::heading_end)
      .def("eval", &CircularSegment<T>::eval)
      .def("get_params", &CircularSegment<T>::get_params)
      .def("set_params", &CircularSegment<T>::set_params)
      .def_readonly("type", &CircularSegment<T>::type);

  // Bind the DubinsPath class
  py::class_<DubinsPath<T>>(m, "DubinsPath")
      .def(py::init<>())
      .def(py::init<std::vector<std::shared_ptr<Segment<T>>>>(),
           py::arg("segments"))
      .def("add_segment",
           [](DubinsPath<T>& self, const std::shared_ptr<Segment<T>>& segment) {
             if (auto line =
                     dynamic_cast<const LineSegment<T>*>(segment.get())) {
               self.add_segment(std::make_unique<LineSegment<T>>(*line));
             } else if (auto circle = dynamic_cast<const CircularSegment<T>*>(
                            segment.get())) {
               self.add_segment(std::make_unique<CircularSegment<T>>(*circle));
             } else {
               throw std::runtime_error("Unknown segment type");
             }
           })
      .def("num_segments", &DubinsPath<T>::num_segments)
      .def("get_segment", &DubinsPath<T>::get_segment)
      .def("lengths", &DubinsPath<T>::lengths)
      .def("length", &DubinsPath<T>::length)
      .def("eval", &DubinsPath<T>::eval)
      .def("start", &DubinsPath<T>::start)
      .def("end", &DubinsPath<T>::end)
      .def("get_params", &DubinsPath<T>::get_params)
      // We need to use an Eigen::Ref for pybind11 to work with numpy arrays,
      // otherwise SIGSEV
      .def(
          "set_params",
          [](DubinsPath<T>& self,
             const Eigen::Ref<const Eigen::VectorXd>& params) {
            self.set_params(drake::VectorX<T>(params));
          },
          py::arg("params"))
      .def("get_constraint_residuals",
           [](DubinsPath<T>& self,
              const Eigen::Ref<const Eigen::VectorXd>& values) {
             return self.get_constraint_residuals(drake::VectorX<T>{values});
           })
      .def("n_params", &DubinsPath<T>::n_params)
      .def("n_constraints", &DubinsPath<T>::n_constraints)
      .def("get_types", &DubinsPath<T>::get_types)
      .def("get_closest_arclength",
           [](DubinsPath<T>& self, const Eigen::Ref<const Eigen::Vector2d>& pos,
              T const& estimated_arclength) {
             return self.get_closest_arclength(pos, estimated_arclength);
           })
      .def("get_true_arclength",
           [](DubinsPath<T>& self,
              const Eigen::Ref<const Eigen::Vector2d>& pos) {
             return self.get_true_arclength(pos);
           });
  // don't bind `to_drake` -- missing pydrake bindings.

  py::class_<SolverResult>(m, "SolverResult")
      .def_readonly("params", &SolverResult::params)
      .def_readonly("converged", &SolverResult::converged);

  py::class_<DubinsSolver>(m, "DubinsSolver")
      .def(py::init<>())
      .def(py::init<double, int, int, bool>(), py::arg("tolerance"),
           py::arg("max_iter"), py::arg("debug"), py::arg("line_search"))
      .def("solve", &DubinsSolver::solve)
      .def("jac", [](DubinsSolver& self, DubinsPath<T>& path,
                     Eigen::Ref<const Eigen::VectorXd> const& params) {
        return self.jac(path, drake::VectorX<T>{params});
      });
}

}  // namespace pympcc