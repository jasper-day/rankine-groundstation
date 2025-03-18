#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "dubins.h"

namespace pympcc {

namespace py = pybind11;

PYBIND11_MODULE(pydubins, m) {
  m.doc() = "Python bindings for Dubins path planning";
  using T = double;
  // Bind the Segment base class
  py::class_<mpcc::dubins::Segment<T>,
             std::shared_ptr<mpcc::dubins::Segment<T>>>(m, "Segment")
      .def("path_coords", &mpcc::dubins::Segment<T>::path_coords)
      .def("length", &mpcc::dubins::Segment<T>::length)
      .def("start", &mpcc::dubins::Segment<T>::start)
      .def("end", &mpcc::dubins::Segment<T>::end)
      .def("heading_start", &mpcc::dubins::Segment<T>::heading_start)
      .def("heading_end", &mpcc::dubins::Segment<T>::heading_end)
      .def("eval", &mpcc::dubins::Segment<T>::eval);

  // Bind the LineSegment class
  py::class_<mpcc::dubins::LineSegment<T>, mpcc::dubins::Segment<T>,
             std::shared_ptr<mpcc::dubins::LineSegment<T>>>(m, "LineSegment")
      //   .def(py::init<>()) // no need for empty constructor
      .def(py::init<const drake::Vector2<T>&, const drake::Vector2<T>&>(),
           py::arg("start"), py::arg("end"))
      .def("path_coords", &mpcc::dubins::LineSegment<T>::path_coords)
      .def("length", &mpcc::dubins::LineSegment<T>::length)
      .def("start", &mpcc::dubins::LineSegment<T>::start)
      .def("end", &mpcc::dubins::LineSegment<T>::end)
      .def("heading_start", &mpcc::dubins::LineSegment<T>::heading_start)
      .def("heading_end", &mpcc::dubins::LineSegment<T>::heading_end)
      .def("eval", &mpcc::dubins::LineSegment<T>::eval);

  // Bind the CircularSegment class
  py::class_<mpcc::dubins::CircularSegment<T>, mpcc::dubins::Segment<T>,
             std::shared_ptr<mpcc::dubins::CircularSegment<T>>>(
      m, "CircularSegment")
      //   .def(py::init<>())
      .def(py::init<const drake::Vector2<T>&, T, T, T, T>(), py::arg("center"),
           py::arg("radius"), py::arg("dir"), py::arg("heading"),
           py::arg("arclength"))
      .def("path_coords", &mpcc::dubins::CircularSegment<T>::path_coords)
      .def("length", &mpcc::dubins::CircularSegment<T>::length)
      .def("start", &mpcc::dubins::CircularSegment<T>::start)
      .def("end", &mpcc::dubins::CircularSegment<T>::end)
      .def("heading_start", &mpcc::dubins::CircularSegment<T>::heading_start)
      .def("heading_end", &mpcc::dubins::CircularSegment<T>::heading_end)
      .def("eval", &mpcc::dubins::CircularSegment<T>::eval);

  // Bind the DubinsPath class
  py::class_<mpcc::dubins::DubinsPath<T>>(m, "DubinsPath")
      .def(py::init<>())
      .def(py::init<std::vector<std::shared_ptr<mpcc::dubins::Segment<T>>>>(),
           py::arg("segments"))
      .def("add_segment",
           [](mpcc::dubins::DubinsPath<T>& self,
              const std::shared_ptr<mpcc::dubins::Segment<T>>& segment) {
             if (auto line = dynamic_cast<const mpcc::dubins::LineSegment<T>*>(
                     segment.get())) {
               self.add_segment(
                   std::make_unique<mpcc::dubins::LineSegment<T>>(*line));
             } else if (auto circle = dynamic_cast<
                            const mpcc::dubins::CircularSegment<T>*>(
                            segment.get())) {
               self.add_segment(
                   std::make_unique<mpcc::dubins::CircularSegment<T>>(*circle));
             } else {
               throw std::runtime_error("Unknown segment type");
             }
           })
      .def("num_segments", &mpcc::dubins::DubinsPath<T>::num_segments)
      .def("get_segment", &mpcc::dubins::DubinsPath<T>::get_segment);
}

}  // namespace pympcc