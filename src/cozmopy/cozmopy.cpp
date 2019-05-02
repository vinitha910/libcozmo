#include <pybind11/pybind11.h>
#include <cozmo_description/cozmo.hpp>

namespace py = pybind11;

namespace libcozmo {
namespace python {

PYBIND11_MODULE(cozmopy, m)
{
	py::class_<libcozmo::Cozmo>(m, "Cozmo")
		.def(py::init<const std::string&>())
		.def("getCozmoSkeleton", &Cozmo::getCozmoSkeleton)
		.def("setForkliftPosition", &Cozmo::setForkliftPosition, py::arg("pos"))
		.def("createState", &Cozmo::createState, py::arg("x"), py::arg("y"), py::arg("th"));

	py::class_<libcozmo::Waypoint>(m, "Waypoint")
		.def(py::init([](const double x, const double y, const double th, const double t) {
			libcozmo::Waypoint w = {.x = x, .y = y, .th = th, .t = t};
			return w;
		}));
}

}  // namespace python
}  // namesapce libcozmo