#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <libcozmo/cozmo_description/cozmo.hpp>

namespace py = pybind11;

namespace libcozmo {
namespace python {

void Cozmo(pybind11::module& m)
{
	py::class_<libcozmo::Cozmo>(m, "Cozmo")
		.def(py::init<const std::string&>())
		.def("getCozmoSkeleton", &Cozmo::getCozmoSkeleton)
		.def("setForkliftPosition", &Cozmo::setForkliftPosition, py::arg("pos"));
}

}  // namespace python
}  // namesapce libcozmo