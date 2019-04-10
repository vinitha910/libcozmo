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
		.def("setForkliftPosition", &Cozmo::setForkliftPosition, py::arg("pos"));
}

}  // namespace python
}  // namesapce libcozmo