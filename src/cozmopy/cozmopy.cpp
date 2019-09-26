#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <cozmo_description/cozmo.hpp>
#include "actionspace/ObjectOrientedActionSpace.hpp"
#include <chrono>
#include <Eigen/Geometry>

namespace py = pybind11;

namespace libcozmo {
namespace python {

PYBIND11_MODULE(cozmopy, m)
{
	// from aikidopy import StateSpace, Interpolated
	py::object StateSpace = py::module::import("aikidopy").attr("StateSpace");
	py::object Interpolated = py::module::import("aikidopy").attr("Interpolated");
	
	py::class_<libcozmo::Cozmo>(m, "Cozmo")
		.def(py::init<const std::string&>())
		.def("getCozmoSkeleton", &Cozmo::getCozmoSkeleton)
		.def("setForkliftPosition", &Cozmo::setForkliftPosition, py::arg("pos"))
		.def("createState", &Cozmo::createState, py::arg("x"), py::arg("y"), py::arg("th"))
		.def("createInterpolatedTraj", &Cozmo::createInterpolatedTraj, py::arg("waypoints"))
		.def("executeTrajectory", [](
			libcozmo::Cozmo& cozmo,
			const std::chrono::duration<double, std::milli> milliseconds, 
			aikido::trajectory::TrajectoryPtr traj) 
		{
			return cozmo.executeTrajectory(
				std::chrono::duration_cast<std::chrono::milliseconds>(milliseconds), traj);
		})
        .def("setState", [](
            libcozmo::Cozmo& cozmo,
            const double& x, 
            const double& y, 
            const std::vector<double>& quat)        
        {
            Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
            cozmo.setState(x, y, q);
        });
        
        py::class_<libcozmo::Waypoint>(m, "Waypoint")
		.def(py::init([](const double x, const double y, const double th, const double t) {
			libcozmo::Waypoint w = {.x = x, .y = y, .th = th, .t = t};
			return w;
		}));

	py::class_<libcozmo::actionspace::ObjectOrientedActionSpace>(m, "ObjectOrientedActionSpace")
		.def(py::init<const std::vector<double>&, 
					  const std::vector<double>&,
					  const Eigen::Vector2d&,
					  const Eigen::Vector2d&,
					  const int&>());
}

}  // namespace python
}  // namesapce libcozmo
