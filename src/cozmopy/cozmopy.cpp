////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Vinitha Ranganeni, Eric Pan
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <Eigen/Geometry>
#include <chrono>
#include <cozmo_description/cozmo.hpp>
#include "actionspace/ObjectOrientedActionSpace.hpp"

namespace py = pybind11;

namespace libcozmo {
namespace python {

PYBIND11_MODULE(cozmopy, m) {
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
            aikido::trajectory::TrajectoryPtr traj) {
            return cozmo.executeTrajectory(
                std::chrono::duration_cast<std::chrono::milliseconds>(milliseconds), traj);
        })
        .def("setState", [](
            libcozmo::Cozmo& cozmo,
            const double& x,
            const double& y,
            const std::vector<double>& quat) {
            Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
            cozmo.setState(x, y, q);
        })
        .def("getState", &Cozmo::getState, py::arg("path"), py::arg("time"));

        py::class_<libcozmo::Waypoint>(m, "Waypoint")
        .def(py::init([](const double x, const double y, const double th, const double t) {
            libcozmo::Waypoint w = {.x = x, .y = y, .th = th, .t = t};
            return w;
        }));

    py::class_<actionspace::ObjectOrientedActionSpace>(m, "ObjectOrientedActionSpace")
        .def(py::init<const std::vector<double>&,
                      const std::vector<double>&,
                      const Eigen::Vector2d&,
                      const Eigen::Vector2d&,
                      const int&>())
        .def("action_similarity", [](
            const actionspace::ObjectOrientedActionSpace& actionspace,
            const int& action_id1,
            const int& action_id2) {
            double similarity = -1;
            bool successful =
                actionspace.action_similarity(action_id1, action_id2, &similarity);
            return similarity, successful;
        })
        .def("get_action",
            &actionspace::ObjectOrientedActionSpace::get_action,
            py::arg("action_id"))
        .def("is_valid_action_id",
            &actionspace::ObjectOrientedActionSpace::is_valid_action_id,
            py::arg("action_id"))
        .def("get_generic_to_object_oriented_action", [](
            const actionspace::ObjectOrientedActionSpace& actionspace,
            const int& action_id,
            const aikido::statespace::StateSpace::State& _state){
            actionspace::ObjectOrientedActionSpace::CozmoAction action(
                0.0, Eigen::Vector3d(0, 0, 0));
            bool successful = actionspace.get_generic_to_object_oriented_action(
                action_id, _state, &action);
            return action;
        })
        .def("size", &actionspace::ObjectOrientedActionSpace::size);

        py::class_<actionspace::ObjectOrientedActionSpace::Action>(m, "Action")
            .def(py::init<const double&, const double&, const double&, const double&>())
            .def("speed", &actionspace::ObjectOrientedActionSpace::Action::speed)
            .def("aspect_ratio", &actionspace::ObjectOrientedActionSpace::Action::aspect_ratio)
            .def("edge_offset", &actionspace::ObjectOrientedActionSpace::Action::edge_offset)
            .def("heading_offset", &actionspace::ObjectOrientedActionSpace::Action::heading_offset);

        py::class_<actionspace::ObjectOrientedActionSpace::CozmoAction>(m, "CozmoAction")
            .def(py::init<const double&, const Eigen::Vector3d&>())
            .def("speed", &actionspace::ObjectOrientedActionSpace::CozmoAction::speed)
            .def("start_pose", &actionspace::ObjectOrientedActionSpace::CozmoAction::start_pose);
}

}  // namespace python
}  // namespace libcozmo
