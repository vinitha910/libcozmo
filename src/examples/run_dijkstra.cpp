////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019,  Brian Lee, Vinitha Ranganeni
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

#include <chrono>
#include "planner/Dijkstra.hpp"
#include "distance/SE2.hpp"
#include "distance/translation.hpp"

/// Example script to run Dijkstra's algorithm, and run the calculated
/// sequenec of actions on the Cozmo via Cozmo SDK.
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "Dijkstra");
    ros::NodeHandle handle;
    ros::Publisher publisher =
        handle.advertise<libcozmo::ActionMsg>("Action", 10);
    auto type_ = libcozmo::model::DeterministicModel::DeterministicModelType();
    libcozmo::statespace::SE2 state_space(10, 4);
    int start = state_space.get_or_create_state(
    libcozmo::statespace::SE2::State(0, 0, 0));
    int goal = state_space.get_or_create_state(
    libcozmo::statespace::SE2::State(25, 25, 3));
    libcozmo::actionspace::GenericActionSpace action_space(
        std::vector<double>{0, 150, 200, 250},
        std::vector<double>{1},
        4);
    libcozmo::model::DeterministicModel model;
    model.load_model(&type_);
    auto distance_metric =
        libcozmo::distance::SE2(
            std::make_shared<libcozmo::statespace::SE2>(10, 4));
    libcozmo::planner::Dijkstra m_solver(
        &action_space, &state_space, &model, &distance_metric, 1);
    std::vector<int> actions;
    m_solver.set_start(start);
    m_solver.set_goal(goal);
    auto start_time = std::chrono::high_resolution_clock::now();
    bool solved  = m_solver.solve(&actions);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            stop - start_time);
    std::cout << "NUM ACTIONS " << actions.size() << "\n";
    std::cout << "TIME TAKEN " << duration.count() << "\n";
    for (int i = 0; i < actions.size(); i++) {
        bool sent = false;
        sent = action_space.publish_action(actions[i], publisher);
        if (sent) {
            std::cout << "msg published: action id " << actions[i] << "\n";
            std::cout << "speed: " << static_cast<
                libcozmo::actionspace::GenericActionSpace::Action*>(
                    action_space.get_action(actions[i]))->m_speed << "\n";
            std::cout << "heading: " << static_cast<
                libcozmo::actionspace::GenericActionSpace::Action*>(
                    action_space.get_action(actions[i]))->m_heading << "\n";
            std::cout << "duration: " << static_cast<
                libcozmo::actionspace::GenericActionSpace::Action*>(
                    action_space.get_action(actions[i]))->m_duration << "\n";
        } else {
            std::cout << "Failed to publish." << "\n";
        }
    }
    return 0;
}

