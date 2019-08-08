////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Vinitha Ranganeni, Brian Lee
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

#ifndef COZMO_GENERIC_ACTION_SPACE_CPP
#define COZMO_GENERIC_ACTION_SPACE_CPP

#include "utils/utils.hpp"
#include <Eigen/Dense>
#include <cmath>
#include<ros/ros.h>

namespace libcozmo {
namespace actionspace {

// Generic Action class
class GenericAction {
  public:

    // Constructor

    // \param speed The speed of action (m/s)
    // \param duration The duration of action (s)
    // \param direction The (x,y) direction vector of action (m)
    GenericAction (
        const double& speed,
        const double& duration,
        const Eigen::Vector2d& direction) : \
        m_speed(speed),
        m_duration(duration),
        m_direction(direction) {}
    ~GenericAction() {}

    // Speed of action
    const double m_speed;

    // Duration of action
    const double m_duration;

    // (x,y) direction vector
    const Eigen::Vector2d m_direction;
};

// Generic Actionspace for Cozmo
class GenericActionSpace {
  public:

    // Constructor
    
    // \param min_speed The minimum speed of action
    // \param max_speed The maximum speed of action
    // \param min_duration The minimum duration of action
    // \param max_duration The maximum duration of action
    // \param num_speed The number of speed values for the actions
    // \param num_duration The number of duration values for the actions
    // \param num_directions The number of direction values for the actions
    // num_directions must be in powers of 2 and >= 4
    GenericActionSpace(
        const double& min_speed,
        const double& max_speed,
        const double& min_duration,
        const double& max_duration,
        const int& num_speed,
        const int& num_duration,
        const int& num_directions) {
            std::vector<double> m_speeds = utils::linspace(min_speed, max_speed, num_speed);
            std::vector<double> m_durations = utils::linspace(min_duration, max_duration, num_duration);
            std::vector<Eigen::Vector2d> m_directions;
			for (int i = 0; i < num_directions; i++) {
            	const double angle = i * 2.0 * M_PI / static_cast<double>(num_directions);
            	m_directions.push_back(Eigen::Vector2d(cos(angle), sin(angle)));
            }
            m_actions = std::vector<GenericAction*>(num_speed * num_duration * num_directions, nullptr);
            for (int j = 0; j < num_speed; j++) {
                for (int k = 0; k < num_duration; k++) {
                  	for (int l = 0; l < num_directions; l++) {
                    	const int id = j * num_duration * num_directions + k * num_directions + l;
                    	m_actions[id] = new GenericAction(
                      		m_speeds[j], 
                      		m_durations[k], 
                      		m_directions[l]); 
                  } 
                }
            }
        }
    ~GenericActionSpace() {
    	for (int i = 0; i < m_actions.size(); ++i) { 
        	delete(m_actions[i]);
		}
		m_actions.clear();
    }

	// Calculates and returns similarity with another action
    // Similarity is defined by the euclidean distance w.r.t.
    // their speed, duration, and direction (x,y calcualted separately)

    // \param action_id1, actionid2 The ID of two actions to compare
    double action_similarity(const int& action_id1, const int& action_id2) const;

    // Returns pointer to action given action ID

    // \param action_id The action ID
    GenericAction* get_action(const int& action_id) const;

    // Executes action on cozmo given action ID

    // \param action_id The action ID
    void execute_action(int& action_id) const;

  private:
    // // Vector of actions
    std::vector<GenericAction*> m_actions;
};
}  // namespace actionspace
}  // namespace libcozmo

#endif