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
    
    // Calculates and returns similarity with another action
    // Similarity is defined by the euclidean distance w.r.t.
    // their speed, duration, and direction (x,y calcualted separately)

    // \param other_action The other action to compare to
    double action_similarity(GenericAction& other_action) const;

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
    // \param num_theta The number of theta values for the actions
    GenericActionSpace(
        const double& min_speed,
        const double& max_speed,
        const double& min_duration,
        const double& max_duration,
        const int& num_speed,
        const int& num_duration,
        const int& num_theta) {
            m_speeds = utils::linspace(min_speed, max_speed, static_cast<size_t>(num_speed));
            m_durations = utils::linspace(min_duration, max_duration, num_duration);
            for (int i = 0; i < num_theta; i++) {
              double angle = i * 2.0 * M_PI / static_cast<double>(num_theta);
              m_directions.push_back(Eigen::Vector2d(cos(angle), sin(angle)));
            }
            m_actions = std::vector<GenericAction*>(num_speed * num_duration * num_theta, nullptr);
            for (int j = 0; j < num_speed; j++) {
                for (int k = 0; k < num_duration; k++) {
                  for (int l = 0; l < num_theta; l++) {
                    const int id = j * num_duration * num_theta + k * num_theta + l;
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

    // Assigns pointer to action given action ID

    // \param action_id The action ID
    // \param action Pointer for chosen action
    GenericAction* get_action(const int& action_id) const;

    // Executes action given action ID

    // \param action_id The action ID
    void execute_action(int& action_id) const;

  private:
    // Vector of possible speed
    std::vector<double> m_speeds;
    // Vector of possible durations
    std::vector<double> m_durations;
    // Linspace count for speed and duration 
    // const int m_num_vals;
    // // Linspace count for angle, in power of 2 and >= 4
    // const int m_num_theta;
    // // Vector of actions
    std::vector<GenericAction*> m_actions;
    // Vector of possible directions
    std::vector<Eigen::Vector2d> m_directions;
};
}  // namespace actionspace
}  // namespace libcozmo

#endif