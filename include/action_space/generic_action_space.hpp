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

#ifndef LIBCOZMO_ACTIONSPACE_GENERICACTIONSPACE_HPP
#define LIBCOZMO_ACTIONSPACE_GENERICACTIONSPACE_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include "utils/utils.hpp"
#include <libcozmo/ActionMsg.h>
#include "ActionSpace.hpp"

namespace libcozmo {
namespace actionspace {

/// This class represents the generic actionspace for Cozmo
/// All actions are not planned specific to any object, but instead
/// with respect to the Cozmo robot itself
class GenericActionSpace : public virtual ActionSpace {
 public:
        /// Generic Action class
    class Action : public ActionSpace::Action {
      public:
        /// Constructor for identity state
        /// Action(): m_speed(0), m_duration(0), m_direction(Eigen::Vector2d(0, 0)) {};
        
        /// ~Action() = default;

        /// Constructor

        /// \param speed The speed of action (m/s)
        /// \param duration The duration of action (s)
        /// \param direction The (x,y) direction vector of action (m)
        explicit Action(
            const double& speed,
            const double& duration,
            const double& heading) : \
            m_speed(speed),
            m_duration(duration),
            m_heading(heading) {}

        /// Speed of action (m/s)
        const double m_speed;

        /// Duration of action (s)
        const double m_duration;

        /// Heading of action (radians)
        const double m_heading;

    };
    /// Constructor

    /// \param min_speed The minimum speed of action
    /// \param max_speed The maximum speed of action
    /// \param min_duration The minimum duration of action
    /// \param max_duration The maximum duration of action
    /// \param num_speed The number of speed values for the actions
    /// \param num_duration The number of duration values for the actions
    /// \param num_directions The number of direction values for the actions
    /// num_directions must be in powers of 2 and >= 4
    GenericActionSpace(
        const std::vector<double>& m_speeds,
        std::vector<double> m_directions,
        const double& min_duration,
        const double& max_duration,
        const int& num_duration) {
            int num_speed = m_speeds.size();
            int num_directions = m_directions.size();
            std::vector<double> m_durations = utils::linspace(
                min_duration,
                max_duration,
                num_duration);
            m_actions = std::vector<Action*>(
                num_speed *
                num_duration *
                num_directions,
                nullptr);
            for (int j = 0; j < num_speed; j++) {
                for (int k = 0; k < num_duration; k++) {
                    for (int l = 0; l < num_directions; l++) {
                        const int id = j * num_duration * num_directions +
                            k * num_directions + l;
                        m_actions[id] = new Action(
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

    /// Documentation inherited
    bool action_similarity(
        const int& action_id1,
        const int& action_id2,
        double* similarity) const override;

    /// Documentation inherited
    bool get_action(const int& action_id, ActionSpace::Action* action) const override;

    void publish_action(const int& action_id, const ros::Publisher& publisher) const override;

    bool is_valid_action_id(const int& action_id) const override;

 private:
    /// Vector of actions
    std::vector<Action*> m_actions;
};
}  /// namespace actionspace
}  /// namespace libcozmo

#endif
