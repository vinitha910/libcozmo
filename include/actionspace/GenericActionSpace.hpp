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

#ifndef INCLUDE_ACTIONSPACE_GENERICACTIONSPACE_HPP_
#define INCLUDE_ACTIONSPACE_GENERICACTIONSPACE_HPP_

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "utils/utils.hpp"
#include "ActionSpace.hpp"

namespace libcozmo {
namespace actionspace {

/// Actions are applicable regardless of Cozmo's state in the world
///
/// Actions consist of speed, duration and heading
/// The total number of possible actions =
/// number of speeds * number of durations * number of headings
///
/// Headings are represented as angles from [0, 2pi]. The difference between
/// two headings is 2pi/number_of_headings.
class GenericActionSpace : public virtual ActionSpace {
 public:
    /// Generic Action class
    class Action : public ActionSpace::Action {
     public:
        /// Constructor

        /// \param speed The speed of action (mm/s)
        /// \param duration The duration of action (s)
        /// \param heading The heading of action (radians)
        explicit Action(
            const double& speed,
            const double& duration,
            const double& heading) : \
            m_speed(speed),
            m_duration(duration),
            m_heading(heading) {}

        /// Documentation inherited
        /// The action vector is in the following format:
        /// [speed, duration, heading]
        Eigen::VectorXd vector() const override {
            Eigen::VectorXd action_vector(3);
            action_vector << m_speed, m_duration, m_heading;
            return action_vector;
        }
        const double m_speed;
        const double m_duration;
        const double m_heading;
    };

    /// Constructs action space with given possible options for speed, duration
    /// and heading
    ///
    /// \param m_speeds Vector of available speeds
    /// \param m_durations Vector of available durations
    /// \param num heading Number of options for heading/direction (required:
    /// power of 2 and >= 4)
    GenericActionSpace(
        const std::vector<double>& speeds,
        const std::vector<double>& durations,
        const int& num_headings);

    ~GenericActionSpace() {
        for (int i = 0; i < m_actions.size(); ++i) {
            delete(m_actions[i]);
        }
        m_actions.clear();
    }

    /// Calculates similarity between two actions
    /// Similarity based on the Euclidean distance between the actions
    ///
    /// \param action_id1, actionid2 IDs of actions to compare
    /// \return similarity value
    bool action_similarity(
        const int& action_id1,
        const int& action_id2,
        double* similarity) const override;

    /// Documentation inherited
    ActionSpace::Action* get_action(const int& action_id) const override;

    /// Documentation inherited
    bool is_valid_action_id(const int& action_id) const override;

    /// Documentation inherited
    int size() const override;

 private:
    /// Vector of actions
    std::vector<Action*> m_actions;
};
}  /// namespace actionspace
}  /// namespace libcozmo

#endif  // INCLUDE_ACTIONSPACE_GENERICACTIONSPACE_HPP_
