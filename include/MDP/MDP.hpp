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

#ifndef INCLUDE_MDP_MDP_H_
#define INCLUDE_MDP_MDP_H_

#include <Eigen/Dense>
#include <vector>
// #include <utility>
#include <unordered_map>
#include "aikido/distance/SE2.hpp"
#include "statespace/statespace.hpp"
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"

namespace libcozmo {
namespace mdp {

// Action for agent to apply on object
struct Action {
  Eigen::Vector2d velocity;
  Eigen::Vector2d position;
  double duration;
};

// Custom Hash function for a action
struct ActionHasher {
    std::size_t operator()(const Action& action) const {
        using boost::hash_value;
        using boost::hash_combine;
        std::size_t seed = 0;
        hash_combine(seed, hash_value(action.velocity.x()));
        hash_combine(seed, hash_value(action.velocity.y()));
        hash_combine(seed, hash_value(action.position.x()));
        hash_combine(seed, hash_value(action.position.y()));
        hash_combine(seed, hash_value(action.duration));
        return seed;
    }
};

// This class implements the Markov Decision process
class MDP {
 public:
    // Constructor

    // \param discount_factor The discounting factor
    // \param res The statespace resolution
    // \param num_theta The number of discrete theta values
    MDP(
        const int& discount_factor,
        const double& res,
        const int& num_theta,
        const int& current_state) : \
        gamma(discount_factor),
        m_resolution(res),
        m_num_theta_vals(num_theta),
        m_current_state_id(current_state),
        myStateSpace(m_resolution, m_num_theta_vals) {
            // initialize subscriber for cozmo
            // ros::spin();   
            // read first location and current location as idx 0
    }

    ~MDP() {}

    // Selects and executes a function
    // Records the change in state given action
    void explore() const;

    // Finds most similar and dissimilar action
    Action action_similarity(Action& action, std::vector<Action>& actions) const;

 private:

    // Generates a set of actions
    std::vector<Action> generate_actions() const;

    // Choose action based on novelty policy

    // \param current_action The action executed last
    // \param actions Vector of actions
    void choose_action_novelty(const Action& current_action, std::vector<Action> actions) const;

    // Returns cozmo's location

    // \param msg The message received
    void callback(const std_msgs::String::ConstPtr& msg);

    // Returns current state of cozmo
    aikido::statespace::SE2 get_cozmo_pose() const;

    // The discount factor
    double gamma;

    // The discrete statespace
    libcozmo::statespace::Statespace myStateSpace;

    // The resolution of statespace
    const double m_resolution;

    // The number of discrete theta values
    const int m_num_theta_vals;

    // The ID of the current state
    int m_current_state_id;

    // Maps action to number executed
    std::unordered_map<Action, int, ActionHasher> m_action_count_map;

    // Maps action to its average delta state
    std::unordered_map<Action, double, ActionHasher> m_action_delta_map;

    // ROS subscriber for cozmo
    const ros::NodeHandle cozmo_handle;
};

}  // namespace MDP
}  // namespace libcozmo

#endif  // INCLUDE_MDP_MDP_H_
