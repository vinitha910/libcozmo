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

#include "MDP/MDP.hpp"

namespace libcozmo {
namespace mdp {

    void MDP::explore() const {

        while(true) {
            geometry_msgs::PoseConstPtr pose_msg;
            do {
                pose_msg =
                    ros::topic::waitForMessage<geometry_msgs::Pose>("cozmo_pose",
                        ros::Duration(5));
            } while (pose_msg == NULL);
            
            const auto position = pose_msg->position;
            const auto orientation = pose_msg->orientation;
            Eigen::Isometry3d transformation = 
                Eigen::Translation3d(position.x, position.y, position.z) *
                Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z);
           
            // auto rotation_mat = quaternion.toRotationMatrix();
            // auto euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

        //     // waits for message from pose publisher
        //     ros::topic::waitForMessage 	("cozmo_pose", cozmo_handle);
        //     // convert message(position, quant) into SE2 state
        //     // then insert it into statespace

        //     // current location set by m_current_state_id
        //     // listen to rosnode to get current location
        //     // use get_or_create_new_state on current location, then
        //     // set m_current_state id to that idx
            
        //     //generate arbitrary functions to explore
        //     std::vector<Action> = generate_actions();
        //     // apply chosen action
        //     // I can use eric's thing here

        //     // record new location
        //     // create new state at new location

        //     //update model, calculate transition
        //     // use get_distance to get transition in delta distance, delta theta
        //     // if action exists, add and average it
        //     // otherwise create new action
        }

    }

    // aikido::statespace::SE2 MDP::get_cozmo_pose() const {

    // }

    // Action MDP::action_similarity(Action& action, std::vector<Action> actions) const {
    //     Action similar;
    //     // double similar_cost = - infity;
    //     Action dissimilar;
    //     // double dissimilar_cost = infty;
        
    //     for (int i = 0; i < actions.size(); i++) {
    //         Action new_action = actions[i];
    //         // if (myStateSpace.get_distance( the two states) > similar_cost) {
    //             // similar = new_action;
    //             // similar_cost = whatever I just compted
    //         // }
    //         // if (myStateSpace.get_distance( the two states) < similar_cost) {
    //             // dissimilar = new_action;
    //             // dissimilar_cost = whatever I just compted
    //         // }
    //     }
    // }

    // std::vector<Action> MDP::generate_actions() const {
    //     std::vector<Action> actions;
    //     actions.push_back(Action(Eigen::Vector2d(0, -1), Eigen::Vector2d(0, 1), 1.0));
    //     actions.push_back(Action(Eigen::Vector2d(0, 1), Eigen::Vector2d(0, -1), 1.0));
    //     actions.push_back(Action(Eigen::Vector2d(-1, 0), Eigen::Vector2d(1, 0), 1.0));
    //     actions.push_back(Action(Eigen::Vector2d(1, 0), Eigen::Vector2d(-1, 0), 1.0));
    //     return actions;
    // }

    void callback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("I heard: [%s]", msg->data.c_str());
    }

}  // namespace statespace
}  // namespace libcozmo

