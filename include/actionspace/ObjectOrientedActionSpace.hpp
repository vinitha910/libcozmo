////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Brian Lee, Eric Pan, Vinitha Ranganeni
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

#ifndef INCLUDE_ACTIONSPACE_OBJECTORIENTEDACTIONSPACE_HPP_
#define INCLUDE_ACTIONSPACE_OBJECTORIENTEDACTIONSPACE_HPP_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include "libcozmo/ObjectOrientedAction.h"
#include "ActionSpace.hpp"

namespace libcozmo {
namespace actionspace {

/// Macros defining Cozmo's heading offset (radians) to face the corresponding
/// sides of the object
#define FRONT 0.0
#define LEFT M_PI / 2
#define BACK M_PI
#define RIGHT 3 * M_PI / 2

/// An Object Oriented Action Space is defined by the speed and duration of
/// Cozmo at a location relative to a rectangular object. We store the actions
/// as generic actions (i.e. the action is not generated with respect to an
/// object's pose). These generic actions can then be converted to object
/// oriented actions as needed.
///
/// Each object oriented action contains a starting pose for Cozmo along one of
/// the sides of the object, from which the action will be executed. The are 4
/// main positions (each at a center along each side of the object); the user
/// can specify n offsets from each center. The number of object oriented
/// actions in the action space is 4 * num_offsets * num_speeds * num_durations
/// Each object oriented action also contains the speed of the action (mm / s).
///
/// For efficiency, the class handles actions based on their unique ID. Each ID
/// belongs to a generic action, and the object oreinted action for a given ID
/// can be calculated as well. Generally, the object oriented action is only
/// calculated when Cozmo is executing a plan.
///
/// Below is an example object with its respective edge offset values and
/// labelled sides. Note, opposite corners have the same edge offsets since
/// applying the action with the same parameters on opposite corners results in
/// the same movement of the object.
///
///                 BACK
///        +1        0       -1
///         -------------------
///         |                 |
/// LEFT  0 |        |--------|--►  v1    0  RIGHT
///         |        |        |
///         ---------|---------
///                  ▼ v2
///
///        -1        0        +1
///                FRONT
///
/// Note the vectors |v1| and |v2|, which refer to the x and y offsets from the
/// center of the object respectively.
class ObjectOrientedActionSpace : public virtual ActionSpace {
 public:
    /// This class handles generic attributes to the action that can be
    /// executed by cozmo, which are speed (mm / s), edge offset (mm),
    /// aspect ratio (mm), and heading offset (radian)
    class Action : public ActionSpace::Action {
     public:
        /// Constructs action with given attributes
        ///
        /// \param speed : the speed of cozmo, in mm / s, negative speed
        ///     refers to backward movement
        /// \param edge_offset : normalized distance from center of edge
        ///     in range [-1, 1], where -1 and 1 are the left and right
        ///     corner respectively
        /// \param aspect_ratio : the aspect ratio of the side cozmo is
        ///     pushing from, indicated by different side length. (mm)
        /// \param heading_offset : The angular distance from front of
        ///     the object, indicating which side of the object the action
        ///     is being applied to (radians)
        explicit Action(
            const double& speed,
            const double& edge_offset,
            const double& aspect_ratio,
            const double& heading_offset);

        double speed() const;
        double aspect_ratio() const;
        double edge_offset() const;
        double heading_offset() const;

        /// Documentation inherited
        /// The action vector is in the following format:
        /// [speed, aspect_ratio, edge_offset, heading_offset]
        Eigen::VectorXd vector() const override;

     private:
        double m_speed;
        double m_aspect_ratio;
        double m_edge_offset;
        double m_heading_offset;

        friend class ObjectOrientedActionSpace;
    };

    /// This class contains the parameters for executing an objected oriented
    /// action. These parameters are calculated from the corresponding generic
    /// action parameters.
    class CozmoAction : public ActionSpace::Action {
     public:
            /// Constructs object oriented action w.r.t the object position
            ///
            /// \param speed : the speed of cozmo, in mm / s, negative speed
            ///     refers to backward movement
            /// \param start_pose : starting pose of cozmo's action,
            ///     its attributes are x(mm), y(mm), and theta(radians)
        explicit CozmoAction(
            const double& speed,
            const Eigen::Vector3d& start_pose);

        double speed() const;
        Eigen::Vector3d start_pose() const;

        /// Documentation inherited
        /// The action vector is in the following format:
        /// [speed, start_pose_x, start_pose_y, start_pose_theta]
        Eigen::VectorXd vector() const override;

     private:
            double m_speed;
            Eigen::Vector3d m_start_pose;

            friend class ObjectOrientedActionSpace;
    };

    /// \param speeds : all possible speeds (mm/s) for the actions
    ///     (duplicates persist)
    /// \param ratios : The ratios (mm) of the object.
    ///     For example, if the size of the object is 300mm x 100mm then the
    ///     ratio = {300, 100} = {width, length}
    /// \param center_offsets : The distance (mm) along a vector, from the
    ///     center of the object, that is perpendicular to an edge.
    ///     For example, |v1| and |v2| are the x and y offsets from the
    ///     center respectively
    /// \param max_edge_offsets : The max distances (mm), along the edge of
    ///     the object, from the center of that edge.
    /// \param num_edge_offsets : number of starting position offsets on each
    ///     side of the object; this value must always be odd
    ObjectOrientedActionSpace(
        const std::vector<double>& speeds,
        const std::vector<double>& ratios,
        const Eigen::Vector2d& center_offsets,
        const Eigen::Vector2d& max_edge_offsets,
        const int& num_edge_offsets);

    ~ObjectOrientedActionSpace() {
        for (size_t i = 0; i < m_actions.size(); ++i) {
            delete(m_actions[i]);
        }
        m_actions.clear();
    }

    /// Calculates the similarity between two actions in the action space
    /// Similarity is defined by the euclidean distance between all
    /// attributes of an ObjectOrientedAction
    ///
    /// \param action_id1, action_id2 : The id's of the actions to compare
    /// \param[out] similarity : The similarity value between two actions
    ///     value of 0 means they are identical
    /// \return true if successful, false otherwise
    bool action_similarity(
        const int& action_id1,
        const int& action_id2,
        double* similarity) const;

    /// Documentation inherited
    ActionSpace::Action* get_action(const int& action_id) const;

    /// Documentation inherited
    bool is_valid_action_id(const int& action_id) const;

    /// Converts a generic action to an object oriented action with respect
    /// to the pose of the object.
    ///
    /// \param _action The generic action
    /// \param _state State of the cube indicating its pose
    /// \param[out] action The object oriented action to modify
    bool get_generic_to_object_oriented_action(
        const int& action_id,
        const aikido::statespace::StateSpace::State& _state,
        CozmoAction* action) const;

    /// Documentation inherited
    bool publish_action(
        const int& action_id,
        const ros::Publisher& publisher,
        const aikido::statespace::StateSpace::State& _state) const;

    /// Documentation inherited
    int size() const;

 private:
    const std::vector<double> m_speeds;
    const std::vector<double> m_ratios;
    const Eigen::Vector2d m_center_offsets;
    const Eigen::Vector2d m_max_edge_offsets;
    std::vector<Action*> m_actions;
    ros::Publisher action_publisher;
};

}  // namespace actionspace
}  // namespace libcozmo

#endif  // INCLUDE_ACTIONSPACE_OBJECTORIENTEDACTIONSPACE_HPP_
