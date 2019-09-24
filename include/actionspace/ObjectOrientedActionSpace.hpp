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

#ifndef OBJECT_ORIENTED_ACTION_SPACE
#define OBJECT_ORIENTED_ACTION_SPACE

#include <libcozmo/ObjectOrientedAction.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include "ActionSpace.hpp"

namespace libcozmo {
namespace actionspace {

/// Macros indicating orientation (radian) correponding to each side of the cube
///
/// The side of the cube opposite to side with the screw at its bottom is
/// identified by cozmo as the front, with 0 orientation.
#define FRONT 0
#define LEFT M_PI / 2
#define BACK M_PI
#define RIGHT 3 * M_PI / 2

/// This class handles actions consisting of speed, edge offset, aspect ration,
/// and heading offset. The actions are handled as generic, where each action is
/// unique but not callibrated to the location of the cube.
///
/// When necessary, such as during successor calculation or action execution,
/// the actionspace can convert the generic action to object oriented action
/// given the pose of the cube.
class ObjectOrientedActionSpace : public virtual ActionSpace {
 public:
    /// This class handles generic attributes to the action that can be
    /// executed by cozmo, which are speed (mm / s), edge offset (m),
    /// aspect ratio (mm), and heading offset (radian)
    class GenericAction : public ActionSpace::Action {
     public:
        /// Constructs action with given attributes
        ///
        /// \param speed : the speed of cozmo, in mm / s, negative speed
        ///     refers to backward movement
        /// \param edge_offset : normalized distance from center of edge
        ///     in range [-1, 1], where -1 and 1 are the left and right
        ///     corner respectively
        /// \param aspect_ratio : the aspect ratio of the side cozmo is
        ///     pushing from, indicated by different side length.
        /// \param heading_offset : The angular distance from front of
        ///     the cube, indicating which side of the cube the action
        ///     is being applied to
        explicit GenericAction(
            const double& speed,
            const double& edge_offset,
            const double& aspect_ratio,
            const double& heading_offset) : \
            m_speed(speed),
            m_edge_offset(edge_offset),
            m_aspect_ratio(aspect_ratio),
            m_heading_offset(heading_offset) {}

        double getSpeed() const { return m_speed; }
        double getAspectRatio() const { return m_aspect_ratio; }
        double getEdgeOffset() const { return m_edge_offset; }
        double getHeadingOffset() const { return m_heading_offset; }

     private:
        double m_speed;
        double m_aspect_ratio;
        double m_edge_offset;
        double m_heading_offset;

        friend class ObjectOrientedActionSpace;
    };

    /// This is an action class that obtains information related to the
    /// pose of the cube for execution/successor calculation
    class ObjectOrientedAction : public ActionSpace::Action {
     public:
            /// Constructs object oriented action, indicating starting position
            /// w.r.t the object pose
            ///
            /// \param speed : the speed of cozmo, in mm / s, negative speed
            ///     refers to backward movement
            /// \param start_pose : starting pose of cozmo's action,
            ///     its attributes are x(mm), y(mm), and theta(radian)
        explicit ObjectOrientedAction(
            const double& speed,
            const Eigen::Vector3d& start_pose) : \
            m_speed(speed),
            m_start_pose(start_pose) {}

        double getSpeed() const { return m_speed; }
        Eigen::Vector3d getStartPose() const { return m_start_pose; }

     private:
            double m_speed;
            Eigen::Vector3d m_start_pose;

            friend class ObjectOrientedActionSpace;
    };

    /// \param speeds : all possible speeds (mm/s) for the actions
    ///     (duplicates persist)
    /// \param ratios : The aspect ratio of sides of the cube, given a
    ///     rectangular object size(ratios) equals 2. The ratio refers to
    ///     numeric length (mm) of each unique side length.
    /// \param edge_offset : The max distance, along the edge of
    ///     the object, from the center of that edge (mm)
    /// \param num_offset : number of starting position offsets on each side
    ///     of the object; this value must always be odd
    ObjectOrientedActionSpace(
        const std::vector<double>& speeds,
        const std::vector<double>& ratios,
        const double& edge_offset,
        const int& num_offset = 40);

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
    /// to the pose of the cube, for both successor state calculation and
    /// cozmo's action excution.
    ///
    /// \param _action The generic action
    /// \param _state State of the cube indicating its pose
    /// \param[out] action The object oriented action to modify
    void generic_to_object(
        const GenericAction& _action,
        const aikido::statespace::StateSpace::State& _state,
        ObjectOrientedAction* action) const;

    /// Documentation inherited
    bool publish_action(
        const int& action_id,
        const ros::Publisher& publisher,
        const aikido::statespace::StateSpace::State& _state) const;

    /// Documentation inherited
    int size() const;

 private:
    std::vector<double> m_speeds;
    std::vector<double> m_ratios;
    int num_offset;
    double m_center_offset;
    double m_edge_offset;
    std::vector<GenericAction*> m_actions;
    ros::Publisher action_publisher;
};

}  // namespace actionspace
}  // namespace libcozmo

#endif
