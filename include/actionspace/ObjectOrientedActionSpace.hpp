////////////////////////////////////////////////////////////////////////////////
//// Copyright (c) 2019, Eric Pan, Vinitha Ranganeni
//// All rights reserved.
////
//// Redistribution and use in source and binary forms, with or without
//// modification, are permitted provided that the following conditions are met:
////
////     1. Redistributions of source code must retain the above copyright notice
////        this list of conditions and the following disclaimer.
////     2. Redistributions in binary form must reproduce the above copyright
////        notice, this list of conditions and the following disclaimer in the
////        documentation and/or other materials provided with the distribution.
////     3. Neither the name of the copyright holder nor the names of its
////        contributors may be used to endorse or promote products derived from
////        this software without specific prior written permission.
////
//// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#ifndef OBJECT_ORIENTED_ACTION_SPACE
#define OBJECT_ORIENTED_ACTION_SPACE

#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include "ActionSpace.hpp"
#include "utils/utils.hpp"
#include <libcozmo/ObjectOrientedAction.h>

namespace libcozmo {
namespace actionspace {

/// An action space class that generates possible
/// actions for Cozmo to execute relative to an object
///
/// An Object Oriented Action Space is defined by
/// the speed and duration of Cozmo at a location relative to an object
///
/// Each action contains a starting pose for cozmo along one of the sides of
/// the cube, from which the action will be executed. The are 4 main positions
/// (each at a center along each side of a cube); the user can specify n
/// offsets from each center. The number of actions in the action space is
/// 4 * num_offsets * num_speeds * num_durations

class ObjectOrientedActionSpace : public virtual ActionSpace {
    public:
        class Action : public ActionSpace::Action {
            public:
                /// \param speed : the speed of cozmo, in mm / s
                ///                negative speed refers to backward movement
                /// \param duration : the amount of time to move,
                ///                   in seconds, should be >= 0
                /// \param start_pos : Cozmo's inital (x, y, theta) position,
                ///                    in (mm, mm, radians)
                /// \param edge_offset : distance from center of edge,
                ///                      normalized [-1, 1], negative values
                ///                      are left of center, positive values
                ///                      are right of center
                explicit Action (
                    const double& speed,
                    const double& duration,
                    const Eigen::Vector3d& start_pose,
                    const double& offset) : \
                    speed(speed),
                    duration(duration),
                    start_pose(start_pose),
                    edge_offset(edge_offset) {}

                double getSpeed() const { return speed; }

                double getDuration() const { return duration; }

                Eigen::Vector3d getStartPose() const { return start_pose; }

                double getEdgeOffset() const { return edge_offset; }

            private:
                double speed;
                double duration;
                Eigen::Vector3d start_pose;
                double edge_offset;

                void update_action(
                    const double& speed,
                    const double& duration,
                    const Eigen::Vector3d& start_pose,
                    const double& edge_offset)
                {
                    this->speed = speed;
                    this->duration = duration;
                    this->start_pose = start_pose;
                    this->edge_offset = edge_offset;
                }

                friend class ObjectOrientedActionSpace;
        };

        /// \param speeds : all possible speeds (mm/s) for the actions
        ///                 (duplicates persist)
        /// \param durations : all possible durations (seconds) for the actions
        ///                    (duplicates persist)
        /// \param num_offset : number of starting position offsets on each
        ///                     side of the cube; this value must always be odd
        ObjectOrientedActionSpace(
            const std::vector<double>& speeds,
            const std::vector<double>& durations,
            const int& num_offset) : \
            speeds(speeds),
            durations(durations),
            num_offset(num_offset),
            center_offset(60) {}

        ~ObjectOrientedActionSpace() {
            for (size_t i = 0; i < actions.size(); ++i) {
                delete(actions[i]);
            }
            actions.clear();
        }

        /// Calculates the similarity between two actions in the action space
        /// Similarity is defined by the euclidean distance
        /// between all attributes of an ObjectOrientedAction
        ///
        /// \param action_id1, action_id2 : The id's of the actions to compare
        /// \param[out] similarity : The similarity value between two actions
        ///                          value of 0 means they are identical
        /// \returns true if successful, false otherwise
        bool action_similarity(
            const int& action_id1,
            const int& action_id2,
            double* similarity) const;

        /// Generates actions given another object's pose
        ///
        /// \param obj_pose : An (x, y, theta) coordinate in (mm, mm, radians)
        ///                  theta should be [0, 2pi]
        /// \param edge_offset (optional) : The max distance, along the edge of
        ///                                 the object, from the center of that
        ///                                 edge
        ///
        /// This function will overwrite any previously
        /// generated actions in the action space
        void generate_actions(
            const Eigen::Vector3d& obj_pose,
            const double& edge_offset=40);

        /// Documentation inherited
        ActionSpace::Action* get_action(const int& action_id) const;

        /// Documentation inherited
        bool is_valid_action_id(const int& action_id) const;

        /// Documentation inherited
        bool publish_action(
            const int& action_id,
            const ros::Publisher& publisher) const;

        /// Documentation inherited
        int size() const;

		void view_action_space() const;

    private:
        std::vector<double> speeds;
        std::vector<double> durations;
        int num_offset;
        double center_offset;
        std::vector<Action*> actions;

        ros::Publisher action_publisher;

        /// Finds the headings for cozmo, relative to a cartesian plane,
        /// to all 4 sides of the cube given the orientation of the cube
        ///
        /// The angles are found in clockwise order where indices correspond to
        /// (in order) the front, left, back, right of the cube respectively
        /// Angles are in radians and are in the range [0, 2pi]
        ///
        /// \param theta_rad : Orientation of cube
        /// \param[out] headings : The headings of cozmo such that it
        ///                        always faces the side of the cube
        void find_headings(
            const double& theta_rad,
            std::vector<double>* headings) const;

        /// Finds Cozmo's start position given an object's pose and edge offsets
        ///
        /// \param obj_pose : (x, y, theta) position of the object
        ///                  we are moving relative to, in (mm, mm, radians)
        /// \param edge_offset : The max distance, along the edge of the object,
        ///                      from the center of that edge (mm)
        /// \param heading : An orientation relative the orientation specified 
        ///                  by obj_pose, (radians)
        /// \param[out] start_pos : Cozmo's calculated initial position,
        ///                         (x, y) in (mm, mm)
        void find_start_pos(
            const Eigen::Vector3d& obj_pose,
            const double& edge_offset,
            const double& heading,
            Eigen::Vector2d* start_pos) const;
};

} // namespace actionspace
} // namespace libcozmo

#endif
