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
        /// An object oriented action consists of 2 parts:
        ///     1) The initial pose (x, y, theta) from
        ///        which the action will be executed
        ///     2) Speed and duration of action
        class Action : public ActionSpace::Action {
            public:
                /// \param speed : the speed of cozmo, in mm / s
                ///                negative speed refers to backward movement
                /// \param duration : the amount of time to move,
                ///                   in seconds, should be >= 0
                /// \param start_pos : Cozmo's inital (x, y, theta) position,
                ///                               in (mm, mm, radians)
                explicit Action (
                    const double& speed,
                    const double& duration,
                    const Eigen::Vector3d& start_pos) : \
                    speed(speed),
                    duration(duration),
                    start_pos(start_pos) {}

                double getSpeed() const { return speed; }

                double getDuration() const { return duration; }

                Eigen::Vector3d getStartPos() const { return start_pos; }

            private:
                double speed;
                double duration;
                Eigen::Vector3d start_pos;

                void update_action(
                    const double& speed,
                    const double& duration,
                    const Eigen::Vector3d& start_pos)
                {
                    this->speed = speed;
                    this->duration = duration;
                    this->start_pos = start_pos;
                }

                friend class ObjectOrientedActionSpace;
        };

        /// \param speeds : generates an action in the action space for each
        ///                 speed (mm/s) in speeds, will generate duplicates
        ///                 if list contains duplicates
        /// \param durations : generates an action in the action space for each
        ///                    duration (seconds) in durations, will generate
        ///                    duplicates if list contains duplicates
        /// \param num_offset : number of starting position offsets on each
        ///                     side of the cube; this value must always be odd
        ObjectOrientedActionSpace(
            const std::vector<double>& speeds,
            const std::vector<double>& durations,
            const int& num_offset) : \
            speeds(speeds),
            durations(durations),
            num_offset(num_offset),
            v_offset(60) {}

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
        /// \param[out] similarty : The similarity value between two actions
        ///                         value of 0 means they are identical
        /// \returns true if successful, false otherwise
        bool action_similarity(
            const int& action_id1,
            const int& action_id2,
            double* similarity) const;

        /// Generates actions given another object's position and theta
        ///
        /// \param obj_pos : An (x, y, theta) coordinate in (mm, mm, radians)
        ///                  theta should be [0, 2pi]
        /// \param edge_offset (optional) : The max horizontal distance from
        ///                              the center of the edge of the object
        ///
        /// This function will overwrite any previously
        /// generated actions in the action space
        void generate_actions(
            const Eigen::Vector3d& obj_pos,
            const double& edge_offset=40);

        ActionSpace::Action* get_action(const int& action_id) const;

        bool is_valid_action_id(const int& action_id) const;

        bool publish_action(
            const int& action_id,
            const ros::Publisher& publisher) const;

        int size() const;

    private:
        std::vector<double> speeds;
        std::vector<double> durations;
        int num_offset;
        double v_offset;
        std::vector<Action*> actions;

        ros::Publisher action_publisher;

        /// Finds the headings for cozmo, relative to a cartesian plane,
        /// to all 4 sides of the cube given the orientation of the cube
        ///
        /// The angles are found in clockwise order where index
        ///     0 corresponds to front of cube
        ///     1 corresponds to left of cube
        ///     2 corresponds to back of cube
        ///     3 corresponds to right of cube
        /// Angles are in radians and are in the range [0, 2pi]
        ///
        /// \param theta_rad : Orientation of cube
        /// \param[out] headings : The headings of cozmo such that it
        ///                        always faces the side of the cube
        void find_headings(
            const double& theta_rad,
            std::vector<double>* headings) const;

        /// Finds Cozmo's start position given an object's position,
        /// theta, and horizontal (cube_offset) distances away from said object
        ///
        /// Horizontal means parallel to the edge of an object
        ///
        /// \param obj_pos : (x, y, theta) position of the object
        ///                  we are moving relative to, in (mm, mm, radians)
        /// \param edge_offset : The horizontal distance away
        ///                      from center of edge of object, in mm
        /// \param heading : An orientation relative to the obj_pos theta,
        ///                  in radians
        /// \param[out] start_pos : Cozmo's calculated initial position,
        ///                         (x, y) in (mm, mm)
        void find_start_pos(
            const Eigen::Vector3d& obj_pos,
            const double& edge_offset,
            const double& heading,
            Eigen::Vector2d* start_pos) const;
};

} // namespace actionspace
} // namespace libcozmo

#endif
