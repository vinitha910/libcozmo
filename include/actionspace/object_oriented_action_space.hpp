#ifndef OBJECT_ORIENTED_ACTION_SPACE
#define OBJECT_ORIENTED_ACTION_SPACE

#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include "utils/utils.hpp"
#include <libcozmo/ObjectOrientedAction.h>

namespace libcozmo {
namespace actionspace {

/// An action space class that generates possible
/// actions for Cozmo to execute relative to an object

/// An Object Oriented Action Space is defined by
/// the speed and duration of Cozmo at a location relative
/// to an object as well as the number of offset positions
/// that Cozmo can move to relative to an object

/// \function action_similarity : finds the similarity between two actions in the action space
/// \function generate_actions : generates possible actions for Cozmo given
///                              another object's position and orientation
/// \function get_action : gets a specific action from the actionspace given an action id
/// \function publish_action : publishes an action msg using a ROS publisher
/// \function size : gets the size of the action space
/// \function view_action_space : prints out the every action in the action space with
///                               their corresponding action id
class ObjectOrientedActionSpace {
    public:
        /// An object oriented action consists of 2 parts:
        ///     1) The initial position and initial heading in the world to go to
        ///     2) Move at a certain speed for a certain duration once at the starting position
        class ObjectOrientedAction {
            public:
                /// \param speed : the speed of cozmo in part 2), in mm / s
                ///     negative speed refers to backward movement
                /// \param duration : the amount of time to move in part 2), in seconds
                ///     should be >= 0
                /// \param start_pos, Cozmo's inital (x, y) position, in mm
                /// \param theta, Cozmo's initial heading, in radians
                ObjectOrientedAction (
                    const double& speed,
                    const double& duration,
                    const Eigen::Vector2d& start_pos,
                    const double& theta) : \
                    speed(speed),
                    duration(duration),
                    start_pos(start_pos),
                    theta(theta) {}
                ~ObjectOrientedAction() {}

                const double speed;
                const double duration;
                const Eigen::Vector2d start_pos;
                const double theta;
        };

        /// \param speeds, generates an action in the action space for each speed (mm/s)
        ///                in speeds, will generate duplicates if list constains duplicates
        ///                Will generate duplicates if list contains duplicate values
        /// \param speeds, generates an action in the action space for each duration (seconds)
        ///                in duration, will generate duplicates if list contains duplicates
        /// \param num_offset, number of horizontal offsets for each side of an object
        ///                    num_offsets must be odd and always includes the center of an edge
        /// \param v_offset (optional), minimum distance away from center of object
        ObjectOrientedActionSpace(
            const std::vector<double>& speeds,
            const std::vector<double>& durations,
            const int& num_offset,
            const double& v_offset=60) : \
            speeds(speeds),
            durations(durations),
            num_offset(num_offset),
            v_offset(v_offset) {}

        ~ObjectOrientedActionSpace() {
            clear_actions();
        }

        /// Calculates the similarity between two actions in the action space
        /// Similarity is defined by the euclidean distance
        /// between all attributes of an ObjectOrientedAction

        /// \param action_id1, action_id2 : The action id of two actions to compare
        /// \throws out_of_range exception if either id's are invalid
        /// \returns a value of how close the two actions are,
        ///          value of 0 means they are identical
        double action_similarity(
            const int& action_id1,
            const int& action_id2) const;

        /// Generates actions given another object's position and theta

        /// \param obj_pos is an (x, y) coordinate in millimeters
        /// \param theta is an orientation angle in radians, should be [-pi, pi]
        /// \param h_offset (optional) is the max horizontal distance
        ///            from center of the edge of the object
        /// \param v_offset (optional) is the max vertical distance
        ///            from center of the object

        /// Horizontal means parallel to the edge of an object
        /// Vertical means perpendicular to the edge of an object

        /// This function will clear any previously generated actions in the action space

        void generate_actions(
            const Eigen::Vector2d& obj_pos,
            const double& theta,
            const double& h_offset=40);

        ObjectOrientedAction* get_action(const int& action_id) const;

        void publish_action(const int& action_id, const ros::Publisher& publisher) const;

        int size() const;

        /// Outputs all the actions in the action space with their corresponding action id
        void view_action_space() const;

    private:
        std::vector<double> speeds;
        std::vector<double> durations;
        int num_offset;
        double v_offset;
        std::vector<ObjectOrientedAction*> actions;

        ros::Publisher action_publisher;

        void clear_actions();

        /// Finds the headings for cozmo, relative to a cartesian plane, to all
        /// 4 sides of the cube given the angle of one of the sides of the cube

        /// The angles are found in clockwise order where index
        ///     0 corresponds to front of cube
        ///     1 corresponds to left of cube
        ///     2 corresponds to back of cube
        ///     3 corresponds to right of cube
        /// Angles are in radians and are in the range [0, 2pi]

        /// \param headings, a place to store the headings found
        /// \param theta, angle of one of the sides of the cube
        void find_headings(std::vector<double>* headings, const double& theta) const;

        /// Finds the start position given some other object's position,
        /// its theta, and horizontal (cube_offset) distances away from said object

        /// Horizontal means parallel to the edge of an object

        /// \param obj_pos, the (x, y) position of the object we are moving relative to
        /// \param start_pos, a place to store cozmo's calculated initial position
        /// \param cube_offset, the horizontal distance away from center of edge of object
        /// \param theta, the orientation of the object
        void find_start_pos(
            const Eigen::Vector2d& obj_pos,
            Eigen::Vector2d* start_pos,
            const double& cube_offset,
            const double& theta) const;

        bool is_valid_action_id(const int& action_id) const;
};

} // namespace actionspace
} // namespace libcozmo

#endif
