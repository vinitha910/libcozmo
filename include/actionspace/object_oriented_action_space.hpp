#ifndef OBJECT_ORIENTED_ACTION_SPACE
#define OBJECT_ORIENTED_ACTION_SPACE

#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include "utils/utils.hpp"
#include <libcozmo/OOAction.h>

namespace libcozmo {
namespace actionspace {

// An object oriented action relative to some object, c
class ObjectOrientedAction {
    public:
        // speed : in millimeters / s
        // duration : in seconds
        // obj_pos : x, y coordinate of c in the world, in millimeters
        // offset : to allow another object, c2, to get near c without touching
        //          x, y represents horizontal and vertical offsets from center of c,
        //          in millimeters
        // theta : the orientation of c, in radians
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

        // Calculates and returns similarity with another action
        // other_action : the other action to compare to
        double action_similarity(ObjectOrientedAction& other_action) const;

        const double speed;
        const double duration;
        const Eigen::Vector2d start_pos;
        const double theta;
};

class ObjectOrientedActionSpace {
    public:
        /**
            Speed can be any real number, in millimeters / s
            negative speed refers to backward movement

            Duration should be >= 0, in seconds

            If num_offset is even, there will be no
            center position for edges of the object
        */
        ObjectOrientedActionSpace(
            const double& min_speed,
            const double& max_speed,
            const int& num_speed,
            const double& min_duration,
            const double& max_duration,
            const int& num_duration,
            const int& num_offset) : \
            num_offset(num_offset)
        {
            speeds = utils::linspace(min_speed, max_speed, num_speed);
            durations = utils::linspace(min_duration, max_duration, num_duration);
        }

        ~ObjectOrientedActionSpace() {
            clear_actions();
        }

        /**
            Calculates the similarity between two actions in the action space

            Similarity is defined by the euclidean distance
            between all attributes of an ObjectOrientedAction
        */
        double action_similarity(
            const int& action_id1,
            const int& action_id2) const;

        /**
            Generates actions given another object's position and theta

            Parameters
            ----------
            obj_pos is an (x, y) coordinate in millimeters
            theta is an orientation angle in radians
            h_offset (optional) is the max horizontal distance
                from center of the edge of the object
            v_offset (optional) is the max vertical distance
                from center of the object

            Horizontal means parallel to the edge of an object
            Vertical means perpendicular to the edge of an object
        */
        void generate_actions(
            const Eigen::Vector2d& obj_pos,
            const double& theta,
            const double& h_offset=40,
            const double& v_offset=60);

        ObjectOrientedAction* get_action(const int& action_id) const;

        int get_action_space_size() const;

        void publish_action(const int& action_id) const;

        // Outputs all the actions in the action space with their corresponding action id
        void view_action_space() const;

    private:
        std::vector<double> speeds;
        std::vector<double> durations;
        int num_offset;
        std::vector<ObjectOrientedAction*> actions;

        // Cozmo ROS node handle
        ros::NodeHandle cozmo_handle;
        ros::Publisher action_publisher;

        void clear_actions();

        void find_sides(std::vector<double>& cube_sides, const double& theta) const;

        void find_start_pos(
            const Eigen::Vector2d& obj_pos,
            Eigen::Vector2d& start_pos,
            const double& v_offset,
            const double& cube_offset,
            const double& theta) const;

        bool is_valid_action_id(const int& action_id) const;
};

} // namespace actionspace
} // namespace libcozmo

#endif
