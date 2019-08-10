#ifndef OBJECT_ORIENTED_ACTION_SPACE
#define OBJECT_ORIENTED_ACTION_SPACE

#include <Eigen/Dense>
#include "utils/utils.hpp"
#include <vector>

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
            for (size_t i = 0; i < actions.size(); ++i) {
                delete(actions[i]);
            }
            actions.clear();
        }

        void generate_actions(
            const Eigen::Vector2d& obj_pos,
            const double& theta,
            const double& h_offset=40,
            const double& v_offset=60);

        ObjectOrientedAction* get_action(const int& action_id) const;

        void view_action_space() const;

    private:
        std::vector<double> speeds;
        std::vector<double> durations;
        int num_offset;
        std::vector<ObjectOrientedAction*> actions;

        void find_sides(std::vector<double>& cube_sides, const double& theta) const;

        void find_start_pos(
            const Eigen::Vector2d& obj_pos,
            Eigen::Vector2d& start_pos,
            const double& v_offset,
            const double& cube_offset,
            const double& theta) const;
};

} // namespace actionspace
} // namespace libcozmo

#endif
