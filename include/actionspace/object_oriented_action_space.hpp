#ifndef OBJECT_ORIENTED_ACTION_SPACE
#define OBJECT_ORIENTED_ACTION_SPACE

#include <Eigen/Dense>
#include <vector>

namespace libcozmo {
namespace actionspace {

struct Action
{
    double lin_vel;
    double ang_vel;
    double duration;
};

struct Pose
{
    double x;
    double y;
    double z;
    double angle_z;
};

struct Point
{
    double x;
    double y;
};

struct Object_Oriented_Action
{
    Pose pose;
    Action action;
};

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
            const Eigen::Vector2d& obj_pos,
            const Eigen::Vector2d& offset,
            const double& theta) : \
            speed(speed),
            duration(duration),
            obj_pos(obj_pos),
            offset(offset),
            theta(theta) {}
        ~ObjectOrientedAction() {}

        // Calculates and returns similarity with another action
        // other_action : the other action to compare to
        double action_similarity(ObjectOrientedAction& other_action) const;

        const double speed;
        const double duration;
        const Eigen::Vector2d obj_pos;
        const Eigen::Vector2d offset;
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
            const int& num_duration) 
        {
            speeds = utils::linspace(min_speed, max_speed, num_speed);
            durations = utils::linspace(min_duration, max_duration, num_duration);
        }
        
        // Generate_actions(vector2d obj_pos, vector2d offset, double theta)

        void generate_actions(Pose pose, int num_offsets,
                              double lin_min, double lin_max, double lin_samples,
                              double dur_min, double dur_max, double dur_samples,
                              double h_offset=40, double v_offset=60);

        Object_Oriented_Action get_action(int action_id);

        std::vector<Object_Oriented_Action> get_action_space();
        
        void view_action_space();

    private:
        std::vector<double> speeds;
        std::vector<double> durations;

        std::vector<Object_Oriented_Action> actions;

        Point cube_offset(double offset, double angle);
        
        /*
        Helper function to find the location of all 4 sides of the cube

        Parameters
        ----------
        angle : the angle of the cube, in radians
        
        returns a sorted list of the angle of each of the 4 sides where index
            0 corresponds to front of cube
            1 corresponds to left of cube
            2 corresponds to back of cube
            3 corresponds to right of cube
        */
        std::vector<double> find_sides(double angle);

        /*
        Helper function to generate cube offset positions

        Parameters
        ----------
        h_offset : the max horizontal offset from the center of the edge of the cube, in millimeters
        v_offset : the vertical offset away from the center of the cube, in millimeters
        */
        std::vector<Pose> generate_offsets(Pose pose, int num_offsets, double h_offset, double v_offset);

};

} // namespace actionspace
} // namespace libcozmo

#endif
