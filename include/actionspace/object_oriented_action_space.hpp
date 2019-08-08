#ifndef OBJECT_ORIENTED_ACTION_SPACE
#define OBJECT_ORIENTED_ACTION_SPACE

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

class ObjectOrientedActionSpace {
    public:
        ObjectOrientedActionSpace();

        void generate_actions(Pose pose, int num_offsets,
                              double lin_min, double lin_max, double lin_samples,
                              double dur_min, double dur_max, double dur_samples,
                              double h_offset=40, double v_offset=60);

        Object_Oriented_Action get_action(int action_id);

        std::vector<Object_Oriented_Action> get_action_space();
        
        void view_action_space();

    private:
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
