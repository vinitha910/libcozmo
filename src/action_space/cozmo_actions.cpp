#include <algorithm>
#include <cfloat>
#include <iostream>
#include <iterator>
#include <list>
#include <math.h>
#include <numeric>
#include <vector>
using namespace std;

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

// Linear interpolation following MATLAB linspace
vector<double> generate_samples(double min, double max, size_t N) {
    double step = (max - min) / static_cast<double>(N - 1);
    vector<double> samples(N);
    vector<double>::iterator i;
    double val;
    for (i = samples.begin(), val = min; i != samples.end(); ++i, val += step) {
        *i = val;
    }
    return samples;
}

// Utility function to generate [num] number of choices from [start] to [stop]
// include_zero : True to add zero to choices, False to not
//                This allows for 0 verlocity in either the linear or angular direction
vector<double> create_choices(double start, double stop, int num, bool include_zero)
{
    double step = (stop - start) / (num - 1);
    vector<double> choices = generate_samples(start, stop, num);
    
    if (include_zero) {
        bool contains_zero = false;
        if (find(choices.begin(), choices.end(), 0) != choices.end()) {
            contains_zero = true;
        }
        if (!contains_zero) {
            choices.insert(choices.begin(), 0);
        }
    }
    return choices;
}

class GenericActionSpace
{
    public:
        GenericActionSpace(double lin_min,
                           double lin_max,
                           double lin_samples,
                           double ang_min,
                           double ang_max,
                           double ang_samples,
                           double dur_min,
                           double dur_max,
                           double dur_samples)
        : lin_min(lin_min),
          lin_max(lin_max),
          lin_samples(lin_samples),
          ang_min(ang_min),
          ang_max(ang_max),
          ang_samples(ang_samples),
          dur_min(dur_min),
          dur_max(dur_max),
          dur_samples(dur_samples)
        { generate_actions(); }

        list<Action> get_action_space() {
            return actions;
        }

    private:
        double lin_min{ 0 };
        double lin_max{ 0 };
        double lin_samples{ 0 };
        double ang_min{ 0 };
        double ang_max{ 0 };
        double ang_samples{ 0 };
        double dur_min{ 0 };
        double dur_max{ 0 };
        double dur_samples{ 0 };
        list<Action> actions;

        void generate_actions() {
            vector<double> lin_choices = create_choices(lin_min, lin_max, lin_samples, true);
            vector<double> ang_choices = create_choices(ang_min, ang_max, ang_samples, true);
            vector<double> dur_choices = create_choices(dur_min, dur_max, dur_samples, false);

            for (const auto& dur_choice : dur_choices) {
                for (const auto& lin_choice : lin_choices) {
                    for (const auto& ang_choice : ang_choices) {
                        if (!(lin_choice == ang_choice && lin_choice == 0) || dur_choice == 0) {
                            Action a {lin_choice, ang_choice, dur_choice};
                            actions.push_back(a);
                        }
                    }
                }
            }
        }
};

class ObjectOrientedActionSpace
{
    public:
        ObjectOrientedActionSpace(Pose pose,
                                  int samples,
                                  double lin_min,
                                  double lin_max,
                                  double lin_samples,
                                  double dur_min,
                                  double dur_max,
                                  double dur_samples)
        : pose(pose),
          samples(samples),
          lin_min(lin_min),
          lin_max(lin_max),
          lin_samples(lin_samples),
          dur_min(dur_min),
          dur_max(dur_max),
          dur_samples(dur_samples)
        { generate_actions();
        }
        
        list<Object_Oriented_Action> get_action_space() {
            return actions;
        }

    private:
        Pose pose{ 0, 0, 0, 0 };
        double samples{ 0 };
        double lin_min{ 0 };
        double lin_max{ 0 };
        double lin_samples{ 0 };
        double dur_min{ 0 };
        double dur_max{ 0 };
        double dur_samples{ 0 };
        list<Object_Oriented_Action> actions;

        Point cube_offset(double offset, double angle) {
            return Point{offset * cos(angle), offset * sin(angle)};
        }
        
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
        vector<double> find_sides(double angle)
        {
            vector<double> sides;
            vector<double> ordered_sides;
            
            sides.push_back(angle);
            for (size_t i = 0; i < 3; i++) {
                // Adding in clockwise order
                angle -= M_PI / 2;
                if (angle < -M_PI) {
                    angle = 2 * M_PI + angle;
                }
                sides.push_back(angle);
            }
            
            int front_idx = nearest_zero(sides);
            ordered_sides.push_back(sides[front_idx]);
            int idx = (front_idx + 1) % 4;
            while (idx != front_idx) {
                ordered_sides.push_back(sides[idx]);
                idx = (idx + 1) % 4;
            }
            return ordered_sides;
        }
        

        void generate_actions(double h_offset=40, double v_offset=60)
        {
            vector<Pose> locations = generate_offsets(h_offset, v_offset);
            GenericActionSpace gen_space (lin_min, lin_max, lin_samples, 0, 0, 0, dur_min, dur_max, dur_samples);
            list<Action> gen_actions = gen_space.get_action_space();
            for (const auto& location : locations) {
                for (const auto& action : gen_actions) {
                    Object_Oriented_Action ooa{location, action};
                    actions.push_back(ooa);
                }
            }
        }

        /*
        Helper function to generate cube offset positions

        Parameters
        ----------
        h_offset : the max horizontal offset from the center of the edge of the cube, in millimeters
        v_offset : the vertical offset away from the center of the cube, in millimeters
        */
        vector<Pose> generate_offsets(double h_offset, double v_offset)
        {
            vector<double> choices;
            if (samples == 1) {
                choices.push_back(0);
            } else {
                choices = create_choices(-h_offset, h_offset, samples, true);
            }
            vector<double> cube_sides = find_sides(pose.angle_z);
            vector<Pose> locations;
            for (size_t i = 0; i < cube_sides.size(); i++) {
                double side = cube_sides[i];
                for (const auto& choice : choices) {
                    Point offset = cube_offset(v_offset, side);
                    Pose location;
                    if (i == 0) { // front of cube
                        location = Pose{pose.x - offset.x, pose.y - offset.y - choice, pose.z, side};
                    } else if (i == 1) { // left of cube
                        location = Pose{pose.x - offset.x - choice, pose.y - offset.y, pose.z, side};
                    } else if (i == 2) { // back of cube
                        location = Pose{pose.x - offset.x, pose.y + offset.y + choice, pose.z, side};
                    } else { // right of cube 
                        location = Pose{pose.x + offset.x + choice, pose.y - offset.y, pose.z, side};
                    }
                    locations.push_back(location);
                }
            }
            return locations;
        }

        /** Helper function to find the value closest to zero in a list,
            used in find_sides to identify which angle of the cube
            is the front

            Note: in case the corner of the cube is perfectly in align with cozmo
            and there is no closest side, we choose the right side to be the front

            returns the index of the value closest to zero
        */
        int nearest_zero(vector<double> values) {
            int nearest = 0;
            for (size_t i = 1; i < values.size(); i++) {
                if (abs(values[i]) < abs(values[nearest])) {
                    nearest = i;
                }
                if (values[i] == M_PI / 2) {
                    return i;
                }
            }
            return nearest;
        }
};


int main() {
    GenericActionSpace gen_space (10, 100, 5, 10, 100, 5, 1, 5, 5);
    list<Action> gen_actions = gen_space.get_action_space();

    // for (const auto& action : actions) {
    //     cout << "Linear Velocity: " << action.lin_vel << " Angular Velocity: " << action.ang_vel << " Duration: " << action.duration << '\n';
    // }
    cout << "Total actions: " << gen_actions.size() << '\n';

    Pose pose{100, 200, 10, 2};
    ObjectOrientedActionSpace oos (pose, 3, 10, 100, 3, 1, 5, 3);
    list<Object_Oriented_Action> oo_actions = oos.get_action_space();

    cout << "Total actions: " << oo_actions.size() << '\n';

}
