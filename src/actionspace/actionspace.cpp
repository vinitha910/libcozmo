#include "actionspace/actionspace.hpp"
#include "utils/utils.hpp"
#include <algorithm>
#include <iostream>
#include <iterator>
using namespace std;

namespace libcozmo {
namespace actionspace {

// Utility function to generate [num] number of choices from [start] to [stop]
// include_zero : True to add zero to choices, False to not
//                This allows for 0 verlocity in either the linear or angular direction
vector<double> create_choices(double start, double stop, int num, bool include_zero)
{
    double step = (stop - start) / (num - 1);
    vector<double> choices = libcozmo::utils::linspace(start, stop, num);

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

GenericActionSpace::GenericActionSpace() {}

void GenericActionSpace::generate_actions(double lin_min, double lin_max, double lin_samples,
                                          double ang_min, double ang_max, double ang_samples,
                                          double dur_min, double dur_max, double dur_samples)
{
    actions.clear();
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

Action GenericActionSpace::get_action(int action_id) {
    return actions[action_id];
}

vector<Action> GenericActionSpace::get_action_space() {
    return actions;
}

void GenericActionSpace::view_action_space() {
    for (size_t i = 0; i < actions.size(); i++) {
        if (i % 5 == 0) {
            cout << "\n";
        }
        cout << i << " : ";
        cout << "Linear Velocity: " << actions[i].lin_vel << ", ";
        cout << "Angular Velocity: " << actions[i].ang_vel << ", ";
        cout << "Duration: " << actions[i].duration << "\n";
    }
}

ObjectOrientedActionSpace::ObjectOrientedActionSpace() {}

void ObjectOrientedActionSpace::generate_actions(Pose pose, int num_offsets,
                                                 double lin_min, double lin_max, double lin_samples,
                                                 double dur_min, double dur_max, double dur_samples,
                                                 double h_offset, double v_offset)
{
    actions.clear();
    vector<Pose> locations = generate_offsets(pose, num_offsets, h_offset, v_offset);
    GenericActionSpace gen_space {};
    gen_space.generate_actions(lin_min, lin_max, lin_samples, 0, 0, 0, dur_min, dur_max, dur_samples);
    vector<Action> gen_actions = gen_space.get_action_space();
    for (const auto& location : locations) {
        for (const auto& action : gen_actions) {
            Object_Oriented_Action ooa{location, action};
            actions.push_back(ooa);
        }
    }
}

Object_Oriented_Action ObjectOrientedActionSpace::get_action(int action_id) {
    return actions[action_id];
}

vector<Object_Oriented_Action> ObjectOrientedActionSpace::get_action_space() {
    return actions;
}

void ObjectOrientedActionSpace::view_action_space() {
    for (size_t i = 0; i < actions.size(); i++) {
        if (i % 5 == 0) {
            cout << "\n";
        }
        Pose p = actions[i].pose;
        Action a = actions[i].action;
        cout << i << " : ";
        cout << "Location: " << p.x << ", " << p.y << "," << p.z << ", " << p.angle_z << ", ";
        cout << "Linear Velocity: " << a.lin_vel << ", ";
        cout << "Angular Velocity: " << a.ang_vel << ", ";
        cout << "Duration: " << a.duration << "\n";
    }
}

Point ObjectOrientedActionSpace::cube_offset(double offset, double angle) {
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
vector<double> ObjectOrientedActionSpace::find_sides(double angle)
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


/*
Helper function to generate cube offset positions

Parameters
----------
h_offset : the max horizontal offset from the center of the edge of the cube, in millimeters
v_offset : the vertical offset away from the center of the cube, in millimeters
*/
vector<Pose> ObjectOrientedActionSpace::generate_offsets(Pose pose, int num_offsets, double h_offset, double v_offset)
{
    vector<double> choices;
    if (num_offsets == 1) {
        choices.push_back(0);
    } else {
        choices = create_choices(-h_offset, h_offset, num_offsets, true);
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
int ObjectOrientedActionSpace::nearest_zero(vector<double> values) {
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

} // namespace actionspace
} // namespace libcozmo

int main() {
    libcozmo::actionspace::GenericActionSpace gen_space {};
    gen_space.generate_actions(10, 100, 5, 10, 100, 5, 1, 5, 5);
    vector<libcozmo::actionspace::Action> gen_actions = gen_space.get_action_space();
    gen_space.view_action_space();
    cout << "Total actions: " << gen_actions.size() << '\n';

    // Angle_z value should be between -pi and pi
    libcozmo::actionspace::Pose pose{100, 200, 10, 2};
    libcozmo::actionspace::ObjectOrientedActionSpace oos {};
    oos.generate_actions(pose, 3, 10, 100, 3, 1, 5, 3);
    vector<libcozmo::actionspace::Object_Oriented_Action> oo_actions = oos.get_action_space();
    oos.view_action_space();
    cout << "Total actions: " << oo_actions.size() << '\n';

}
