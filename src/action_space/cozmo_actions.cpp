#include <algorithm>
#include <cfloat>
#include <iostream>
#include <iterator>
#include <list>
#include <math.h>
#include <numeric>
#include <vector>

struct Action
{
    double lin_vel;
    double ang_vel;
    double duration;
};

struct Pose
{
    double pos_x;
    double pos_y;
    double pos_z;
    double angle_z;
};

struct Object_Oriented_Action
{
    Pose pose;
    Action action;
};

// For generating a sequence of equidistant values
struct double_inc_iterator : std::iterator<std::forward_iterator_tag, double>
{
    double_inc_iterator(double initial, double inc = 1.0) : _value(initial), _inc(inc) {}
    value_type operator*() const { return _value; }
    double_inc_iterator& operator++() { _value += _inc; return *this; }

    bool operator==(double_inc_iterator const& r) const { return _value >= r._value; }
    bool operator!=(double_inc_iterator const& r) const { return !(*this == r); }

    value_type _value;
    value_type _inc;
};

// Utility function to generate [num] number of choices from [start] to [stop]
// include_zero : True to add zero to choices, False to not
//                This allows for 0 verlocity in either the linear or angular direction
std::vector<double> create_choices(double start, double stop, int num, bool include_zero)
{
    double step = (stop - start) / (num - 1);
    std::vector<double> choices(double_inc_iterator(start, step), double_inc_iterator(stop + FLT_EPSILON));
    if (include_zero) {
        bool contains_zero = false;
        if (std::find(choices.begin(), choices.end(), 0) != choices.end()) {
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
        { generate_actions();
        }

        std::list<Action> get_action_space()
        {
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
        std::list<Action> actions;

        void generate_actions()
        {
            std::vector<double> lin_choices = create_choices(lin_min, lin_max, lin_samples, true);
            std::vector<double> ang_choices = create_choices(ang_min, ang_max, ang_samples, true);
            std::vector<double> dur_choices = create_choices(dur_min, dur_max, dur_samples, false);

            for (double dur_choice : dur_choices) {
                for (double lin_choice : lin_choices) {
                    for (double ang_choice : ang_choices) {
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

    private:
        Pose pose{ 0, 0, 0, 0 };
        double samples{ 0 };
        double lin_min{ 0 };
        double lin_max{ 0 };
        double lin_samples{ 0 };
        double dur_min{ 0 };
        double dur_max{ 0 };
        double dur_samples{ 0 };
        std::list<Object_Oriented_Action> actions;

        void generate_actions(double h_offset=40, double v_offset=60)
        {
            generate_offsets(h_offset, v_offset);
        }

        void generate_offsets(double h_offset, double v_offset)
        {
            std::vector<double> choices;
            if (samples == 1) {
                choices.push_back(0);
            } else {
                choices = create_choices(-h_offset, h_offset, samples, true);
            }
            find_sides(pose.angle_z);

        }

        std::vector<double> find_sides(double angle)
        {
            // std::cout << angle;
            std::vector<double> sides;
            sides.push_back(angle);
            for (int i = 0; i < 3; i++) {
                angle -= M_PI / 2;
                std::cout << angle;
            }
            return sides;
        }
};


int main() {
    GenericActionSpace gen_space (10, 100, 5, 10, 100, 5, 1, 5, 5);
    std::list<Action> actions = gen_space.get_action_space();
    for (auto a : actions) {
        std::cout << a.lin_vel << ", " << a.ang_vel << ", " << a.duration << "\n";
    }

    Pose pose{100, 200, 10, 0.5};
    ObjectOrientedActionSpace oos (pose, 3, 10, 100, 3, 1, 5, 3);
}
