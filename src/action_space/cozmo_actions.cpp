#include <algorithm>
#include <cfloat>
#include <iostream>
#include <iterator>
#include <list>
#include <numeric>
#include <vector>

struct action {
    float lin_vel;
    float ang_vel;
    float duration;
};

// For generating a sequence of equidistant values 
struct double_inc_iterator : std::iterator<std::forward_iterator_tag, double> {
    double_inc_iterator(double initial, double inc = 1.0) : _value(initial), _inc(inc) {}
    value_type operator*() const { return _value; }
    double_inc_iterator& operator++() { _value += _inc; return *this; }

    bool operator==(double_inc_iterator const& r) const { return _value >= r._value; }
    bool operator!=(double_inc_iterator const& r) const { return !(*this == r); }

    value_type _value;
    value_type _inc;
};

class GenericActionSpace {
    public:
        GenericActionSpace(float lin_min, 
                           float lin_max, 
                           float lin_samples, 
                           float ang_min,
                           float ang_max,
                           float ang_samples,
                           float dur_min,
                           float dur_max,
                           float dur_samples)
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

        std::list<action> get_action_space() 
        {
            return actions;
        }

    private:
        float lin_min{ 0 }; 
        float lin_max{ 0 };
        float lin_samples{ 0 };
        float ang_min{ 0 };
        float ang_max{ 0 };
        float ang_samples{ 0 };
        float dur_min{ 0 };
        float dur_max{ 0 };
        float dur_samples{ 0 };
        std::list<action> actions;

        void generate_actions() 
        {
            std::vector<double> lin_choices = create_choices(lin_min, lin_max, lin_samples, true);
            std::vector<double> ang_choices = create_choices(ang_min, ang_max, ang_samples, true);
            std::vector<double> dur_choices = create_choices(dur_min, dur_max, dur_samples, false);
            
            for (double dur_choice : dur_choices) {
                for (double lin_choice : lin_choices) {
                    for (double ang_choice : ang_choices) {
                        if (!(lin_choice == ang_choice && lin_choice == 0) || dur_choice == 0) {
                            action a;
                            a.lin_vel = lin_choice;
                            a.ang_vel = ang_choice;
                            a.duration = dur_choice;
                            actions.push_back(a);
                        }
                    }
                }
            }
    }

    // Helper function to generate [num] number of choices from [start] to [stop]
    // include_zero : True to add zero to choices, False to not
    //                This allows for 0 verlocity in either the linear or angular direction
    std::vector<double> create_choices(double start, double stop, int num, bool include_zero) {
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
};


int main() {
    GenericActionSpace gen_space (10, 100, 5, 10, 100, 5, 1, 5, 5);
    std::list<action> actions = gen_space.get_action_space();
}
