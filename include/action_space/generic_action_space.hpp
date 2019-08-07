#ifndef GENERIC_ACTION_SPACE
#define GENERIC_ACTION_SPACE

#include "utils/utils.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace libcozmo {
namespace actionspace {

class GenericAction {
  public:

    // Constructor

    // \param speed The speed of action
    // \param duration The duration of action
    // \param direction The direction of action
    GenericAction (
        const double& speed,
        const double& duration,
        const double& direction) : \
        m_speed(speed),
        m_duration(duration),
        m_direction(direction) {}
    ~GenericAction() {}
    
    // Calculates and returns similarity with another action

    // \param other_action The other action to compare to
    double action_similarity(GenericAction& other_action) const;

    const double m_speed;
    const double m_duration;
    const double m_direction;
};

// Actionspace for Cozmo
class GenericActionSpace {
  public:
    GenericActionSpace(
        const double& min_speed,
        const double& max_speed,
        const double& min_duration,
        const double& max_duration,
        const int& num_vals) : \
        m_min_speed(min_speed),
        m_max_speed(max_speed),
        m_min_duration(min_duration),
        m_max_duration(max_duration),
        m_num_vals(num_vals) {
            std::vector<double> speeds = utils::linspace(m_min_speed, m_max_speed, static_cast<size_t>(m_num_vals));
            std::vector<double> durations = utils::linspace(m_min_duration, m_max_duration, m_num_vals);
            std::vector<Eigen::Vector2d> directions;
            for i in range(int i = 0; i < m_num_vals; i++) {
              double angle = i * M_PI / static_cast<double>(m_num_vals);
              directions.push_back(Eigen::Vector2d(acos(angle), asin(angle)));
            }
            
            // for (int j = 0; j < m_num_vals; j++) {
            //     for (int k = 0; k < m_num_vals; k++) {
            //         m_actions.push_back(GenericAction())
            //     }
            // }
        
        }
    ~GenericActionSpace() {}

    int get_action();



  private:
    const double m_min_speed;
    const double m_max_speed;
    const double m_min_duration;
    const double m_max_duration;
    // number of "bins" to assign linspace out of 
    const int m_num_vals;
    std::vector<Action> m_actions;

};

} 
}

#endif