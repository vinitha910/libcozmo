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
        const Eigen::Vector2d& direction) : \
        m_speed(speed),
        m_duration(duration),
        m_direction(direction) {}
    ~GenericAction() {}
    
    // Calculates and returns similarity with another action

    // \param other_action The other action to compare to
    double action_similarity(GenericAction& other_action) const;

    const double m_speed;
    const double m_duration;
    const Eigen::Vector2d m_direction;
};

// Actionspace for Cozmo
class GenericActionSpace {
  public:

    // Constructor
    
    // \param min_speed The minimum speed of action
    // \param max_speed The maximum speed of action
    // \param min_duration The minimum duration of action
    // \param max_duration The maximum duration of action
    // \param num_vals The number of values for action linspace
    // \param num_theta The number of theta values
    GenericActionSpace(
        const double& min_speed,
        const double& max_speed,
        const double& min_duration,
        const double& max_duration,
        const int& num_vals,
        const int& num_theta) : \
        m_num_vals(num_vals),
        m_num_theta(num_theta) {
            m_speeds = utils::linspace(min_speed, max_speed, static_cast<size_t>(m_num_vals));
            m_durations = utils::linspace(min_duration, max_duration, m_num_vals);
            for (int i = 0; i < m_num_theta; i++) {
              double angle = i * M_PI / static_cast<double>(m_num_theta);
              m_directions.push_back(Eigen::Vector2d(acos(angle), asin(angle)));
            }
            
            for (int j = 0; j < m_num_vals; j++) {
                for (int k = 0; k < m_num_vals; k++) {
                  for (int l = 0; l < m_num_theta; l++) {
                    m_actions.push_back(GenericAction(
                      m_speeds[j], 
                      m_durations[k], 
                      m_directions[l]));
                  } 
                }
            }
        }
    ~GenericActionSpace() {}

    // Assigns pointer to action given action ID

    // \param action_id The action ID
    // \param action Pointer for chosen action
    void get_action(int& action_id, GenericAction* action);

    // Executes action given action ID

    // \param action_id The action ID
    void execute_action(int& action_id) const;

  private:
    std::vector<double> m_speeds;
    std::vector<double> m_durations;
    // number of "bins" to assign linspace out of 
    const int m_num_vals;
    const int m_num_theta;
    std::vector<GenericAction> m_actions;
    std::vector<Eigen::Vector2d> m_directions;

};

} 
}

#endif