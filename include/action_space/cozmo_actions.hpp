#ifndef _INCLUDE_COZMO_ACTIONS_HPP
#define _INCLUDE_COZMO_ACTIONS_HPP

// #include <vector>
#include "utils/utils.hpp"
#include <Eigen/dense>
#include <cmath>

namespace libcozmo{
namespace actionspace{

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
            std::vector<double> speeds = utils::linspace(m_min_speed, m_max_speed, m_num_vals);
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



  private:
    const double m_min_speed;
    const double m_max_speed;
    const double m_min_duration;
    const double m_max_duration;
    // number of "bins" to assign linspace out of 
    const int m_num_vals;
    std::Vector<Action> m_actions;

};




// Linear interpolation following MATLAB linspace
std::vector<double> generate_samples(double min, double max, std::size_t N);

// Utility function to generate [num] number of choices from [start] to [stop]
// include_zero : True to add zero to choices, False to not
//                This allows for 0 verlocity in either the linear or angular direction
std::vector<double> create_choices(double start, double stop, int num, bool include_zero);

class GenericActionSpace {
    public:
        GenericActionSpace();

        void generate_actions(double lin_min, double lin_max, double lin_samples,
                              double ang_min, double ang_max, double ang_samples,
                              double dur_min, double dur_max, double dur_samples);

        Action get_action(int action_id);

        std::vector<Action> get_action_space();

        void view_action_space();
    private:
        std::vector<Action> actions;

};

// class ObjectOrientedActionSpace {
//     public:
//         ObjectOrientedActionSpace();

//         void generate_actions(Pose pose, int num_offsets,
//                               double lin_min, double lin_max, double lin_samples,
//                               double dur_min, double dur_max, double dur_samples,
//                               double h_offset=40, double v_offset=60);

//         Object_Oriented_Action get_action(int action_id);

//         std::vector<Object_Oriented_Action> get_action_space();
        
//         void view_action_space();

//     private:
//         std::vector<Object_Oriented_Action> actions;

//         Point cube_offset(double offset, double angle);
        
//         /*
//         Helper function to find the location of all 4 sides of the cube

//         Parameters
//         ----------
//         angle : the angle of the cube, in radians
        
//         returns a sorted list of the angle of each of the 4 sides where index
//             0 corresponds to front of cube
//             1 corresponds to left of cube
//             2 corresponds to back of cube
//             3 corresponds to right of cube
//         */
//         std::vector<double> find_sides(double angle);

//         /*
//         Helper function to generate cube offset positions

//         Parameters
//         ----------
//         h_offset : the max horizontal offset from the center of the edge of the cube, in millimeters
//         v_offset : the vertical offset away from the center of the cube, in millimeters
//         */
//         std::vector<Pose> generate_offsets(Pose pose, int num_offsets, double h_offset, double v_offset);

//         /** Helper function to find the value closest to zero in a list,
//             used in find_sides to identify which angle of the cube
//             is the front

//             Note: in case the corner of the cube is perfectly in align with cozmo
//             and there is no closest side, we choose the right side to be the front

//             returns the index of the value closest to zero
//         */
//         int nearest_zero(std::vector<double> values);
// };

} //namespace actionspace
} //namespace libcozmo

#endif
