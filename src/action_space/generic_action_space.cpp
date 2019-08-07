#include <action_space/generic_action_space.hpp>

namespace libcozmo {
namespace actionspace {

double GenericAction::action_similarity(GenericAction& other_action) const {
    return sqrt(
        pow((m_speed - other_action.m_speed), 2) + 
        pow((m_duration - other_action.m_duration), 2) + 
        pow((m_direction.x() - other_action.m_direction.x()), 2) +
        pow((m_direction.y() - other_action.m_direction.y()), 2));
}

GenericAction* GenericActionSpace::get_action(const int& action_id) const {
    return m_actions[action_id];
}
}
}