#ifndef GENERIC_ACTION_SPACE
#define GENERIC_ACTION_SPACE

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

    int get_action();

 private:
    const double m_speed;
    const double m_duration;
    const double m_direction;
};

} 
}

#endif