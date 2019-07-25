#include <list>

struct action {
  float lin_vel;
  float ang_vel;
  float duration;
}

class GenericActionSpace {
  public:
    GenericActionSpace(float lin_min, float lin_max, float lin_samples, float ang_min, float ang_max, float ang_samples, float dur_min, float dur_max, float dur_samples)
      : lin_min(lin_min),
        lin_max(lin_max),
        lin_samples(lin_samples),
        ang_min(ang_min),
        ang_max(ang_max),
        ang_samples(ang_samples),
        dur_min(dur_min),
        dur_max(dur_max),
        dur_samples(dur_samples)
    {   generate_actions();
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

    void generate_actions() {
      list<action> actions;
      actions.push_back(act)
    }
};
