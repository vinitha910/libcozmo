//This class creates states used to build graph

class State {
    
    public:
        State(const int& width,
          const int& height,
          const int& theta) : \
          m_width(width),
          m_height(height),
          m_theta(theta) {}

        ~State() {}
    private:
        int m_width, m_height, m_theta;
};