namespace libcozmo {
namespace statespace {

// Custom Hash function for a discretized state
class StateHasher {
  public:
    std::size_t operator()(const Eigen::Vector3i& state) const {
        using boost::hash_value;
        using boost::hash_combine;
        std::size_t seed = 0;
        hash_combine(seed, hash_value(state.x()));
        hash_combine(seed, hash_value(state.y()));
        hash_combine(seed, hash_value(state.z()));
        return seed;
    }
};

}  // namespace statespace
}  // namespace libcozmo