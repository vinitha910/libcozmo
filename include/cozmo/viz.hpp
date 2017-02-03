#ifndef VIZ_VIZ_HPP_
#define VIZ_VIZ_HPP_

#include "cozmo/cozmo.hpp"

namespace visualizer {

class Viz
{
public:
Viz(libcozmo::Cozmo Cozmo, const std::string& mesh_dir, int argc, char* argv[]);
  
};
}
#endif  // VIZ_VIZ_HPP_
