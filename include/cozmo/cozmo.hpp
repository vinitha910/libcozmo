#ifndef COZMO_COZMO_HPP_
#define COZMO_COZMO_HPP_

#include "dart/dart.hpp"

namespace cozmo {
  using BodyNodePtr = dart::dynamics::BodyNodePtr;
  using SkeletonPtr = dart::dynamics::SkeletonPtr;
  using WorldPtr = dart::simulation::WorldPtr;
  
  void setGeometry(const BodyNodePtr& bn);
  BodyNodePtr makeRootBody(const SkeletonPtr& cozmo, const std::string& name);
  WorldPtr createCozmo(const std::string& mesh_dir);
}

#endif  // COZMO_COZMO_HPP_
