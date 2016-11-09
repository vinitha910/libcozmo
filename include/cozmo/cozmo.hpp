#ifndef COZMO_COZMO_HPP_
#define COZMO_COZMO_HPP_

#include "dart/dart.hpp"

namespace cozmo {
  using BodyNodePtr = dart::dynamics::BodyNodePtr;
  using SkeletonPtr = dart::dynamics::SkeletonPtr;
  void setGeometry(const BodyNodePtr& bn);
  BodyNodePtr makeRootBody(const SkeletonPtr& cozmo, const std::string& name);
  SkeletonPtr createCubeCozmo();
}

#endif  // COZMO_COZMO_HPP_
