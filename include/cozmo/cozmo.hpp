#ifndef COZMO_COZMO_HPP_
#define COZMO_COZMO_HPP_

#include "dart/dart.hpp"

using BodyNodePtr = dart::dynamics::BodyNodePtr;
using SkeletonPtr = dart::dynamics::SkeletonPtr;
//using WorldPtr = dart::dynamics::WorldPtr;

class Cozmo
{
private:
  SkeletonPtr cozmo;
  BodyNodePtr head;
  BodyNodePtr base;
  BodyNodePtr forklift;
  BodyNodePtr lower_forklift_strut_left;
  BodyNodePtr lower_forklift_strut_right_1;
  BodyNodePtr lower_forklift_strut_right_2;
  BodyNodePtr upper_forklift_strut_left;
  BodyNodePtr upper_forklift_strut_right;
  
  BodyNodePtr makeRootBody(const SkeletonPtr& cozmo, const std::string& name, const std::string& mesh_dir);
  BodyNodePtr addBody(const SkeletonPtr& cozmo, BodyNodePtr parent, const std::string& name, const std::string& mesh_dir,
	              Eigen::Vector3d transformFromParent, Eigen::Vector3d transformFromChild, Eigen::Vector3d color, float initPos);
  SkeletonPtr createCozmo(const std::string& mesh_dir);
  
public:
  Cozmo(const std::string& mesh_dir);
  void setPosition(float pos);
  SkeletonPtr getCozmoSkeleton() { return cozmo; };
};

#endif  // COZMO_COZMO_HPP_
