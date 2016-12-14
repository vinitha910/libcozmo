#ifndef COZMO_COZMO_HPP_
#define COZMO_COZMO_HPP_

#include "dart/dart.hpp"

using BodyNodePtr = dart::dynamics::BodyNodePtr;
using SkeletonPtr = dart::dynamics::SkeletonPtr;
using InverseKinematicsPtr = dart::dynamics::InverseKinematicsPtr;

class Cozmo
{
private:
  SkeletonPtr cozmo;
  BodyNodePtr head;
  BodyNodePtr base;
  BodyNodePtr forklift;
  BodyNodePtr ghost_strut;
  BodyNodePtr lower_forklift_strut_left;
  BodyNodePtr lower_forklift_strut_right;
  BodyNodePtr upper_forklift_strut_left;
  BodyNodePtr upper_forklift_strut_right;
  InverseKinematicsPtr ik;
  
  BodyNodePtr makeRootBody(const SkeletonPtr& cozmo, const std::string& name, const std::string& mesh_dir);
  BodyNodePtr addBody(const SkeletonPtr& cozmo, BodyNodePtr parent, const std::string& name, const std::string& mesh_dir,
	              Eigen::Vector3d transformFromParent, Eigen::Vector3d transformFromChild);
  SkeletonPtr createCozmo(const std::string& mesh_dir);
  void createIKModule();
  
public:
  Cozmo(const std::string& mesh_dir);
  void setPosition(double pos);
  SkeletonPtr getCozmoSkeleton() { return cozmo; };
};

#endif  // COZMO_COZMO_HPP_
