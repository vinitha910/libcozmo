#include "cozmo/cozmo.hpp"
#include "Eigen/Dense"

namespace cozmo{
using BoxShape = dart::dynamics::BoxShape;
using FreeJoint = dart::dynamics::FreeJoint;
using VisualAspect = dart::dynamics::VisualAspect;
using CollisionAspect = dart::dynamics::CollisionAspect;
using DynamicsAspect = dart::dynamics::DynamicsAspect;
using Skeleton = dart::dynamics::Skeleton; 

void setGeometry(const BodyNodePtr& bn)
{
  // Create a BoxShape to be used for both visualization and collision checking
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(0.0573, 0.0988, 0.07034)));

  // Create a shape node for visualization and collision checking
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Red());

  // Set the location of the shape node
  //Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  //Eigen::Vector3d center = Eigen::Vector3d();
  //box_tf.translation() = center;
  //shapeNode->setRelativeTransform(box_tf);

  // Move the center of mass to the center of the object
  //bn->setLocalCOM(center);
}

BodyNodePtr makeRootBody(const SkeletonPtr& cozmo, const std::string& name)
{
  FreeJoint::Properties properties;
  properties.mName = name + "_joint";
  //properties.mRestPositions = Eigen::Vector3d::Constant(default_rest_position);
  //properties.mSpringStiffnesses = Eigen::Vector3d::Constant(default_stiffness);
  //properties.mDampingCoefficients = Eigen::Vector3d::Constant(default_damping);

  BodyNodePtr bn = cozmo->createJointAndBodyNodePair<FreeJoint>(
			  nullptr, properties, 
			  dart::dynamics::BodyNode::AspectProperties(name)).second;

  // Make a shape for the Joint
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(0.0573, 0.0988, 0.07034)));
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the geometry of the Body
  //setGeometry(bn);

  return bn;
}

SkeletonPtr createCubeCozmo()
{
  SkeletonPtr cozmo = Skeleton::create("cozmo");
  BodyNodePtr bn = makeRootBody(cozmo, "body");

  return cozmo;
}
}
