#include "dart/dart.hpp"
#include "cozmo.hpp"

using namespace cozmo;

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
  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0.07034 / 2.0);
  box_tf.translation() = center;
  shapeNode->setRelativeTransform(box_tf);

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);
}

BodyNode* makeRootBody(const SkeletonPtr& cozmo, const std::string& name)
{
  BallJoint::Properties properties;
  properties.mName = name + "_joint";
  //properties.mRestPositions = Eigen::Vector3d::Constant(default_rest_position);
  //properties.mSpringStiffnesses = Eigen::Vector3d::Constant(default_stiffness);
  //properties.mDampingCoefficients = Eigen::Vector3d::Constant(default_damping);

  BodyNodePtr bn = cozmo->createJointAndBodyNodePair<BallJoint>(
        nullptr, properties, BodyNode::AspectProperties(name)).second;

  // Make a shape for the Joint
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(0.0573, 0.0988, 0.07034)));
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the geometry of the Body
  setGeometry(bn);

  return bn;
}

SkeletonPtr* createCubeCozmo()
{
  SkeletonPtr cozmo = Skeleton::create("cozmo");
  BodyNode* bn = makeRootBody(cozmo, "body");

  WorldPtr world(new World);
  world->addSkeleton(cozmo);

  MyWindow window(world);
  window.initWindow(640, 480, "Cube Cozmo");

  return cozmo;
}
