#include "cozmo/cozmo.hpp"
#include "Eigen/Dense"
#include <cmath>

namespace cozmo{
using BoxShape = dart::dynamics::BoxShape;
using MeshShape = dart::dynamics::MeshShape;
using FreeJoint = dart::dynamics::FreeJoint;
using Joint = dart::dynamics::Joint;
using RevoluteJoint = dart::dynamics::RevoluteJoint;
using VisualAspect = dart::dynamics::VisualAspect;
using CollisionAspect = dart::dynamics::CollisionAspect;
using DynamicsAspect = dart::dynamics::DynamicsAspect;
using Skeleton = dart::dynamics::Skeleton; 
using FixedFrame = dart::dynamics::FixedFrame;  
  //using WorldPtr = dart::simulation::WorldPtr;
using World = dart::simulation::World;
using WeldJointConstraint = dart::constraint::WeldJointConstraint;
  
BodyNodePtr makeRootBody(const SkeletonPtr& cozmo, const std::string& name)
{
  FreeJoint::Properties properties;

  BodyNodePtr bn = cozmo->createJointAndBodyNodePair<FreeJoint>(
			  nullptr, properties, 
			  dart::dynamics::BodyNode::AspectProperties(name)).second;

  // Make a shape for the Base
  std::shared_ptr<MeshShape> base(new MeshShape(
      Eigen::Vector3d(0., 0., 0.),
      MeshShape::loadMesh("/home/vinitha910/workspaces/cozmo_workspace/src/cozmo_description/meshes/cozmo_base.STL")));
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(std::static_pointer_cast<dart::dynamics::Shape>(base));
  
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());
  tf.linear() = R;

  bn->getParentJoint()->setTransformFromChildBodyNode(tf);
  shapeNode->getVisualAspect()->setRGB(Eigen::Vector3d(202/255., 180/255., 148/255.));

  return bn;
}

BodyNodePtr addBody(const SkeletonPtr& cozmo, BodyNodePtr parent, const std::string& name, const std::string& filepath)
{
  RevoluteJoint::Properties properties;
  
  auto joint_bn = cozmo->createJointAndBodyNodePair<RevoluteJoint>(
			  parent, properties,
			  dart::dynamics::BodyNode::AspectProperties(name));
  auto bn = joint_bn.second;
  auto joint = joint_bn.first;
  
  std::shared_ptr<MeshShape> child(new MeshShape(Eigen::Vector3d(0., 0., 0.), MeshShape::loadMesh(filepath)));
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(std::static_pointer_cast<dart::dynamics::Shape>(child));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  
  if (name == "head") {
    shapeNode->getVisualAspect()->setRGB(Eigen::Vector3d(190/255., 190/255., 190/255.));
    
    Eigen::Vector3d T = Eigen::Vector3d(0.03, 0.0615, 0.0385);
    tf.translation() = T;
    joint->setTransformFromParentBodyNode(tf);
    
    T =  Eigen::Vector3d(0.022, 0.02, 0.0);
    tf.translation() = T;
    joint->setTransformFromChildBodyNode(tf);
  } else if (name == "forklift") {
    shapeNode->getVisualAspect()->setRGB(Eigen::Vector3d(193/255., 24/255., 22/255.));

    Eigen::Vector3d T = Eigen::Vector3d(0.066, 0.001, 0.0032);
    tf.translation() = T;
    joint->setTransformFromParentBodyNode(tf);
    
    T =  Eigen::Vector3d(0.0028, 0.025, 0.0);
    tf.translation() = T;
    joint->setTransformFromChildBodyNode(tf);

    bn->getParentJoint()->setPosition(0, -0.12);
  } else if (name == "lower_forklift_strut_left_1" ||
	     name == "lower_forklift_strut_left_2") {
    shapeNode->getVisualAspect()->setRGB(Eigen::Vector3d(230/255., 230/255., 230/255.));

    Eigen::Vector3d T = Eigen::Vector3d(-0.004, 0.044, 0.0805);
    tf.translation() = T;
    joint->setTransformFromParentBodyNode(tf); //transfrom from parent moves joint and body node
    
    T = Eigen::Vector3d(0.006, 0.015, 0.0);
    tf.translation() = T;
    joint->setTransformFromChildBodyNode(tf); //transform from child moves only body node

    bn->getParentJoint()->setPosition(0, 0.07);
  } else if (name == "lower_forklift_strut_right_1") {
    shapeNode->getVisualAspect()->setRGB(Eigen::Vector3d(230/255., 230/255., 230/255.));
    
    Eigen::Vector3d T = Eigen::Vector3d(-0.004, 0.044, 0.0315);
    tf.translation() = T;
    joint->setTransformFromParentBodyNode(tf); //transfrom from parent moves joint and body node
    
    T = Eigen::Vector3d(0.006, 0.015, 0.0);
    tf.translation() = T;
    joint->setTransformFromChildBodyNode(tf); //transform from child moves only body node

    bn->getParentJoint()->setPosition(0, 0.07);
  } else if (name == "lower_forklift_strut_right_2") {
    shapeNode->getVisualAspect()->setRGB(Eigen::Vector3d(230/255., 230/255., 230/255.));
    
    Eigen::Vector3d T = Eigen::Vector3d(0.001, 0.014, 0.0);
    tf.translation() = T;
    joint->setTransformFromParentBodyNode(tf); //transfrom from parent moves joint and body node
    
    T = Eigen::Vector3d(0.0691, 0.0032, 0.0032);
    tf.translation() = T;
    joint->setTransformFromChildBodyNode(tf); //transform from child moves only body node

    bn->getParentJoint()->setPosition(0, 0.04);
  } else if (name == "upper_forklift_strut_left_1" ||
	     name == "upper_forklift_strut_left_2") {
    shapeNode->getVisualAspect()->setRGB(Eigen::Vector3d(230/255., 230/255., 230/255.));
   
    Eigen::Vector3d T = Eigen::Vector3d(-0.004, 0.058, 0.0805);
    tf.translation() = T;
    joint->setTransformFromParentBodyNode(tf); //transfrom from parent moves joint and body node
    
    T = Eigen::Vector3d(0.003, 0.021, 0.0);
    tf.translation() = T;
    joint->setTransformFromChildBodyNode(tf); //transform from child moves only body node

    bn->getParentJoint()->setPosition(0, 0.15);
    
  } else if (name == "upper_forklift_strut_right_1" ||
	     name == "upper_forklift_strut_right_2") {
    shapeNode->getVisualAspect()->setRGB(Eigen::Vector3d(230/255., 230/255., 230/255.));
   
    Eigen::Vector3d T = Eigen::Vector3d(-0.004, 0.058, 0.0315);
    tf.translation() = T;
    joint->setTransformFromParentBodyNode(tf); //transfrom from parent moves joint and body node
    
    T = Eigen::Vector3d(0.003, 0.021, 0.0);
    tf.translation() = T;
    joint->setTransformFromChildBodyNode(tf); //transform from child moves only body node

    bn->getParentJoint()->setPosition(0, 0.15);
  }
  
  return bn;
}
  
WorldPtr createCozmo()
{
  // Create a world and add the pendulum to the world
  WorldPtr world(new World);
  
  SkeletonPtr cozmo = Skeleton::create("cozmo");  
  BodyNodePtr base = makeRootBody(cozmo, "body");
  BodyNodePtr head = addBody(cozmo, base, "head",
  			   "/home/vinitha910/workspaces/cozmo_workspace/src/cozmo_description/meshes/head.STL");
  
  BodyNodePtr upper_forklift_strut_left_1 =
    addBody(cozmo, base, "upper_forklift_strut_left_1",
  	    "/home/vinitha910/workspaces/cozmo_workspace/src/cozmo_description/meshes/upper_forklift_strut.STL");
  BodyNodePtr upper_forklift_strut_right_1 =
    addBody(cozmo, base, "upper_forklift_strut_right_1",
            "/home/vinitha910/workspaces/cozmo_workspace/src/cozmo_description/meshes/upper_forklift_strut.STL");
  BodyNodePtr lower_forklift_strut_left_1 =
    addBody(cozmo, base, "lower_forklift_strut_left_1",
            "/home/vinitha910/workspaces/cozmo_workspace/src/cozmo_description/meshes/lower_forklift_strut.STL");
  BodyNodePtr lower_forklift_strut_right_1 =
    addBody(cozmo, base, "lower_forklift_strut_right_1",
            "/home/vinitha910/workspaces/cozmo_workspace/src/cozmo_description/meshes/lower_forklift_strut.STL");
  
  BodyNodePtr forklift = addBody(cozmo, upper_forklift_strut_right_1, "forklift",
  	                         "/home/vinitha910/workspaces/cozmo_workspace/src/cozmo_description/meshes/forklift.STL");

  BodyNodePtr lower_forklift_strut_right_2 =
    addBody(cozmo, forklift, "lower_forklift_strut_right_2",
            "/home/vinitha910/workspaces/cozmo_workspace/src/cozmo_description/meshes/lower_forklift_strut.STL");

  world->addSkeleton(cozmo);
  auto constraint = std::make_shared<WeldJointConstraint>(lower_forklift_strut_right_1, lower_forklift_strut_right_2);

  // world->getConstraintSolver()->addConstraint(constraint_1);
  // world->getConstraintSolver()->addConstraint(constraint_2);
  // world->getConstraintSolver()->addConstraint(constraint_3);
  // world->getConstraintSolver()->addConstraint(constraint_4);

  return world;
}
}

