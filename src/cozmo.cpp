#include "cozmo/cozmo.hpp"
#include "Eigen/Dense"
#include <cmath>

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
using World = dart::simulation::World;
using WeldJointConstraint = dart::constraint::WeldJointConstraint;
using JacobianNode = dart::dynamics::JacobianNode;
using InverseKinematicsPtr = dart::dynamics::InverseKinematicsPtr;

Cozmo::Cozmo(const std::string& mesh_dir){
  createCozmo(mesh_dir);
}

void Cozmo::setPosition(float pos) {
  InverseKinematicsPtr ik = dart::dynamics::InverseKinematics::create(cozmo->getBodyNode("lower_forklift_strut_right_2"));
  ik->useChain();
  
  lower_forklift_strut_right_1->getParentJoint()->setPosition(0, pos);
  upper_forklift_strut_right->getParentJoint()->setPosition(0, pos + 0.07);
  lower_forklift_strut_left->getParentJoint()->setPosition(0, pos);
  upper_forklift_strut_left->getParentJoint()->setPosition(0, pos + 0.07);
  
  Eigen::Isometry3d goal_pose;
  goal_pose = lower_forklift_strut_right_1->getTransform(base);
  
  // Solve IK
  ik->getTarget()->setTransform(goal_pose, base);
  Eigen::VectorXd ik_solution;
  if (ik->solve(ik_solution, true)) {
    std::cout << "IK solution found!\n";
    std::cout << ik_solution.head(3) << std::endl;
  } else {
    std::cout << "No IK solution found" << std::endl;
  }
  
}  

BodyNodePtr Cozmo::makeRootBody(const SkeletonPtr& cozmo, const std::string& name, const std::string& mesh_dir)
  {
    FreeJoint::Properties properties;

    BodyNodePtr bn = cozmo->createJointAndBodyNodePair<FreeJoint>(nullptr, properties, 
								  dart::dynamics::BodyNode::AspectProperties(name)).second;

    std::shared_ptr<MeshShape> base(new MeshShape(Eigen::Vector3d(0., 0., 0.), MeshShape::loadMesh(mesh_dir + "/cozmo_base.STL")));
    auto shapeNode = bn->createShapeNodeWith<VisualAspect>(std::static_pointer_cast<dart::dynamics::Shape>(base));
  
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    
    R = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());
    tf.linear() = R;

    bn->getParentJoint()->setTransformFromChildBodyNode(tf);
    shapeNode->getVisualAspect()->setRGB(Eigen::Vector3d(202/255., 180/255., 148/255.));

    return bn;
  }

BodyNodePtr Cozmo::addBody(const SkeletonPtr& cozmo, BodyNodePtr parent, const std::string& name, const std::string& mesh_dir,
	                   Eigen::Vector3d transformFromParent, Eigen::Vector3d transformFromChild, Eigen::Vector3d color, float initPos)
  {
    RevoluteJoint::Properties properties;
    properties.mName = name;
    
    auto joint_bn = cozmo->createJointAndBodyNodePair<RevoluteJoint>(parent, properties,
								     dart::dynamics::BodyNode::AspectProperties(name));
    auto bn = joint_bn.second;
    auto joint = joint_bn.first;

    const std::string& filepath = mesh_dir + "/"  + name.substr(0,20) + ".STL";
    
    std::shared_ptr<MeshShape> child(new MeshShape(Eigen::Vector3d(0., 0., 0.), MeshShape::loadMesh(filepath)));
    auto shapeNode = bn->createShapeNodeWith<VisualAspect>(std::static_pointer_cast<dart::dynamics::Shape>(child));

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    shapeNode->getVisualAspect()->setRGB(color);

    tf.translation() = transformFromParent;
    joint->setTransformFromParentBodyNode(tf);
    
    tf.translation() = transformFromChild;
    joint->setTransformFromChildBodyNode(tf);

    bn->getParentJoint()->setInitialPosition(0, initPos);

    return bn;
  }
  
SkeletonPtr Cozmo::createCozmo(const std::string& mesh_dir)
  {
    //WorldPtr world(new World);

    Eigen::Vector3d grey = Eigen::Vector3d(190/255., 190/255., 190/255.);
    Eigen::Vector3d red = Eigen::Vector3d(193/255., 24/255., 22/255.);
    Eigen::Vector3d white = Eigen::Vector3d(230/255., 230/255., 230/255.);
    
    cozmo = Skeleton::create("cozmo");  
    base = makeRootBody(cozmo, "body", mesh_dir);
    head = addBody(cozmo, base, "head", mesh_dir, Eigen::Vector3d(0.03, 0.0615, 0.0385),
			        Eigen::Vector3d(0.022, 0.02, 0.0), grey, 0.0);
  
    upper_forklift_strut_left =
      addBody(cozmo, base, "upper_forklift_strut_left", mesh_dir,
	      Eigen::Vector3d(-0.0045, 0.058, 0.0805), Eigen::Vector3d(0.003, 0.021, 0.0), white, 0.15);
  
    upper_forklift_strut_right =
      addBody(cozmo, base, "upper_forklift_strut_right", mesh_dir,
	      Eigen::Vector3d(-0.0045, 0.058, 0.0315), Eigen::Vector3d(0.003, 0.021, 0.0), white, 0.15);
  
    lower_forklift_strut_left =
      addBody(cozmo, base, "lower_forklift_strut_left", mesh_dir,
	      Eigen::Vector3d(-0.0025, 0.044, 0.0805), Eigen::Vector3d(0.006, 0.015, 0.0), white, 0.07);
  
    lower_forklift_strut_right_1 =
      addBody(cozmo, base, "lower_forklift_strut_right_1", mesh_dir,
	      Eigen::Vector3d(-0.0025, 0.044, 0.0315), Eigen::Vector3d(0.006, 0.015, 0.0), white, 0.07);
  
    forklift =
      addBody(cozmo, upper_forklift_strut_right, "forklift", mesh_dir,
              Eigen::Vector3d(0.066, 0.001, 0.0032), Eigen::Vector3d(0.0028, 0.025, 0.0), red, -0.12);

    lower_forklift_strut_right_2 =
      addBody(cozmo, forklift, "lower_forklift_strut_right_2", mesh_dir,
	      Eigen::Vector3d(0.003, 0.01, 0.0), Eigen::Vector3d(0.0691, 0.0032, 0.0032), white, 0.04);
    setPosition(0.0);
    //world->addSkeleton(cozmo);
    auto constraint = std::make_shared<WeldJointConstraint>(lower_forklift_strut_right_1, lower_forklift_strut_right_2);    
    return cozmo;
  }
  
