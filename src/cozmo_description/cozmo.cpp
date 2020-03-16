#include "cozmo_description/cozmo.hpp"
#include "Eigen/Dense"
#include <cmath>
#include <chrono>
#include <thread>
#include "aikido/trajectory/Trajectory.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/statespace/Interpolator.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"
#include "aikido/statespace/SE2.hpp"

namespace libcozmo{
using BoxShape = dart::dynamics::BoxShape;
using MeshShape = dart::dynamics::MeshShape;
using FreeJoint = dart::dynamics::FreeJoint;
using RevoluteJoint = dart::dynamics::RevoluteJoint;
using VisualAspect = dart::dynamics::VisualAspect;
using Skeleton = dart::dynamics::Skeleton; 
using WeldJointConstraint = dart::constraint::WeldJointConstraint;
using InverseKinematicsPtr = dart::dynamics::InverseKinematicsPtr;
using Interpolator = aikido::statespace::Interpolator;
using GeodesicInterpolator = aikido::statespace::GeodesicInterpolator;
using Interpolated = aikido::trajectory::Interpolated;
using aikido::statespace::SE2;

Cozmo::Cozmo(const std::string& mesh_dir){
  createCozmo(mesh_dir);
}

void Cozmo::createIKModule() 
{
    ik = dart::dynamics::InverseKinematics::create(ghost_strut);
    ik->useChain();
}
    
void Cozmo::setForkliftPosition(double pos) 
{
    lower_forklift_strut_right->getParentJoint()->setPosition(0, pos);
    upper_forklift_strut_right->getParentJoint()->setPosition(0, pos + 0.08);
    lower_forklift_strut_left->getParentJoint()->setPosition(0, pos);
    upper_forklift_strut_left->getParentJoint()->setPosition(0, pos + 0.08);
    
    Eigen::Isometry3d goal_pose;
    goal_pose = lower_forklift_strut_right->getTransform(base);
    
    // Solve IK
    ik->getTarget()->setTransform(goal_pose, base);
    Eigen::VectorXd ik_solution;
    if (ik->solveAndApply(ik_solution, true)) {
        std::cout << "IK solution found!\n";
    } else {
        std::cout << "No IK solution found.\n" << std::endl;
    } 
}  

BodyNodePtr Cozmo::makeRootBody(const SkeletonPtr& cozmo,
				const std::string& mesh_name,
				const std::string& mesh_dir)
{
    FreeJoint::Properties properties;

    BodyNodePtr bn = cozmo->createJointAndBodyNodePair<FreeJoint>(
        nullptr,
		properties, 
		dart::dynamics::BodyNode::AspectProperties(mesh_name)).second;

    std::shared_ptr<MeshShape> base(new MeshShape(Eigen::Vector3d(1., 1., 1.),
						  MeshShape::loadMesh(mesh_dir + "/cozmo_base.STL")));
    
    auto shapeNode = bn->createShapeNodeWith<VisualAspect>(std::static_pointer_cast<dart::dynamics::Shape>(base));
  
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    
    R = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());
    tf.linear() = R;

    bn->getParentJoint()->setTransformFromChildBodyNode(tf);
    shapeNode->getVisualAspect()->setRGB(Eigen::Vector3d(190/255., 190/255., 190/255.));

    return bn;
}

BodyNodePtr Cozmo::addBody(const SkeletonPtr& cozmo,
			   BodyNodePtr parent,
			   const std::string& mesh_name,
			   const std::string& mesh_dir,
	                   Eigen::Vector3d transformFromParent,
			   Eigen::Vector3d transformFromChild)
{
    RevoluteJoint::Properties properties;
    properties.mName = mesh_name;
    
    auto joint_bn = cozmo->createJointAndBodyNodePair<RevoluteJoint>(parent,
								     properties,
								     dart::dynamics::BodyNode::AspectProperties(mesh_name));
    auto bn = joint_bn.second;
    auto joint = joint_bn.first;

    // Assumes that all mesh file names are at most 20 characters
    // Pulls the file name out of the longer body node name and creates file path 
    const std::string& filepath = mesh_dir + "/"  + mesh_name.substr(0,20) + ".STL";
    
    std::shared_ptr<MeshShape> child(new MeshShape(Eigen::Vector3d(1., 1., 1.),
						   MeshShape::loadMesh(filepath)));

    auto shapeNode = bn->createShapeNodeWith<VisualAspect>(std::static_pointer_cast<dart::dynamics::Shape>(child));

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    shapeNode->getVisualAspect()->setRGB(Eigen::Vector3d(190/255., 190/255., 190/255.));

    tf.translation() = transformFromParent;
    joint->setTransformFromParentBodyNode(tf);
    
    tf.translation() = transformFromChild;
    joint->setTransformFromChildBodyNode(tf);

    return bn;
}
  
SkeletonPtr Cozmo::createCozmo(const std::string& mesh_dir)
{
    cozmo = Skeleton::create("cozmo");  
    base = makeRootBody(cozmo, "body", mesh_dir);
    head = addBody(cozmo,
    		  base,
    		  "head",
    		  mesh_dir,
    		  Eigen::Vector3d(0.03, 0.0615, 0.0385),
    		  Eigen::Vector3d(0.022, 0.02, 0.0));
  
    upper_forklift_strut_left = addBody(cozmo,
					base,
					"upper_forklift_strut_left",
					mesh_dir,
					Eigen::Vector3d(-0.0045, 0.058, 0.0805),
					Eigen::Vector3d(0.003, 0.021, 0.0));
  
    upper_forklift_strut_right = addBody(cozmo,
					base,
					"upper_forklift_strut_right",
					mesh_dir,
					Eigen::Vector3d(-0.0045, 0.058, 0.0315),
					Eigen::Vector3d(0.003, 0.021, 0.0));
  
    lower_forklift_strut_left = addBody(cozmo,
					base,
					"lower_forklift_strut_left",
					mesh_dir,
					Eigen::Vector3d(-0.0025, 0.044, 0.0805),
					Eigen::Vector3d(0.006, 0.015, 0.0));
  
    lower_forklift_strut_right = addBody(cozmo,
					base,
					"lower_forklift_strut_right",
					mesh_dir,
					Eigen::Vector3d(-0.0025, 0.044, 0.0315),
					Eigen::Vector3d(0.006, 0.015, 0.0));
  
    forklift = addBody(cozmo,
		      upper_forklift_strut_right,
		      "forklift",
		      mesh_dir,
		      Eigen::Vector3d(0.066, 0.001, 0.0032),
		      Eigen::Vector3d(0.0028, 0.025, 0.0));

    // We solve IK in the setForkliftPosition to make this strut exactly
    // match lower_forklift_strut_right in order to compensate for the
    // inability to model closed chains 
    ghost_strut = addBody(cozmo,
  			  forklift,
  			  "lower_forklift_strut_ghost",
  			  mesh_dir,
  			  Eigen::Vector3d(0.003, 0.01, 0.0),
  			  Eigen::Vector3d(0.0691, 0.0032, 0.0032));

    createIKModule();
    setForkliftPosition(0.0);
    
    return cozmo;
}

void Cozmo::executeTrajectory(
    std::chrono::milliseconds period,
    TrajectoryPtr traj) 
{
    using std::chrono::system_clock;
    using std::chrono::duration;
    using std::chrono::duration_cast;
  
    system_clock::time_point startTime = system_clock::now();
    bool trajInExecution = true;
    
    while (trajInExecution) {   
        auto space = traj->getStateSpace();
        if (space == NULL) { std::cout << "State space is NULL" << std::endl; }    
        auto scopedState = space->createState();
        
        system_clock::time_point const now = system_clock::now();
        double t = duration_cast<duration<double> >(now - startTime).count();

        traj->evaluate(t, scopedState);

        std::unique_lock<std::mutex> skeleton_lock(cozmo->getMutex());

        auto state = static_cast<SE2::State*>(scopedState.getState());
        Eigen::Isometry2d trans = state->getIsometry();
        
        Eigen::Isometry3d trans_3d = Eigen::Isometry3d::Identity();
        trans_3d.translation() << trans.translation()[0], trans.translation()[1], 0.;
        trans_3d.linear().block<2,2>(0,0) = trans.linear();

        base->getParentJoint()->setPositions(
            dart::dynamics::FreeJoint::convertToPositions(trans_3d));

        skeleton_lock.unlock();

        bool const is_done = (t >= traj->getEndTime());
        if (is_done) trajInExecution = false;

        std::this_thread::sleep_until(now + period);
    }
}

void Cozmo::setState(const double& x, const double& y, const Eigen::Quaterniond& orientation) {
    Eigen::Isometry3d state = Eigen::Isometry3d::Identity();
    state.linear() = orientation.normalized().toRotationMatrix();
    state.translation() << x, y, 0.;
    base->getParentJoint()->setPositions(
        dart::dynamics::FreeJoint::convertToPositions(state));
}

SE2::State Cozmo::createState(const double x, const double y, const double th) 
{
    SE2::State s;
    Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
    const Eigen::Rotation2D<double> rot(th);
    t.linear() = rot.toRotationMatrix();
    Eigen::Vector2d trans;
    trans << x, y;
    t.translation() = trans;
    s.setIsometry(t);
    return s;
}
  
std::shared_ptr<Interpolated> Cozmo::createInterpolatedTraj(
  std::vector<Waypoint> waypoints) 
{
    std::shared_ptr<SE2> statespace = std::make_shared<SE2>();
    std::shared_ptr<Interpolator> interpolator = 
      std::make_shared<GeodesicInterpolator>(statespace);
   
    const int num_waypoints = waypoints.size();
    Waypoint *w = new Waypoint;

    SE2::State s;
    std::shared_ptr<Interpolated> traj;
    traj = std::make_shared<Interpolated>(statespace, interpolator);

    for (int i=0; i < num_waypoints; i++) {    
      w = &waypoints.at(i);
      s = createState(w->x, w->y, w->th);
      traj->addWaypoint(w->t, &s);
    }

    return traj;
}

Eigen::Vector3d Cozmo::getState(
    const std::shared_ptr<Interpolated> path, const double& time) {
    auto space = path->getStateSpace();
    auto scopedState = space->createState();
    path->evaluate(time, scopedState);
    const auto state = 
        static_cast<aikido::statespace::SE2::State*>(scopedState.getState());
    const auto transform = state->getIsometry();
    Eigen::Rotation2Dd rotation = Eigen::Rotation2Dd::Identity();
    rotation.fromRotationMatrix(transform.rotation());
    const auto translation = transform.translation();
    return Eigen::Vector3d(translation.x(), translation.y(), rotation.angle());
}

}  // namespace libcozmo
