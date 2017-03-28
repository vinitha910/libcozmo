#include "cozmo/cozmo.hpp"
#include "Eigen/Dense"
#include <cmath>
#include <Python.h>
#include <chrono>
#include <thread>
#include "aikido/trajectory/Trajectory.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/statespace/Interpolator.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"
#include "aikido/statespace/SE2.hpp"
#include "aikido/statespace/Rn.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace libcozmo{
using BoxShape = dart::dynamics::BoxShape;
using MeshShape = dart::dynamics::MeshShape;
using FreeJoint = dart::dynamics::FreeJoint;
using RevoluteJoint = dart::dynamics::RevoluteJoint;
using VisualAspect = dart::dynamics::VisualAspect;
using Skeleton = dart::dynamics::Skeleton; 
using WeldJointConstraint = dart::constraint::WeldJointConstraint;
using InverseKinematicsPtr = dart::dynamics::InverseKinematicsPtr;
using Rn = aikido::statespace::Rn;
using Interpolator = aikido::statespace::Interpolator;
using GeodesicInterpolator = aikido::statespace::GeodesicInterpolator;
using InterpolatorPtr = aikido::statespace::InterpolatorPtr;
using StateSpace = aikido::statespace::StateSpace;
using StateSpacePtr = aikido::statespace::StateSpacePtr;
using aikido::statespace::SE2;

Cozmo::Cozmo(const std::string& mesh_dir){
  createCozmo(mesh_dir);

  Py_Initialize();
  PyRun_SimpleString("import sys; import os; sys.path.insert(0, os.getcwd)");
}

Cozmo::~Cozmo() {
  Py_Finalize();
}

cozmoPose Cozmo::getPose() {
  PyGILState_STATE gs;
  gs = PyGILState_Ensure();

  std::stringstream buf;
  buf << "import cozmo" << std::endl
      << "pose = None" << std::endl
      << "def getPose(robot: cozmo.robot.Robot):" << std::endl
      << "    x = robot.pose.position.x" << std::endl
      << "    y = robot.pose.position.y" << std::endl
      << "    angle_z = robot.pose.rotation.angle_z.radians" << std::endl
      << "    global pose" << std::endl
      << "    pose = [x, y, angle_z]" << std::endl
      << "cozmo.run_program(getPose)" << std::endl;

  PyObject *pCompiledFn;
  pCompiledFn = Py_CompileString(buf.str().c_str(), "", Py_file_input);
  std::cout << "[cozmo.cpp] Compiled Python Function" << std::endl;

  PyObject *pModule;
  pModule = PyImport_ExecCodeModule("getPose", pCompiledFn);
  std::cout << "[cozmo.cpp] Getting Pose" << std::endl;

  PyObject *pCozPose = PyObject_GetAttrString(pModule, "pose");

  double x = PyFloat_AsDouble(PyList_GetItem(pCozPose, 0));
  double y = PyFloat_AsDouble(PyList_GetItem(pCozPose, 1));
  double angle_z = PyFloat_AsDouble(PyList_GetItem(pCozPose, 2));

  std::cout << "Pose: [" << x << ", " << y << ", " << angle_z << "]" << std::endl;
  PyGILState_Release(gs);

  cozmoPose pose;
  pose.x = x;
  pose.y = y;
  pose.th = angle_z;

  return pose;
}

  // TODO: Remove z position because Cozmo can't fly 
void Cozmo::goToPose(std::vector<double> pos, double angle_z) {
  PyGILState_STATE gs;
  gs = PyGILState_Ensure();

  std::stringstream buf;
  buf << "import cozmo" << std::endl
      << "pose = None" << std::endl
      << "def goToPose(robot: cozmo.robot.Robot):" << std::endl
      << "    action = robot.go_to_pose(pose)" << std::endl
      << "    action.wait_for_completed()" << std::endl
      << "def setCozPose(p):" << std::endl
      << "    r = cozmo.util.Angle(radians=p[3])" << std::endl
      << "    global pose" << std::endl
      << "    pose = cozmo.util.pose_z_angle(p[0], p[1], p[2], r)" << std::endl
      << "    cozmo.run_program(goToPose)" << std::endl;

  PyObject *pCompiledFn;
  pCompiledFn = Py_CompileString(buf.str().c_str(), "", Py_file_input);
  std::cout << "[cozmo.cpp] Compiled Python Function" << std::endl;

  PyObject *pModule;
  pModule = PyImport_ExecCodeModule("goToPose", pCompiledFn);
  std::cout << "[cozmo.cpp] Create goToPose Module" << std::endl;

  PyObject *pGoToPoseFn = PyObject_GetAttrString(pModule, "setCozPose");
  std::cout << "[cozmo.cpp] Retrieved goToPose Function" << std::endl;

  PyObject *pyList = PyList_New(4);
  int setItem;

  PyObject *x = Py_BuildValue("f", pos[0]);
  PyObject *y = Py_BuildValue("f", pos[1]);
  PyObject *z = Py_BuildValue("f", pos[2]);
  PyObject *rot = Py_BuildValue("f", angle_z);

  setItem = PyList_SetItem(pyList, 0, x);
  setItem = PyList_SetItem(pyList, 1, y);
  setItem = PyList_SetItem(pyList, 2, z);
  setItem = PyList_SetItem(pyList, 3, rot);

  PyObject *args = PyTuple_Pack(1, pyList);

  std::cout << "[cozmo.cpp] Created PyList args" << std::endl;
  std::cout << "[cozmo.cpp] Going to Pose [x: " << pos[0] << ", y: " << pos[1] 
	    << ", z: " << pos[2] << ", angle_z: " << angle_z << "]" << std::endl;
  PyObject *myResult = PyObject_CallObject(pGoToPoseFn, args);
 
  PyGILState_Release(gs);
}

void Cozmo::driveStraight(double dist, double speed, double distInInches) {
  PyGILState_STATE gs;
  gs = PyGILState_Ensure();

  std::stringstream buf;
  buf << "import cozmo" << std::endl
      << "dist = 0" << std::endl
      << "speed = 0" << std::endl
      << "def driveStraight(robot: cozmo.robot.Robot):" << std::endl
      << "    action = robot.drive_straight(dist, speed, should_play_anim=False)" << std::endl
      << "    action.wait_for_completed()" << std::endl
      << "def setCozObj(args):" << std::endl
      << "    global dist" << std::endl
      << "    if args[2] == 1.0:" << std::endl
      << "        dist = cozmo.util.distance_inches(args[0])" << std::endl
      << "    else:" << std::endl
      << "        dist = cozmo.util.distance_mm(args[0])" << std::endl
      << "    global speed" << std::endl
      << "    speed = cozmo.util.Speed(args[1])" << std::endl
      << "    cozmo.run_program(driveStraight)" << std::endl;

  PyObject *pCompiledFn;
  pCompiledFn = Py_CompileString(buf.str().c_str(), "", Py_file_input);
  std::cout << "[cozmo.cpp] Compiled Python Function" << std::endl;

  PyObject *pModule;
  pModule = PyImport_ExecCodeModule("driveStraight", pCompiledFn);
  std::cout << "[cozmo.cpp] Create driveStraight Module" << std::endl;

  PyObject *pDriveStraightFn = PyObject_GetAttrString(pModule, "setCozObj");
  std::cout << "[cozmo.cpp] Retrieved driveStraight Function" << std::endl;

  PyObject *pyList = PyList_New(3);
  int setItem;

  PyObject *d = Py_BuildValue("f", dist);
  PyObject *s = Py_BuildValue("f", speed);
  PyObject *dist_type = Py_BuildValue("f", distInInches);
   
  setItem = PyList_SetItem(pyList, 0, d);
  setItem = PyList_SetItem(pyList, 1, s);
  setItem = PyList_SetItem(pyList, 2, dist_type);
  PyObject *args = PyTuple_Pack(1, pyList);

  std::cout << "[cozmo.cpp] Created PyList args" << std::endl;
  std::cout << "[cozmo.cpp] Driving Stright [distance: " << dist 
	    << ", speed: " << speed << "]" << std::endl;

  PyObject *myResult = PyObject_CallObject(pDriveStraightFn, args);
 
  PyGILState_Release(gs);
}

void Cozmo::turnInPlace(double angle, double angleInRad) {
  PyGILState_STATE gs;
  gs = PyGILState_Ensure();

  std::stringstream buf;
  buf << "import cozmo" << std::endl
      << "angle = 0" << std::endl
      << "def turnInPlace(robot: cozmo.robot.Robot):" << std::endl
      << "    action = robot.turn_in_place(angle)" << std::endl
      << "    action.wait_for_completed()" << std::endl
      << "def setCozObj(args):" << std::endl
      << "    global angle" << std::endl
      << "    if args[1] == 1.0:" << std::endl
      << "        angle = cozmo.util.radians(args[0])" << std::endl
      << "    else:" << std::endl
      << "        angle = cozmo.util.degrees(args[0])" << std::endl
      << "    cozmo.run_program(turnInPlace)" << std::endl;

  PyObject *pCompiledFn;
  pCompiledFn = Py_CompileString(buf.str().c_str(), "", Py_file_input);
  std::cout << "[cozmo.cpp] Compiled Python Function" << std::endl;

  PyObject *pModule;
  pModule = PyImport_ExecCodeModule("turnInPlace", pCompiledFn);
  std::cout << "[cozmo.cpp] Created turnInPlace Module" << std::endl;

  PyObject *pTurnInPlaceFn = PyObject_GetAttrString(pModule, "setCozObj");
  std::cout << "[cozmo.cpp] Retrieved turnInPlace Function" << std::endl;

  PyObject *pyList = PyList_New(2);
  int setItem;

  PyObject *a = Py_BuildValue("f", angle);
  PyObject *angle_type = Py_BuildValue("f", angleInRad);
     
  setItem = PyList_SetItem(pyList, 0, a);
  setItem = PyList_SetItem(pyList, 1, angle_type);
  PyObject *args = PyTuple_Pack(1, pyList);

  std::cout << "[cozmo.cpp] Created PyList args" << std::endl;
  std::cout << "[cozmo.cpp] Turning In Place [angle: " << angle 
	    << "]" << std::endl;

  PyObject *myResult = PyObject_CallObject(pTurnInPlaceFn, args);
 
  PyGILState_Release(gs);
}

void Cozmo::driveWheels(double l_wheel_speed, double r_wheel_speed,
			double l_wheel_acc, double r_wheel_acc,
			double duration) {
  PyGILState_STATE gs;
  gs = PyGILState_Ensure();

  std::stringstream buf;
  buf << "import cozmo" << std::endl
      << "import time" << std::endl
      << "vals = None" << std::endl
      << "def driveWheels(robot: cozmo.robot.Robot):" << std::endl
      << "    robot.drive_wheels(vals[0],vals[1],vals[2],vals[3],vals[4])" << std::endl
      << "def createGlobals(args):" << std::endl
      << "    global vals" << std::endl
      << "    vals = args" << std::endl
      << "    cozmo.run_program(driveWheels)" << std::endl;

  PyObject *pCompiledFn;
  pCompiledFn = Py_CompileString(buf.str().c_str(), "", Py_file_input);
  std::cout << "[cozmo.cpp] Compiled Python Function" << std::endl;

  PyObject *pModule;
  pModule = PyImport_ExecCodeModule("driveWheels", pCompiledFn);
  std::cout << "[cozmo.cpp] Create driveWheels Module" << std::endl;

  PyObject *pDriveWheelsFn = PyObject_GetAttrString(pModule, "createGlobals");
  std::cout << "[cozmo.cpp] Retrieved driveWheels Function" << std::endl;

  PyObject *pyList = PyList_New(5);
  int setItem;

  PyObject *lws = Py_BuildValue("f", l_wheel_speed);
  PyObject *rws = Py_BuildValue("f", r_wheel_speed);
  PyObject *lwa = Py_BuildValue("f", l_wheel_acc);
  PyObject *rwa = Py_BuildValue("f", r_wheel_acc);
  PyObject *dur = Py_BuildValue("f", duration);

  setItem = PyList_SetItem(pyList, 0, lws);
  setItem = PyList_SetItem(pyList, 1, rws);
  setItem = PyList_SetItem(pyList, 2, lwa);
  setItem = PyList_SetItem(pyList, 3, rwa);
  setItem = PyList_SetItem(pyList, 4, dur);

  PyObject *args = PyTuple_Pack(1, pyList);

  std::cout << "[cozmo.cpp] Created PyList args" << std::endl;
  std::cout << "[cozmo.cpp] Driving Wheels [l_wheel_speed: " << l_wheel_speed
	    << ", r_wheel_speed: " << r_wheel_speed 
            << ", l_wheel_acc: " << l_wheel_acc
            << ", r_wheel_acc: " << r_wheel_acc 
	    << ", duration: " << duration << "]" << std::endl;

  PyObject *myResult = PyObject_CallObject(pDriveWheelsFn, args);
 
  PyGILState_Release(gs);
}

void Cozmo::executeTwist(double V, double w, double dt) {
  double vdiff = (w * wheel_base)/2;
  double vl = V - vdiff;
  double vr = V + vdiff;

  driveWheels(vl,vr,0.0,0.0,dt);
}

void Cozmo::executeTrajectory(SkeletonPtr _cozmo, 
                              std::chrono::milliseconds _period,
			      TrajectoryPtr _traj) {
  using std::chrono::system_clock;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using aikido::statespace::dart::MetaSkeletonStateSpace;
  
  system_clock::time_point startTime = system_clock::now();
  bool trajInExecution = true;
  
  while (trajInExecution) {
        
    auto space = std::dynamic_pointer_cast<MetaSkeletonStateSpace>(_traj->getStateSpace());
    auto state = space->createState();

    system_clock::time_point const now = system_clock::now();
    double t = duration_cast<duration<double> >(now - startTime).count();

    _traj->evaluate(t, state);

    std::unique_lock<std::mutex> skeleton_lock(_cozmo->getMutex());
    space->setState(state);
    skeleton_lock.unlock();

    bool const is_done = (t >= _traj->getEndTime());
    if (is_done) trajInExecution = false;

    std::this_thread::sleep_until(now + _period);
  }
}

SE2::State createState(double x, double y, double th) {
  SE2::State s;
  Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
  Eigen::Rotation2D<double> rot(th);
  t.linear() = rot.toRotationMatrix();
  Eigen::Vector2d trans;
  trans << x, y;
  t.translation() = trans;
  s.setIsometry(t);
  return s;
}
  
std::shared_ptr<Interpolated> createInterpolatedTraj(std::vector<waypoint> waypoints) {
  std::shared_ptr<Rn> rvss = std::make_shared<Rn>(3);
  std::shared_ptr<Interpolator> interpolator = std::make_shared<GeodesicInterpolator>(rvss);
 
  int num_waypoints = waypoints.size();
  waypoint w;
 
  w = waypoints.at(0);
  SE2::State s1 = createState(w.x, w.y, w.th);
  
  w = waypoints.at(1);
  SE2::State s2 = createState(w.x, w.y, w.th);
  
  SE2::State s1inv;
  SE2::State s3;
  SE2 ss;
  Eigen::VectorXd twist;
  ss.getInverse(&s1, &s1inv);
  ss.compose(&s1inv, &s2, &s3);
  ss.logMap(&s3, twist);

  auto s = rvss->createState();
  rvss->setValue(s, twist);
  std::shared_ptr<Interpolated> traj;
  traj = std::make_shared<Interpolated>(rvss, interpolator);
  traj->addWaypoint(w.t, s);

  for (int i=2; i < num_waypoints; i++) {
    ss.copyState(&s2,&s1);
    
    w = waypoints.at(i);
    s2 = createState(w.x, w.y, w.th);
    
    ss.getInverse(&s1, &s1inv);
    ss.compose(&s1inv, &s2, &s3);
    ss.logMap(&s3, twist);
    rvss->setValue(s, twist);
    traj->addWaypoint(w.t, s);
  }

  return traj;
}

void Cozmo::createIKModule() {
  ik = dart::dynamics::InverseKinematics::create(ghost_strut);
  ik->useChain();
}
    
void Cozmo::setForkliftPosition(double pos) {
  lower_forklift_strut_right->getParentJoint()->setPosition(0, pos);
  upper_forklift_strut_right->getParentJoint()->setPosition(0, pos + 0.08);
  lower_forklift_strut_left->getParentJoint()->setPosition(0, pos);
  upper_forklift_strut_left->getParentJoint()->setPosition(0, pos + 0.08);
  
  Eigen::Isometry3d goal_pose;
  goal_pose = lower_forklift_strut_right->getTransform(base);
  
  // Solve IK
  ik->getTarget()->setTransform(goal_pose, base);
  Eigen::VectorXd ik_solution;
  if (ik->solve(ik_solution, true)) {
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

    BodyNodePtr bn = cozmo->createJointAndBodyNodePair<FreeJoint>(nullptr,
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
}  
