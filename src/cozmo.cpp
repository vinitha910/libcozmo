#include "cozmo/cozmo.hpp"
#include "Eigen/Dense"
#include <cmath>
#include <Python.h>

namespace libcozmo{
using BoxShape = dart::dynamics::BoxShape;
using MeshShape = dart::dynamics::MeshShape;
using FreeJoint = dart::dynamics::FreeJoint;
using RevoluteJoint = dart::dynamics::RevoluteJoint;
using VisualAspect = dart::dynamics::VisualAspect;
using Skeleton = dart::dynamics::Skeleton; 
using WeldJointConstraint = dart::constraint::WeldJointConstraint;
using InverseKinematicsPtr = dart::dynamics::InverseKinematicsPtr;

Cozmo::Cozmo(const std::string& mesh_dir){
  createCozmo(mesh_dir);

  Py_Initialize();
  PyRun_SimpleString("import sys; import os; sys.path.insert(0, os.getcwd)");
}

Cozmo::~Cozmo() {
  Py_Finalize();
}

std::vector<double> Cozmo::getPose() {
  PyGILState_STATE gs;
  gs = PyGILState_Ensure();

  std::stringstream buf;
  buf << "import cozmo" << std::endl
      << "def getPose(robot: cozmo.robot.Robot):" << std::endl
      << "    x = robot.pose.position.x" << std::endl
      << "    y = robot.pose.position.y" << std::endl
      << "    z = robot.pose.position.z" << std::endl
      << "    angle_z = robot.pose.rotation.angle_z.radians" << std::endl
      << "    return [x, y, z, angle_z]" << std::endl
      << "cozmo.run_program(getPose)" << std::endl;

  PyObject *pCompiledFn;
  pCompiledFn = Py_CompileString(buf.str().c_str(), "", Py_file_input);
  std::cout << "[cozmo.cpp] Compiled Python Function" << std::endl;

  PyObject *pCozPose;
  pCozPose = PyImport_ExecCodeModule("getPose", pCompiledFn);
  std::cout << "[cozmo.cpp] Getting Pose" << std::endl;

  std::cout << "[cozmo.cpp] Received Pose List: " << PyList_Check(pCozPose) << std::endl;

  double x = PyFloat_AsDouble(PyList_GetItem(pCozPose, 0));
  double y = PyFloat_AsDouble(PyList_GetItem(pCozPose, 1));
  double z = PyFloat_AsDouble(PyList_GetItem(pCozPose, 2));
  double angle_z = PyFloat_AsDouble(PyList_GetItem(pCozPose, 3));

  PyGILState_Release(gs);

  std::vector<double> pose;
  pose.push_back(x);
  pose.push_back(y);
  pose.push_back(z);
  pose.push_back(angle_z);

  return pose;
}

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
