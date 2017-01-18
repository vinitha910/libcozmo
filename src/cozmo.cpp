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

  PyGILState_STATE gs;
  gs = PyGILState_Ensure();

  std::stringstream buf;
  buf << "import cozmo" << std::endl
      << "robot = None" << std::endl
      << "def run(sdk_conn):" << std::endl
      << "    print(\"[PYTHON] CONNECTING TO COZMO\")" << std::endl
      << "    global robot" << std::endl
      << "    robot = sdk_conn.wait_for_robot()" << std::endl
      << "    import IPython" << std::endl
      << "    IPython.embed()" << std::endl
      << "def connect_to_coz():" << std::endl
      << "    cozmo.setup_basic_logging()" << std::endl
      << "    try:" << std::endl
      << "        cozmo.connect(run)" << std::endl
      << "    except cozmo.ConnectionError as e:" << std::endl
      << "        sys.exit(\"[PYTHON] A connection error occurred: %s\" % e)" <<std::endl;

  PyObject *pCompiledFn;
  pCompiledFn = Py_CompileString(buf.str().c_str(), "", Py_file_input);
  std::cout << "[cozmo.cpp] Compiled Python Function" << std::endl;

  PyObject *pModule;
  pModule = PyImport_ExecCodeModule("cozmo_conn", pCompiledFn);
  std::cout << "[cozmo.cpp] Created Module" << std::endl;

  PyObject *pConn;
  pConn = PyObject_GetAttrString(pModule, "connect_to_coz()");
  std::cout << "[cozmo.cpp] Connecting to Cozmo" << std::endl;

  pRobot = PyObject_GetAttrString(pModule, "robot");
  std::cout << "[cozmo.cpp] Obtained robot Py_Object" << std::endl;

  PyGILState_Release(gs);
}

Cozmo::~Cozmo() {
  Py_Finalize();
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
