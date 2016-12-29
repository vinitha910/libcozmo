#include "cozmo/cozmo.hpp"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "ros/ros.h"
#include <cstdlib>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>

#include <Python.h>
#include <cassert>

static const std::string topicName("dart_markers");

int main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "Please include the path to the mesh directory" << std::endl;
    return 0;
  }

  const std::string mesh_dir = argv[1];
  libcozmo::Cozmo cozmo(mesh_dir);
  
  // Start the RViz viewer.
  std::cout << "Starting ROS node." << std::endl;
  ros::init(argc, argv, "load_cozmo");

  std::cout << "Starting viewer. Please subscribe to the '" << topicName
	    << "' InteractiveMarker topic in RViz." << std::endl;

  aikido::rviz::InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(cozmo.getCozmoSkeleton());
  viewer.setAutoUpdate(true);

  Py_Initialize();

  PyRun_SimpleString("import sys; sys.path.insert(0, '/home/vinitha910/workspaces/cozmo_workspace/src/libcozmo/src/')");
  
  PyGILState_STATE gs;
  gs = PyGILState_Ensure();
  
  std::stringstream buf;
  // buf << "import cozmo" << std::endl
  //     << "robot = None" << std::endl
  //     << "def run(sdk_conn):" << std::endl
  //     << "    print(\"CONNECTING TO COZMO\")" << std::endl
  //     << "    global robot" << std::endl
  //     << "    robot = sdk_conn.wait_for_robot()" << std::endl
  //     << "    import IPython" << std::endl
  //     << "    IPython.embed()" << std::endl
  //     << "def connect_to_coz():" << std::endl
  //     << "    cozmo.setup_basic_logging()" << std::endl
  //     << "    try:" << std::endl
  //     << "        cozmo.connect(run)" << std::endl
  //     << "    except cozmo.ConnectionError as e:" << std::endl
  //     << "        sys.exit(\"A connection error occurred: %s\" % e)" <<std::endl;

  // buf << "from console import robot" << std::endl
  //     << "def GetPose():" << std::endl
  //     << "    return float(robot.pose.position.x)"  << std::endl;  
  // std::cout << buf.str().c_str() << std::endl;

  // PyObject *pCompiledFn;
  // pCompiledFn = Py_CompileString(buf.str().c_str(), "", Py_file_input);
  // assert(pCompiledFn != NULL);
  // std::cout << "[SimExample] Compiled Python Function" << std::endl;

  PyObject *pModule;
  // pModule = PyImport_ExecCodeModule("robot_module", pCompiledFn);
  // assert(pModule != NULL);
  // std::cout << "[SimExample] Created Module " << std::endl;
  pModule = PyImport_ImportModule("console");
  std::cout << "[SimExample] Imported console" << std::endl;
  
  PyObject *pGetPoseFn;
  pGetPoseFn = PyObject_GetAttrString(pModule, "robot.pose.position.x");
  assert(pGetPoseFn != NULL);
  std::cout << "[SimExample] Located GetPose Function" << std::endl;
  
  //PyObject *pose = PyObject_CallObject(pGetPoseFn, NULL);
  //assert(pose != NULL);
  std::cout << "[SimExample] Got Pose" << std::endl;
  std::cout << "pose: " << PyUnicode_AsUTF8(PyObject_Str(pGetPoseFn)) << std::endl;
  std::cout << "pose: " << PyFloat_AsDouble(pGetPoseFn) << std::endl;

  PyGILState_Release(gs);

  Py_Finalize();
  
  ros::spin();

  return 0;
}
