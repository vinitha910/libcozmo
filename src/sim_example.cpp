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

  PyGILState_STATE gs;
  gs = PyGILState_Ensure();

  std::stringstream buf;
  buf << "import cozmo" << std::endl
      << "robot = sdk_conn.wait_for_robot()" << std::endl
      << "print('CONNECTING TO COZMO')" << std::endl;
  std::cout << buf.str().c_str() << std::endl;

  PyObject *pCompiledFn;
  pCompiledFn = Py_CompileString(buf.str().c_str(), "", Py_file_input);
  assert(pCompiledFn != NULL);
  std::cout << "[SimExample] Compiled Python Function" << std::endl;

  PyObject *pModule;
  pModule = PyImport_ExecCodeModule("conn_coz", pCompiledFn);
  assert(pModule != NULL);
  std::cout << "[SimExample] Created Module" << std::endl;

  //std::cout << "[SimExample] Predicting control." << std::endl;
  //PyObject *robot = PyObject_CallObject(pPredictFn, NULL);

  //PyObject *pose = PyObject_GetAttrString(robot, "pose");

  PyGILState_Release(gs);

  Py_Finalize();
  
  ros::spin();

  return 0;
}
