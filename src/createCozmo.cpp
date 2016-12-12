#include "cozmo/cozmo.hpp"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "ros/ros.h"
#include <aikido/rviz/InteractiveMarkerViewer.hpp>

static const std::string topicName("dart_markers");

int main(int argc, char* argv[])
{
  const std::string mesh_dir = "/home/vinitha910/workspaces/cozmo_workspace/src/cozmo_description/meshes";
  Cozmo cozmo(mesh_dir);
  
  // Start the RViz viewer.
  std::cout << "Starting ROS node." << std::endl;
  ros::init(argc, argv, "load_cozmo");

  std::cout << "Starting viewer. Please subscribe to the '" << topicName
	    << "' InteractiveMarker topic in RViz." << std::endl;

  aikido::rviz::InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(cozmo.getCozmoSkeleton());
  viewer.setAutoUpdate(true);

  std::cout << "Press <Ctrl> + C to exit." << std::endl;

  std::cin.get();

  cozmo.setPosition(M_PI/6);
  
  std::cin.get();

  
  ros::spin();
}
