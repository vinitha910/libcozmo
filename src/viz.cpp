#include "cozmo/cozmo.hpp"
#include "cozmo/viz.hpp"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "ros/ros.h"
#include <cstdlib>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>

namespace visualizer{
  Viz::Viz(libcozmo::Cozmo cozmo, const std::string& mesh_dir, int argc, char* argv[])
{ 
  static const std::string topicName("dart_markers");

  // Start the RViz viewer.
  std::cout << "Starting ROS node." << std::endl;
  ros::init(argc, argv, "load_cozmo");

  std::cout << "Starting viewer. Please subscribe to the '" << topicName
	    << "' InteractiveMarker topic in RViz." << std::endl;

  aikido::rviz::InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(cozmo.getCozmoSkeleton());
  viewer.setAutoUpdate(true);
}
}
