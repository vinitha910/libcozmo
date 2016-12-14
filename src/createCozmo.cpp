#include "cozmo/cozmo.hpp"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "ros/ros.h"
#include <aikido/rviz/InteractiveMarkerViewer.hpp>

static const std::string topicName("dart_markers");

int main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "Please include the path to the mesh directory" << std::endl;
    return 0;
  }

  const std::string mesh_dir = argv[1];
  Cozmo cozmo(mesh_dir);
  
  // Start the RViz viewer.
  std::cout << "Starting ROS node." << std::endl;
  ros::init(argc, argv, "load_cozmo");

  std::cout << "Starting viewer. Please subscribe to the '" << topicName
	    << "' InteractiveMarker topic in RViz." << std::endl;

  aikido::rviz::InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(cozmo.getCozmoSkeleton());
  viewer.setAutoUpdate(true);

  double input;
  do {
    std::cout << "\nEnter forklift position (0-0.86 radians, -1 to quit): ";
    std::cin >> input;
    if (input > 0.86 || input < 0) {
      std::cout << "\nThis value exceeds the joint limits, please enter a valid position" << std::endl;
      continue;
    }
    cozmo.setPosition(input);
  } while (input != -1.0);

  return 0;
  
  ros::spin();
}
