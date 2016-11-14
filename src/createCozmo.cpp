#include "cozmo/cozmo.hpp"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "ros/ros.h"
#include <aikido/rviz/InteractiveMarkerViewer.hpp>

// using WorldPtr = dart::simulation::WorldPtr;
static const std::string topicName("dart_markers");

int main(int argc, char* argv[])
{
  dart::dynamics::SkeletonPtr coz = cozmo::createCubeCozmo();

  // Start the RViz viewer.
  std::cout << "Starting ROS node." << std::endl;
  ros::init(argc, argv, "load_cozmo");

  std::cout << "Starting viewer. Please subscribe to the '" << topicName
	    << "' InteractiveMarker topic in RViz." << std::endl;
  aikido::rviz::InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(coz);
  viewer.setAutoUpdate(true);

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();

  // DART crappy viewer
  // WorldPtr world(new dart::simulation::World);
  // world->addSkeleton(coz);

  // dart::gui::SimWindow win;
  // win.setWorld(world);

  // glutInit(&argc, argv);
  // win.initWindow(640, 480, "Cube Cozmo");
  // glutMainLoop();
}
