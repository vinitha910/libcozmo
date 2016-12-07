#include "cozmo/cozmo.hpp"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "ros/ros.h"
#include <aikido/rviz/InteractiveMarkerViewer.hpp>

static const std::string topicName("dart_markers");

int main(int argc, char* argv[])
{
  dart::simulation::WorldPtr world = cozmo::createCozmo();
  dart::dynamics::SkeletonPtr coz = world->getSkeleton("cozmo");

  // Start the RViz viewer.
  std::cout << "Starting ROS node." << std::endl;
  ros::init(argc, argv, "load_cozmo");

  std::cout << "Starting viewer. Please subscribe to the '" << topicName
	    << "' InteractiveMarker topic in RViz." << std::endl;
  aikido::rviz::InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(coz);
  viewer.setAutoUpdate(true);

  std::cout << "Press <Ctrl> + C to exit." << std::endl;

  std::cin.get();

  coz->getBodyNode("lower_forklift_strut_right_1")->getParentJoint()->setPosition(0, M_PI/4);
  coz->getBodyNode("upper_forklift_strut_right_1")->getParentJoint()->setPosition(0, M_PI/4 + 0.08);
  coz->getBodyNode("lower_forklift_strut_left_1")->getParentJoint()->setPosition(0, M_PI/4);
  coz->getBodyNode("upper_forklift_strut_left_1")->getParentJoint()->setPosition(0, M_PI/4 + 0.08);
  
  std::cin.get();

  world->getConstraintSolver()->solve();
  
  ros::spin();
}
