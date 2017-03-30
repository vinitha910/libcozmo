#include "cozmo/cozmo.hpp"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "ros/ros.h"
#include <chrono>
#include <cstdlib>
#include <math.h>
#include "aikido/trajectory/Interpolated.hpp"
#include <aikido/rviz/InteractiveMarkerViewer.hpp>

using Interpolated = aikido::trajectory::Interpolated;

int main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "Please include the path to the mesh directory" << std::endl;
    return 0;
  }

  const std::string mesh_dir = argv[1];
  libcozmo::Cozmo cozmo(mesh_dir);

  dart::dynamics::SkeletonPtr skeleton;
  skeleton = cozmo.getCozmoSkeleton();

  //visualizer::Viz viz(cozmo, mesh_dir, argc, argv);
  static const std::string topicName("dart_markers");

  // Start the RViz viewer.
  std::cout << "Starting ROS node." << std::endl;
  ros::init(argc, argv, "load_cozmo");

  std::cout << "Starting viewer. Please subscribe to the '" << topicName
	    << "' InteractiveMarker topic in RViz." << std::endl;

  aikido::rviz::InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(skeleton);
  viewer.setAutoUpdate(true);

  std::cout << "creating waypoints" << std::endl;
  libcozmo::Waypoint *w1 = new libcozmo::Waypoint;
  w1->x = 30;
  w1->y = 0;
  w1->th = 0;
  w1->t = 1;

  libcozmo::Waypoint *w2 = new libcozmo::Waypoint;
  w2->x = 30;
  w2->y = 30;
  w2->th = M_PI/2;
  w2->t = 3;

  libcozmo::Waypoint *w3 = new libcozmo::Waypoint;
  w3->x = 60;
  w3->y = 60;
  w3->th = M_PI;
  w3->t = 5;

  std::cout << "created waypoints" << std::endl;

  std::cout << "adding waypoints to list" << std::endl;
  std::vector<libcozmo::Waypoint> waypoints;
  waypoints.push_back(*w1);
  waypoints.push_back(*w2);
  waypoints.push_back(*w3);
  std::cout << "added waypoints to list" << std::endl;

  std::cout << "creating traj" << std::endl;
  std::shared_ptr<Interpolated> traj;
  traj = cozmo.createInterpolatedTraj(waypoints);
  std::cout << "created traj" << std::endl;

  std::cout << "executing traj" << std::endl;
  std::chrono::milliseconds period(200);
  cozmo.executeTrajectory(skeleton, period, traj);
  std::cout << "executed traj" << std::endl;

  return 0;
}
