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

  static const std::string topicName("dart_markers");

  // Start the RViz viewer.
  std::cout << "Starting ROS node." << std::endl;
  ros::init(argc, argv, "load_cozmo");

  std::cout << "Starting viewer. Please subscribe to the '" << topicName
	    << "' InteractiveMarker topic in RViz." << std::endl;

  aikido::rviz::InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(skeleton);
  viewer.setAutoUpdate(true);

  libcozmo::Waypoint w1;
  w1.x = .3;
  w1.y = .2;
  w1.th = M_PI/2;
  w1.t = 1;

  libcozmo::Waypoint w2;
  w2.x = .3;
  w2.y = .3;
  w2.th = M_PI/2;
  w2.t = 3;

  libcozmo::Waypoint w3;
  w3.x = .3;
  w3.y = .4;
  w3.th = M_PI;
  w3.t = 5;

  std::vector<libcozmo::Waypoint> waypoints;
  waypoints.push_back(w1);
  waypoints.push_back(w2);
  waypoints.push_back(w3);

  std::shared_ptr<Interpolated> traj;
  traj = cozmo.createInterpolatedTraj(waypoints);

  std::chrono::milliseconds period(6);
  cozmo.executeTrajectory(period, traj);

  return 0;
}
