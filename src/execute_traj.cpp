#include "cozmo/cozmo.hpp"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "ros/ros.h"
#include <cstdlib>
#include <math.h>
#include "aikido/trajectory/Interpolated.hpp"

int main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "Please include the path to the mesh directory" << std::endl;
    return 0;
  }

  const std::string mesh_dir = argv[1];
  libcozmo::Cozmo cozmo(mesh_dir);

  libcozmo::waypoint w1;
  w1.x = 30;
  w1.y = 0;
  w1.th = 0;
  w1.t = 1;

  libcozmo::waypoint w2;
  w2.x = 30;
  w2.y = 30;
  w2.th = M_PI/2;
  w2.t = 3;

  libcozmo::waypoint w3;
  w3.x = 60;
  w3.y = 60;
  w3.th = M_PI;
  w3.t = 5;

  std::vector<libcozmo::waypoint> waypoints;
  waypoints.push_back(w1);
  waypoints.push_back(w2);
  waypoints.push_back(w2);

  std::shared_ptr<aikido::trajectory::Interpolated> traj;
  traj = cozmo.createInterpolatedTraj(waypoints);

  dart::dynamics::SkeletonPtr skeleton;
  skeleton = cozmo.getCozmoSkeleton();
  return 0;
}
