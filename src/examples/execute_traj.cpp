#include "cozmo_description/cozmo.hpp"
#include "ros/ros.h"
#include <chrono>
#include <cstdlib>
#include <math.h>
#include "aikido/trajectory/Interpolated.hpp"
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include "aikido/rviz/FrameMarker.hpp"

using Interpolated = aikido::trajectory::Interpolated;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("base_link");

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
  ros::NodeHandle nh("~");

  dart::dynamics::SkeletonPtr skeleton;
  skeleton = cozmo.getCozmoSkeleton();

  std::cout << "Starting viewer. Please subscribe to the '" << topicName
	    << "' InteractiveMarker topic in RViz." << std::endl;

  aikido::rviz::InteractiveMarkerViewer viewer(topicName, baseFrameName);
  viewer.addSkeletonMarker(skeleton);

  auto body = skeleton->getBodyNode(0);
  // Create x-axis
  auto frameShapeFrameX = dart::dynamics::SimpleFrame(body, "axis x");
  auto arrowX = std::make_shared<dart::dynamics::ArrowShape>(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  frameShapeFrameX.setShape(arrowX);

  auto frameShapeFrameY = dart::dynamics::SimpleFrame(body, "axis y");
  auto arrowY = std::make_shared<dart::dynamics::ArrowShape>(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY());
  frameShapeFrameY.setShape(arrowY);

  auto frameShapeFrameZ = dart::dynamics::SimpleFrame(body, "axis z");
  auto arrowZ = std::make_shared<dart::dynamics::ArrowShape>(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());
  frameShapeFrameZ.setShape(arrowZ);
  viewer.addFrame(&frameShapeFrameX);
  viewer.addFrame(&frameShapeFrameY);
  viewer.addFrame(&frameShapeFrameZ);
  // Eigen::Isometry3d tf = frame->getWorldTransform();
  // auto world = dart::dynamics::Frame::World();
  // dart::dynamics::SimpleFrame simple_frame(world, "target", tf);
  // viewer.addFrame(&simple_frame);

  viewer.setAutoUpdate(true);

  std::vector<libcozmo::Waypoint> waypoints = {
      {.x = 0.0, .y = 0.0, .th = 0, .t = 0},
      {.x = 0.3, .y = 0.0, .th = 0, .t = 2},
      {.x = 0.3, .y = 0.0, .th = M_PI/2, .t = 3},
      {.x = 0.3, .y = 0.3, .th = M_PI/2, .t = 5},
      {.x = 0.3, .y = 0.3, .th = M_PI, .t = 6},
      {.x = 0.0, .y = 0.3, .th = M_PI, .t = 8},
      {.x = 0.0, .y = 0.3, .th = -M_PI/2, .t = 9},
      {.x = 0.0, .y = 0.0, .th = -M_PI/2, .t = 11},
      {.x = 0.0, .y = 0.0, .th = 0, .t = 12}
  };

  std::shared_ptr<Interpolated> traj;
  traj = cozmo.createInterpolatedTraj(waypoints);

  std::chrono::milliseconds period(6);
  cozmo.executeTrajectory(period, traj);

  return 0;
}
