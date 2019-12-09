#include "cozmo_description/cozmo.hpp"
#include "ros/ros.h"
#include <chrono>
#include <cstdlib>
#include <math.h>
#include <stdlib.h>  
#include "aikido/trajectory/Interpolated.hpp"
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include "aikido/rviz/FrameMarker.hpp"
#include "planner/Dijkstra.hpp"
#include "distance/SE2.hpp"

using Interpolated = aikido::trajectory::Interpolated;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("base_link");
const Eigen::Vector4d sphere_1(0.03249, -0.001, 0.03391, 0.08);
const Eigen::Vector4d sphere_2(-0.00861, -0.001, 0.03391, 0.08);
const Eigen::Vector4d collision_thresh(0.01144, 0.0, 0.04252, 0.119);

Eigen::Vector2d transform_spheres(
    Eigen::Vector3d& robot_pose, Eigen::Vector4d& sphere_pose)
{
    const Eigen::Vector2d sphere =
            Eigen::Translation2d(robot_pose.x(), robot_pose.y()) * \
            Eigen::Rotation2Dd(robot_pose.w()) * \
            Eigen::Vector2d(sphere_pose.x(), sphere.y());
    return sphere;
}

bool in_collision(
    Eigen::Vector3d& robot1_pose, 
    Eigen::Vector3d& robot2_pose,
    Eigen::Vector4d& sphere)
{
    Eigen::Vector2d s1 = transform_spheres(robot1_pose, sphere);
    Eigen::Vector2d s2 = transform_spheres(robot2_pose, sphere);
    const double dx = (s1.x() - s2.x()) * (s1.x() - s2.x());
    const double dy = (s1.y() - s2.y()) * (s1.y() - s2.y());
    return std::sqrt(dx + dy) < sphere[3] ? true : false;
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
      std::cerr << "Please include the path to the mesh directory" << std::endl;
      return 0;
    }

    // Start the RViz viewer.
    std::cout << "Starting ROS node." << std::endl;
    ros::init(argc, argv, "load_cozmo");
    ros::NodeHandle nh;

    std::cout << "Starting viewer. Please subscribe to the '" << topicName
        << "' InteractiveMarker topic in RViz." << std::endl;

    aikido::rviz::InteractiveMarkerViewer viewer(topicName, baseFrameName);

    const std::string mesh_dir = argv[1];
    libcozmo::Cozmo cozmo = libcozmo::Cozmo(mesh_dir);
    viewer.addSkeletonMarker(cozmo.getCozmoSkeleton());
    viewer.setAutoUpdate(true);

    auto GAS =
        std::make_shared<libcozmo::actionspace::GenericActionSpace>(
            libcozmo::utils::linspace(0.0, 4.0, 4.0),
            libcozmo::utils::linspace(1.0, 3.0, 3.0),
            8);

    auto SE2 = std::make_shared<libcozmo::statespace::SE2>(10, 8);
    libcozmo::planner::Dijkstra planner(GAS, SE2, 20);

    std::map<std::string, double> start;
    nh.getParam("start_pose", start);
    const int start_id = SE2->get_or_create_state(
        Eigen::Vector4d(start["x_mm"], start["y_mm"], start["theta_rad"], 0.0), true);
    planner.set_start(start_id);

    std::map<std::string, double> goal;
    nh.getParam("goal_pose", goal);
    const int goal_id = SE2->get_or_create_state(
        Eigen::Vector4d(goal["x_mm"], goal["y_mm"], goal["theta_rad"], 10.0), true);
    planner.set_goal(goal_id);

    std::vector<int> path;
    bool success = planner.solve(&path);

    std::vector<libcozmo::Waypoint> waypoints;
    for (const int& state_id : path) {
        const libcozmo::statespace::StateSpace::State* state = SE2->get_state(state_id);
        Eigen::Vector4d cont_state;
        SE2->discrete_state_to_continuous(*state, &cont_state);
        waypoints.push_back(
            {.x = cont_state[0]/100., 
             .y = cont_state[1]/100., 
             .th = cont_state[2],
             .t = cont_state[3]});
        std::cout << cont_state[0] << " " << cont_state[1] << " " << cont_state[2] << " " << cont_state[3] << std::endl;
    }

    std::shared_ptr<Interpolated> traj;
    traj = cozmo.createInterpolatedTraj(waypoints);

    std::chrono::milliseconds period(10);
    cozmo.executeTrajectory(period, traj);

    return 0;
}
