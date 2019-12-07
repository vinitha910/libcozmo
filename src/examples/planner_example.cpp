#include "cozmo_description/cozmo.hpp"
#include "ros/ros.h"
#include <chrono>
#include <cstdlib>
#include <math.h>
#include "aikido/trajectory/Interpolated.hpp"
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include "aikido/rviz/FrameMarker.hpp"
#include "planner/Dijkstra.hpp"
#include "distance/SE2.hpp"

using Interpolated = aikido::trajectory::Interpolated;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("base_link");

int main(int argc, char* argv[])
{
    if (argc < 2) {
      std::cerr << "Please include the path to the mesh directory" << std::endl;
      return 0;
    }

    // Start the RViz viewer.
    std::cout << "Starting ROS node." << std::endl;
    ros::init(argc, argv, "load_cozmo");
    ros::NodeHandle nh("~");

    std::cout << "Starting viewer. Please subscribe to the '" << topicName
        << "' InteractiveMarker topic in RViz." << std::endl;

    aikido::rviz::InteractiveMarkerViewer viewer(topicName, baseFrameName);

    const std::string mesh_dir = argv[1];
    const int num_robots = 2;
    std::vector<libcozmo::Cozmo*> robots(num_robots);
    for (int i = 0; i < num_robots; ++i) {
        robots[i] = new libcozmo::Cozmo(mesh_dir);
        viewer.addSkeletonMarker(robots[i]->getCozmoSkeleton());
    } 
    viewer.setAutoUpdate(true);

    auto GAS =
        std::make_shared<libcozmo::actionspace::GenericActionSpace>(
            libcozmo::utils::linspace(0.0, 4.0, 4.0),
            libcozmo::utils::linspace(1.0, 3.0, 3.0),
            8);

    auto SE2 = std::make_shared<libcozmo::statespace::SE2>(10, 8);
    libcozmo::planner::Dijkstra planner(GAS, SE2, 20);
    const int start_id = SE2->get_or_create_state(
        libcozmo::statespace::SE2::State(0, 0, 3));
    const int goal_id = SE2->get_or_create_state(
        libcozmo::statespace::SE2::State(10, 10, 0));
    planner.set_start(start_id);
    planner.set_goal(goal_id);

    std::vector<int> path;
    bool success = planner.solve(&path);

    std::vector<libcozmo::Waypoint> waypoints;
    double t = 0;
    for (const int& state_id : path) {
        const libcozmo::statespace::StateSpace::State* state = SE2->get_state(state_id);
        Eigen::Vector3d cont_state;
        SE2->discrete_state_to_continuous(*state, &cont_state);
        waypoints.push_back(
            {.x = cont_state[0]/100., 
             .y = cont_state[1]/100., 
             .th = cont_state[2],
             .t = t});
        t += 1;
    }

    std::shared_ptr<Interpolated> traj;
    traj = robots[0]->createInterpolatedTraj(waypoints);

    std::chrono::milliseconds period(10);
    robots[0]->executeTrajectory(period, traj);

    for (int i = 0; i < num_robots; ++i) {
        delete robots[i];
    }

    return 0;
}
