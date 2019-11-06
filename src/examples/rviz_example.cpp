#include "cozmo_description/cozmo.hpp"
#include "ros/ros.h"
#include <cstdlib>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

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

    std::cout << "Starting viewer. Please subscribe to the '" << topicName
        << "' InteractiveMarker topic in RViz." << std::endl;

    // Start Visualization Topic
    static const std::string execTopicName = topicName + "/forklift_sim";

    aikido::rviz::InteractiveMarkerViewer viewer(topicName, baseFrameName);
    viewer.addSkeletonMarker(cozmo.getCozmoSkeleton());
    viewer.setAutoUpdate(true);

    std::string input = "";
    double i = 0;
    do {
        std::cout << "\nEnter forklift position (0-0.86 radians, -1 to quit): ";
        std::cin >> input;
        char* end;
        i = std::strtod(input.c_str(), &end);
        if (end == input.c_str()) {
            std::cout << "\nPlease enter a valid double." << std::endl;
            continue;
        }
        if (i > 0.86 || i < 0) {
            std::cout << "\nThis value exceeds the joint limits, please enter a valid position." << std::endl;
            continue;
        }
        cozmo.setForkliftPosition(i);
    } while (i != -1.0);

    ros::shutdown();
    return 0;
}
