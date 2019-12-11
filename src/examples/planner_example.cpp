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
#include "libcozmo/GetTrajectory.h"
#include "libcozmo/Done.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

using Interpolated = aikido::trajectory::Interpolated;
using Interpolator = aikido::statespace::Interpolator;
using GeodesicInterpolator = aikido::statespace::GeodesicInterpolator;
using aikido::statespace::SE2;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("base_link");
const Eigen::Vector4d sphere_1(0.03249, -0.001, 0.03391, 0.08);
const Eigen::Vector4d sphere_2(-0.00861, -0.001, 0.03391, 0.08);
const Eigen::Vector4d collision_thresh(0.01144, 0.0, 0.04252, 0.119);

// Map priority -> service
std::unordered_map<int, ros::ServiceServer> receivers;
std::vector<ros::ServiceClient> senders;
std::vector<ros::ServiceServer> complete_receivers;
std::vector<ros::ServiceClient> complete_senders;

// Map time -> (x,y)
std::unordered_map<double, std::vector<std::pair<int, int>>> dynamic_obstacles = {};
std::unordered_set<int> complete = {};
std::vector<int> dependency_list;
std::vector<std::shared_ptr<Interpolated>> path_cache;
int priority;

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

bool collision(Eigen::Vector3d& r1, Eigen::Vector3d& r2)
{       
    const double dx = (r1.x() - r2.x()) * (r1.x() - r2.x());
    const double dy = (r1.y() - r2.y()) * (r1.y() - r2.y());
    return std::sqrt(dx + dy) <= 0.119;
}

Eigen::Vector3d get_state(
    const std::shared_ptr<Interpolated> path, const double& time) {
    auto space = path->getStateSpace();
    auto scopedState = space->createState();
    path->evaluate(time, scopedState);
    const auto state = 
        static_cast<aikido::statespace::SE2::State*>(scopedState.getState());
    const auto transform = state->getIsometry();
    Eigen::Rotation2Dd rotation = Eigen::Rotation2Dd::Identity();
    rotation.fromRotationMatrix(transform.rotation());
    const auto translation = transform.translation();
    return Eigen::Vector3d(translation.x(), translation.y(), rotation.angle());
}

bool collision_check(const std::shared_ptr<Interpolated> path) {
    auto space = path->getStateSpace();
    std::vector<double> times = 
        libcozmo::utils::linspace(
            path->getStartTime(), 
            path->getEndTime(), 
            floor(path->getDuration()) + 1);

    bool in_collision = false;
    for (const double time : times) {
        Eigen::Vector3d s1 = get_state(path, time);

        for (const auto cached_path : path_cache) {
            Eigen::Vector3d s2 = get_state(cached_path, time);

            for (double dt = 1.0; dt >= -1.0; dt -= 0.5) {
                // std::cout << dt << std::endl;
                auto iter = dynamic_obstacles.find(time + dt);
                if (iter == dynamic_obstacles.end()) {
                    dynamic_obstacles[time + dt] = {std::pair<int, int>(s2.x()*100, s2.y()*100)};
                } else {
                    dynamic_obstacles[time + dt].push_back(std::pair<int, int>(s2.x()*100, s2.y()*100));                    
                }
            }
            
            if (collision(s1, s2)) {
                in_collision = true;
                // std::cout << "Collision at " << s1.x()*100 << " " << s1.y()*100 << " " << s1[2] << " " << time << std::endl;
            }
        }    
    }
    return in_collision;
}

SE2::State createState(const double x, const double y, const double th) 
{
    SE2::State s;
    Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
    const Eigen::Rotation2D<double> rot(th);
    t.linear() = rot.toRotationMatrix();
    Eigen::Vector2d trans;
    trans << x, y;
    t.translation() = trans;
    s.setIsometry(t);
    return s;
}
  
std::shared_ptr<Interpolated> createInterpolatedTraj(
  std::vector<libcozmo::Waypoint> waypoints) 
{
    std::shared_ptr<SE2> statespace = std::make_shared<SE2>();
    std::shared_ptr<Interpolator> interpolator = 
      std::make_shared<GeodesicInterpolator>(statespace);
   
    const int num_waypoints = waypoints.size();
    libcozmo::Waypoint w;

    SE2::State s;
    std::shared_ptr<Interpolated> traj;
    traj = std::make_shared<Interpolated>(statespace, interpolator);

    for (int i=0; i < num_waypoints; i++) {    
      w = waypoints.at(i);
      s = createState(w.x, w.y, w.th);
      traj->addWaypoint(w.t, &s);
    }

    return traj;
}

void update_path_cache(const nav_msgs::Path& path) {
    std::vector<libcozmo::Waypoint> waypoints;
    for (const auto pose : path.poses) {
        double yaw = tf::getYaw(pose.pose.orientation);
        waypoints.push_back({
            .x = pose.pose.position.x,
            .y = pose.pose.position.y,
            .th = yaw,
            .t = pose.header.stamp.toSec()
        });
    }

    path_cache.push_back(createInterpolatedTraj(waypoints));
}

bool ack_traj(
    libcozmo::GetTrajectory::Request &req, 
    libcozmo::GetTrajectory::Response &res)
{
    // Send back priority of robot that received the message
    res.receiver_priority = priority;

    // Store the received path in the path cache
    update_path_cache(req.path);
    dependency_list.push_back(req.sender_priority);

    ROS_INFO("Message from robot #%i received by #%i", req.sender_priority, priority);
    return true;
}

bool ack_done(
    libcozmo::Done::Request &req, 
    libcozmo::Done::Response &res)
{
    // Ack message
    res.receiver_priority = priority;

    // Get the priority of the robot that has finished planning
    complete.insert(req.sender_priority); 
    receivers[req.sender_priority].shutdown();
    receivers.erase(req.sender_priority);

    // ROS_INFO("Message from robot #%i received by #%i", req.sender_priority, priority);
    return true;
}

nav_msgs::Path generate_traj(std::vector<libcozmo::Waypoint>& waypoints)
{
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    for (const auto waypoint : waypoints) {
        pose.pose.position.x = waypoint.x;
        pose.pose.position.y = waypoint.y;
        pose.pose.position.z = 0.0;

        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, waypoint.th);
        pose.pose.orientation.x = quat[0];
        pose.pose.orientation.y = quat[1];
        pose.pose.orientation.z = quat[2];
        pose.pose.orientation.w = quat[3];

        pose.header.stamp = ros::Time(waypoint.t);
        path.poses.push_back(pose);
    }
    return path;    
}

bool send_traj(ros::ServiceClient client, const nav_msgs::Path& path)
{
    libcozmo::GetTrajectory srv;
    srv.request.sender_priority = priority;
    srv.request.path = path;

    ROS_INFO("Robot #%i sending trajectory message", priority);
    bool sent = false;
    while (!sent) {
        sent = client.call(srv);
    }
    ROS_INFO("Trajectory received by robot #%i", srv.response.receiver_priority);

    // if (client.call(srv)) {
    //     ROS_INFO("Trajectory received by robot #%i", srv.response.receiver_priority);
    // } else { 
    //     ROS_INFO("Robot #%i failed to send trajectory", priority);
    //     return false;
    // }

    return true;
}

bool send_complete(ros::ServiceClient client)
{
    libcozmo::Done srv;
    srv.request.sender_priority = priority;

    bool sent = false;
    while (!sent) {
        sent = client.call(srv);
    }
    ROS_INFO("Robot #%i's complete message received by %i", priority, srv.response.receiver_priority);

    // if (sent) {
    //     ROS_INFO("Robot #%i's complete message received by %i", priority, srv.response.receiver_priority);
    // } else { 
    //     ROS_INFO("Robot #%i failed to send complete message", priority);
    //     return false;
    // }

    return true;
}

void set_start_goal(
    ros::NodeHandle nh, 
    std::shared_ptr<libcozmo::statespace::SE2> SE2,
    libcozmo::planner::Dijkstra& planner) 
{
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
}

void plan(
    ros::NodeHandle nh,
    std::shared_ptr<libcozmo::statespace::SE2> SE2,
    libcozmo::planner::Dijkstra& planner,
    libcozmo::Cozmo& cozmo,
    std::vector<libcozmo::Waypoint>* waypoints) 
{
    set_start_goal(nh, SE2, planner);

    std::vector<int> path;
    bool success = planner.solve(&path);

    for (const int& state_id : path) {
        const libcozmo::statespace::StateSpace::State* state = SE2->get_state(state_id);
        Eigen::Vector4d cont_state;
        SE2->discrete_state_to_continuous(*state, &cont_state);
        waypoints->push_back(
            {.x = cont_state[0]/100., 
             .y = cont_state[1]/100., 
             .th = cont_state[2],
             .t = cont_state[3]});
        // std::cout << cont_state[0] << " " << cont_state[1] << " " << cont_state[2] << " " << cont_state[3] << std::endl;
    }

    // Send plan to robots with lower priority
    nav_msgs::Path traj_msg = generate_traj(*waypoints);
    for (const auto sender : senders) {
        send_traj(sender, traj_msg);
    }

    // Wait till all messages from robots in dependency list are received
    while (dependency_list.size() < receivers.size()) {
        ros::spinOnce();
    }
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
    
    nh.getParam("priority", priority);

    const int num_robots = 5;

    for (int i = 1; i <= num_robots; ++i) {
        if (i == priority) continue;
        std::string service = 
            "/group_" + std::to_string(i) + "_" + std::to_string(priority);
        std::cout << service << std::endl;
        complete_receivers.push_back(nh.advertiseService(service + "/complete", ack_done));
        if (i < priority) {
            receivers[i] = nh.advertiseService(service + "/trajectory", ack_traj);
        }
    }

    for (int i = 1; i <= num_robots; ++i) {
        if (i == priority) continue;
        std::string client = 
            "/group_" + std::to_string(priority) + "_" + std::to_string(i);
        // std::cout << client << std::endl;
        ros::service::waitForService(client + "/complete");
        complete_senders.push_back(nh.serviceClient<libcozmo::Done>(client + "/complete"));
        if (i > priority) {
            ros::service::waitForService(client + "/trajectory");
            senders.push_back(nh.serviceClient<libcozmo::GetTrajectory>(client + "/trajectory"));
        }
    }

    std::cout << "Starting viewer. Please subscribe to the '" << topicName
        << "' InteractiveMarker topic in RViz." << std::endl;

    aikido::rviz::InteractiveMarkerViewer viewer(topicName, baseFrameName);

    const std::string mesh_dir = argv[1];
    libcozmo::Cozmo cozmo = libcozmo::Cozmo(mesh_dir);
    viewer.addSkeletonMarker(cozmo.getCozmoSkeleton());
    viewer.setAutoUpdate(true);

    auto GAS =
        std::make_shared<libcozmo::actionspace::GenericActionSpace>(
            libcozmo::utils::linspace(0.0, 8.0, 8.0),
            libcozmo::utils::linspace(0.5, 1.0, 2.0),
            8);

    auto SE2 = std::make_shared<libcozmo::statespace::SE2>(10, 8);
    libcozmo::planner::Dijkstra planner(GAS, SE2, 20);

    std::vector<libcozmo::Waypoint> waypoints;
    plan(nh, SE2, planner, cozmo, &waypoints);
    std::shared_ptr<Interpolated> traj = 
        cozmo.createInterpolatedTraj(waypoints);
    bool in_collision = collision_check(traj);

    while (in_collision || receivers.size() > 0) {
        dependency_list.clear();

        // Update the obstacle map and replan
        SE2->update_obstacle_map(dynamic_obstacles);
        path_cache.clear();

        waypoints.clear();
        plan(nh, SE2, planner, cozmo, &waypoints);

        // Check if planned path is in collision with robots that have a higher 
        // priority
        traj = cozmo.createInterpolatedTraj(waypoints);
        in_collision = collision_check(traj);
    }

    // Tell all other robots that you have finished planning
    for (const auto sender : complete_senders) {
        send_complete(sender);
    }
    // for (auto sender : senders) {
    //     sender.shutdown();
    // }
    // senders.clear();

    ROS_INFO("Robot #%i done", priority);

    while (complete.size() < num_robots - 1) {
        ros::spinOnce();
    }

    // traj = cozmo.createInterpolatedTraj(waypoints);
    std::chrono::milliseconds period(2);
    cozmo.executeTrajectory(period, traj);
    
    ros::spin();
    return 0;
}
