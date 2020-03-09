#ifndef COZMO_COZMO_HPP_
#define COZMO_COZMO_HPP_

#include "ros/ros.h"
#include "dart/dart.hpp"
#include <chrono>
#include "aikido/trajectory/Trajectory.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/statespace/SE2.hpp"

namespace libcozmo {
using BodyNodePtr = dart::dynamics::BodyNodePtr;
using SkeletonPtr = dart::dynamics::SkeletonPtr;
using InverseKinematicsPtr = dart::dynamics::InverseKinematicsPtr;
using TrajectoryPtr = aikido::trajectory::TrajectoryPtr;
using Interpolated = aikido::trajectory::Interpolated;
using aikido::statespace::SE2;
  
class Waypoint 
{
public:
    double x;
    double y;
    double th;
    double t;
};

class Cozmo
{
public:
    /// Creates Cozmo object
    ///
    /// \param mesh_dir path to the libcozmo/meshes folder
    Cozmo(const std::string& mesh_dir);
   
    ~Cozmo() {};

    /// Returns SkeletonPtr to cozmo
    /// Though Cozmo only has a 1 DOF forklift, it is modelled as 6 DOF due
    /// to the inability to model the 4 bar linkage. Therefore, setting
    /// joint positions on this skeleton will NOT have the expected result.
    SkeletonPtr getCozmoSkeleton() { return cozmo; };
    
    /// Takes in pos (angle in radians) and sets forklift position
    void setForkliftPosition(double pos);
    
    /// Executes a trajectory defined by a set of waypoints 
    ///
    /// \param period The period of the trajectory 
    /// \param traj The Trajectory Pointer to the trajectory that needs to be executed
    void executeTrajectory(
        std::chrono::milliseconds period, TrajectoryPtr traj);

    void setState(const double& x, const double& y, const Eigen::Quaterniond& orientation);

    /// Returns the (x, y, theta) of cozmo at time t
    ///
    /// \param path Cozmo's trajectory
    /// \param time The time (ms)
    Eigen::Vector3d getState(
        const std::shared_ptr<Interpolated> path, const double& time);
    
    /// Return and SE2 State defined by the inputted x, y and theta
    ///
    /// \param x The x coordinate of the state
    /// \param y The y coordinate of the state
    /// \param th The rotation theta of the state
    SE2::State createState(double x, double y, double th);

    /// Creates an interpolated trajectory given a set of waypoints
    ///
    /// \param waypoints A vector of waypoints
    std::shared_ptr<Interpolated> createInterpolatedTraj(std::vector<Waypoint> waypoints);
 
private:
    /// SkeletonPtr to Cozmo
    SkeletonPtr cozmo;

    // BodyNodePtr to various links on Cozmo skeleton
    BodyNodePtr head;
    BodyNodePtr base;
    BodyNodePtr forklift;
    BodyNodePtr ghost_strut;
    BodyNodePtr lower_forklift_strut_left;
    BodyNodePtr lower_forklift_strut_right;
    BodyNodePtr upper_forklift_strut_left;
    BodyNodePtr upper_forklift_strut_right;

    /// InverseKinematicsPtr to Inverse Kinematics Module
    InverseKinematicsPtr ik;

    /// Creates and returns a BodyNodePtr to Cozmo's base
    ///
    /// \param cozmo SkeletonPtr to cozmo 
    /// \param mesh_name name of the mesh file without the extension
    /// \param mesh_dir path to the libcozmo/meshes folder
    BodyNodePtr makeRootBody(const SkeletonPtr& cozmo,
  			   const std::string& mesh_name,
  			   const std::string& mesh_dir);

    /// Helper method to create an individual link to the base; returns BodyNodePtr to body
    ///
    /// \param cozmo SkeletonPtr to cozmo 
    /// \param parent parent body node
    /// \param mesh_name name of the mesh file without the extension
    /// \param mesh_dir path to the libcozmo/meshes folder
    /// \param transformFromParent body's transform from parent body node
    /// \param transformFromChild body's transform from child body node 
    BodyNodePtr addBody(const SkeletonPtr& cozmo,
  		      BodyNodePtr parent,
  		      const std::string& mesh_name,
  		      const std::string& mesh_dir,
  	              Eigen::Vector3d transformFromParent,
  		      Eigen::Vector3d transformFromChild);

    /// Creates and returns SkeletonPtr to Cozmo skeleton
    ///
    /// \param mesh_dir path to the libcozmo/meshes folder
    SkeletonPtr createCozmo(const std::string& mesh_dir);

    /// Creates IK Module on ghost strut
    void createIKModule();
};
}
#endif  // COZMO_COZMO_HPP_
