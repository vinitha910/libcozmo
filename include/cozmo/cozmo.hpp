#ifndef COZMO_COZMO_HPP_
#define COZMO_COZMO_HPP_

#include "dart/dart.hpp"
#include <Python.h>

namespace libcozmo {
using BodyNodePtr = dart::dynamics::BodyNodePtr;
using SkeletonPtr = dart::dynamics::SkeletonPtr;
using InverseKinematicsPtr = dart::dynamics::InverseKinematicsPtr;

class Cozmo
{
public:
  /// Creates Cozmo object

  /// \param mesh_dir path to the libcozmo/meshes folder
  Cozmo(const std::string& mesh_dir);

  /// Frees all memory allocated by the Python Interpreter 
  ~Cozmo();

  /// The PyObject of the instance of the CozmoConnection class 
  PyObject *pConn;

  /// Returns SkeletonPtr to cozmo
  /// Though Cozmo only has a 1 DOF forklift, it is modelled as 6 DOF due
  /// to the inability to model the 4 bar linkage. Therefore, setting
  /// joint positions on this skeleton will NOT have the expected result.
  SkeletonPtr getCozmoSkeleton() { return cozmo; };
  
  /// Takes in pos (angle in radians) and sets forklift position
  void setForkliftPosition(double pos);

  /// Returns pose (x, y, z, angle_z) of robot
  std::vector<double> getPose();

  /// Returns True is robot successfully moves to specified pose; False otherwise

  /// \param pos vector that contains the x, y, z position in mm
  /// \param angle_z z Euler component of the obkect's rotation
  void goToPose(std::vector<double> pos, double angle_z);

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
  
  /// \param cozmo SkeletonPtr to cozmo 
  /// \param mesh_name name of the mesh file without the extension
  /// \param mesh_dir path to the libcozmo/meshes folder
  BodyNodePtr makeRootBody(const SkeletonPtr& cozmo,
			   const std::string& mesh_name,
			   const std::string& mesh_dir);

  /// Helper method to create an individual link to the base; returns BodyNodePtr to body
  
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

  /// \param mesh_dir path to the libcozmo/meshes folder
  SkeletonPtr createCozmo(const std::string& mesh_dir);

  /// Creates IK Module on ghost strut
  void createIKModule();
};
}
#endif  // COZMO_COZMO_HPP_
