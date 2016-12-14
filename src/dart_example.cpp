#include "cozmo/cozmo.hpp"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"

using WorldPtr = dart::simulation::WorldPtr;

int main(int argc, char* argv[])
{
  dart::dynamics::FreeJoint::Properties properties;
  properties.mName = "_joint";
  dart::dynamics::SkeletonPtr cube;
  cube = dart::dynamics::Skeleton::create("cube"); 
  dart::dynamics::BodyNodePtr bn = cube->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
			  nullptr, properties, 
			  dart::dynamics::BodyNode::AspectProperties("cube")).second;

  // Make a shape for the Joint
  std::shared_ptr<dart::dynamics::BoxShape> box(new dart::dynamics::BoxShape(
      Eigen::Vector3d(0.0573, 0.0988, 0.07034)));
  auto shapeNode = bn->createShapeNodeWith<dart::dynamics::VisualAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());




  
  if (argc < 2) {
    std::cerr << "Please include the path to the mesh directory" << std::endl;
    return 0;
  }

  const std::string mesh_dir = argv[1];
  Cozmo cozmo(mesh_dir);

  dart::dynamics::SkeletonPtr coz = cozmo.getCozmoSkeleton();
  WorldPtr world(new dart::simulation::World);
  coz->getBodyNode("body")->getParentJoint()->setPosition(0, 0);
  coz->getBodyNode("body")->getParentJoint()->setPosition(1, 0);
  coz->getBodyNode("body")->getParentJoint()->setPosition(2, 0);
  
  world->addSkeleton(coz);
  world->addSkeleton(cube);
  
  dart::gui::SimWindow win;
  win.setWorld(world);

  glutInit(&argc, argv);
  win.initWindow(900, 600, "Cozmo");
  glutMainLoop();
}
