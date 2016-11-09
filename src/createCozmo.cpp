#include "cozmo/cozmo.hpp"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"

using WorldPtr = dart::simulation::WorldPtr;

int main(int argc, char* argv[])
{
  dart::dynamics::SkeletonPtr coz = cozmo::createCubeCozmo();
  WorldPtr world(new dart::simulation::World);
  world->addSkeleton(coz);

  dart::gui::SimWindow win;
  win.setWorld(world);

  glutInit(&argc, argv);
  win.initWindow(640, 480, "Cube Cozmo");
  glutMainLoop();
}
