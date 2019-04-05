#include "cozmo_description/cozmo.hpp"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"

using WorldPtr = dart::simulation::WorldPtr;

int main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "Please include the path to the mesh directory" << std::endl;
    return 0;
  }

  const std::string mesh_dir = argv[1];
  libcozmo::Cozmo cozmo(mesh_dir);

  dart::dynamics::SkeletonPtr coz = cozmo.getCozmoSkeleton();
  WorldPtr world(new dart::simulation::World);
  world->addSkeleton(coz);
  
  dart::gui::glut::SimWindow win;
  win.setWorld(world);

  glutInit(&argc, argv);
  win.initWindow(900, 600, "Cozmo");
  glutMainLoop();
}
