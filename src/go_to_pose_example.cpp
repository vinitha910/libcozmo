#include "cozmo/cozmo.hpp"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "ros/ros.h"
#include <cstdlib>
#include <math.h>

bool checkDouble(std::string input)
{
  char* end;
  if (end == input.c_str()) {
    std::cout << "\nPlease enter a valid double." << std::endl;
    return false;
  }
  return true;
}

double checkPosition(double i)
{
  if (i < 0) {
    std::cout << "\nPlease enter a value >= 0." << std::endl;
    return -1.0;
  }
  else { return i; }
}

double checkAngle(double i)
{
  if (-2*M_PI > i || i > 2*M_PI) {
    std::cout << "\nPlease enter a value between -2PI and 2PI." << std::endl;
    return -1.0;
  }
  else { return i; }
}

int main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "Please include the path to the mesh directory" << std::endl;
    return 0;
  }

  const std::string mesh_dir = argv[1];
  libcozmo::Cozmo cozmo(mesh_dir);
  
  std::vector<double> pose;

  std::string input;
  double i = 0;
  char* end;
  do { // TODO: Check validity of input
    std::cout << "\nEnter desired x position in mm (Ctrl+C to quit): ";
    std::cin >> input;
    i = std::strtod(input.c_str(), &end);
    pose.push_back(i);

    std::cout << "\nEnter desired y position in mm (Ctrl+C to quit): ";
    std::cin >> input;
    i = std::strtod(input.c_str(), &end);
    pose.push_back(i);

    std::cout << "\nEnter desired z position in mm (Ctrl+C to quit): ";
    std::cin >> input;
    i = std::strtod(input.c_str(), &end);
    pose.push_back(i);

    std::cout << "\nEnter desired z angle position in radians (Ctrl+C to quit): ";
    std::cin >> input;
    i = std::strtod(input.c_str(), &end);

    cozmo.goToPose(pose, i);
    pose.clear();
  } while (i != -1.0);

  return 0;
}
