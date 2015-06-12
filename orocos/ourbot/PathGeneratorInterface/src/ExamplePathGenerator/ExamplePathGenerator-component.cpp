#include "ExamplePathGenerator-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>


ExamplePathGenerator::ExamplePathGenerator(std::string const& name) : PathGeneratorInterface(name){

}

bool ExamplePathGenerator::initialize(){
  return true;
}

bool ExamplePathGenerator::pathUpdate(){
  // Get current pose
  std::vector<double> est_pose = getEstPose();

  // Dummy implementation of path updater
  std::vector<double> ref_pose_x(getPathLength(), 0.0);
  std::vector<double> ref_pose_y(getPathLength(), 0.0);
  std::vector<double> ref_pose_t(getPathLength(), 0.0);
  std::vector<double> ref_ffw_x(getPathLength(), 0.0);
  std::vector<double> ref_ffw_y(getPathLength(), 0.0);
  std::vector<double> ref_ffw_t(getPathLength(), 0.0);

  // Write reference
  setRefPosePath(ref_pose_x, ref_pose_y, ref_pose_t);
  setRefFfwPath(ref_ffw_x, ref_ffw_y, ref_ffw_t);

  return true;
}

ORO_CREATE_COMPONENT(ExamplePathGenerator);
