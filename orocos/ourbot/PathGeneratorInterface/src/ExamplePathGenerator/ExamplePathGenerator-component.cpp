#include "ExamplePathGenerator-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>


ExamplePathGenerator::ExamplePathGenerator(std::string const& name) : PathGeneratorInterface(name), _even(true){

}

bool ExamplePathGenerator::initialize(){
  _path_length = getPathLength();
  return true;
}

bool ExamplePathGenerator::pathUpdate(){
  // Get current pose
  std::vector<double> est_pose = getEstPose();

  // Dummy implementation of path updater
  std::vector<double> ref_pose_x(_path_length, 0.0);
  std::vector<double> ref_pose_y(_path_length, 0.0);
  std::vector<double> ref_pose_t(_path_length, 0.0);
  std::vector<double> ref_ffw_x(_path_length, 0.0);
  std::vector<double> ref_ffw_y(_path_length, 0.0);
  std::vector<double> ref_ffw_t(_path_length, 0.0);

  if(_even){
    for (int i=0; i<_path_length; i++){
      ref_pose_x.at(i) = i;
      ref_pose_y.at(i) = i;
      ref_pose_t.at(i) = i;
      ref_ffw_x.at(i) = i;
      ref_ffw_y.at(i) = i;
      ref_ffw_t.at(i) = i;
    }
    _even = false;
  }
  else{
    for (int i=0; i<_path_length; i++){
      ref_pose_x.at(i) = i + 10;
      ref_pose_y.at(i) = i + 10;
      ref_pose_t.at(i) = i + 10;
      ref_ffw_x.at(i) = i + 10;
      ref_ffw_y.at(i) = i + 10;
      ref_ffw_t.at(i) = i + 10;
    }
    _even = true;
  }

  // Write reference
  setRefPosePath(ref_pose_x, ref_pose_y, ref_pose_t);
  setRefFfwPath(ref_ffw_x, ref_ffw_y, ref_ffw_t);

  return true;
}

ORO_LIST_COMPONENT_TYPE(ExamplePathGenerator);
