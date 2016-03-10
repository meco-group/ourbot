#include "SimpleEstimator-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

SimpleEstimator::SimpleEstimator(std::string const& name) : EstimatorInterface(name){

}

bool SimpleEstimator::initialize(){
  return true;
}

bool SimpleEstimator::estimateUpdate(){
  std::vector<double> enc_pose = getEncPose();
  setEstPose(enc_pose);
  return true;
}

ORO_LIST_COMPONENT_TYPE(SimpleEstimator);
