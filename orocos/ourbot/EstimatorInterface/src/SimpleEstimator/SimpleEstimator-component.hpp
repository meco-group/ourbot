#ifndef OROCOS_SIMPLEESTIMATOR_COMPONENT_HPP
#define OROCOS_SIMPLEESTIMATOR_COMPONENT_HPP

#include "../EstimatorInterface/EstimatorInterface-component.hpp"

class SimpleEstimator : public EstimatorInterface{
  public:
    SimpleEstimator(std::string const& name);
    bool estimateUpdate();
    bool initialize();
};

#endif
