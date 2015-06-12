#ifndef OROCOS_EXAMPLEESTIMATOR_COMPONENT_HPP
#define OROCOS_EXAMPLEESTIMATOR_COMPONENT_HPP

#include "../EstimatorInterface/EstimatorInterface-component.hpp"

class ExampleEstimator : public EstimatorInterface{

  public:
    ExampleEstimator(std::string const& name);
    bool estimateUpdate();
    bool initialize();
};

#endif
