#ifndef OROCOS_EXAMPLEESTIMATOR_COMPONENT_HPP
#define OROCOS_EXAMPLEESTIMATOR_COMPONENT_HPP

#include "../EstimatorInterface/EstimatorInterface-component.hpp"

class ExampleEstimator : public EstimatorInterface{
  private:
    int _cnt;

  public:
    ExampleEstimator(std::string const& name);
    bool estimateUpdate();
    bool initialize();
};

#endif
