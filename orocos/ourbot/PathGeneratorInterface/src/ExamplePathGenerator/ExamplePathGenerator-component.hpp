#ifndef OROCOS_EXAMPLEPATHGENERATOR_COMPONENT_HPP
#define OROCOS_EXAMPLEPATHGENERATOR_COMPONENT_HPP

#include "../PathGeneratorInterface/PathGeneratorInterface-component.hpp"

class ExamplePathGenerator : public PathGeneratorInterface{
  private:
    int _path_length;
    bool _even;

  public:
    ExamplePathGenerator(std::string const& name);
    bool pathUpdate();
    bool initialize();
};

#endif
