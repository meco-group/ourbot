#include "ExampleVelocityCommand-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

ExampleVelocityCommand::ExampleVelocityCommand(std::string const& name) : VelocityCommandInterface(name), _data_send(30), _data_receive(30){
    _cnt = 0.;
    ports()->addPort("test_port_out",_test_port_out);
    ports()->addPort("test_port_in",_test_port_in);
    ports()->addPort("test_port_vec_out",_test_port_vec_out);
    ports()->addPort("test_port_vec_in",_test_port_vec_in);
}

void ExampleVelocityCommand::updateHook(){
    double data;
    if(_test_port_in.read(data) == RTT::NewData){
        std::cout << "reading " << data << std::endl;
    }
    if(_test_port_vec_in.read(_data_send) == RTT::NewData){
        std::cout << "reading (" << std::endl;
        for(int k=0; k<30; k++){
            std::cout << _data_send[k] << ",";
        }
        std::cout << std::endl;
    }

    setVelocity(_cnt, _cnt, _cnt);
    std::cout << "writing " << _cnt << std::endl;
    _test_port_out.write(_cnt);
    for(int k=0; k<30; k++){
        _data_send[k] = 0.01*_cnt+k;
    }
    _test_port_vec_out.write(_data_send);
    _cnt += 1.;
    VelocityCommandInterface::updateHook();
}

ORO_LIST_COMPONENT_TYPE(ExampleVelocityCommand);
