#ifndef OROCOS_COMMUNICATOR_COMPONENT_HPP
#define OROCOS_COMMUNICATOR_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include "Connection.hpp"

using namespace RTT;

typedef base::PortInterface* Port;

class Communicator : public RTT::TaskContext{
  private:
    std::vector<Connection *> _connections;
    Connection* getIncomingConnection(Port port, int port_nr);
    Connection* getOutgoingConnection(Port port, int port_nr, const std::string& remote_address);

  public:
    Communicator(std::string const& name);
    void updateHook();
    void cleanupHook();
    bool addOutgoing(const std::string& component_name, const std::string& port_name, int port_nr, const std::string& remote_address);
    bool addIncoming(const std::string& component_name, const std::string& port_name, int port_nr);
};

#endif
