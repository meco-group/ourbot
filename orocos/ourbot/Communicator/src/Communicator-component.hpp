#ifndef OROCOS_COMMUNICATOR_COMPONENT_HPP
#define OROCOS_COMMUNICATOR_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include "Connection.hpp"

using namespace RTT;

typedef base::PortInterface* Port;

class Communicator : public RTT::TaskContext{
  private:
    std::map<int, int> _sockets;
    std::vector<Connection*> _connections;
    std::vector<std::string> _trusted_hosts;

    Connection* getIncomingConnection(Port port, int port_nr);
    Connection* getOutgoingConnection(Port port, int port_nr, const std::vector<std::string>& remote_addresses);
    bool retrieveSocket(Connection* connection, int port_nr);
    Connection* findConnection(int port_nr);

  public:
    Communicator(std::string const& name);
    void updateHook();
    bool configureHook();
    void cleanupHook();
    bool addOutgoing(const std::string& component_name, const std::string& port_name, int port_nr, const std::vector<std::string>& remote_address);
    bool addIncoming(const std::string& component_name, const std::string& port_name, int port_nr);
};

#endif
