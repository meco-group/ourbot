#ifndef OROCOS_COMMUNICATOR_COMPONENT_HPP
#define OROCOS_COMMUNICATOR_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include "Connection.hpp"
#include <zyre.h>

using namespace RTT;

typedef base::PortInterface* Port;
typedef base::PortInterface* Port;
typedef base::PortInterface* Port;

class Communicator : public RTT::TaskContext{
  private:
    std::string _iface;
    int _portnr;
    std::string _host;

    std::vector<Connection*> _connections;
    zpoller_t* _poller;
    std::map<std::string, Connection*> _con_map;

    bool isInput(Port port);
    Connection* getIncomingConnection(Port port, const string& host, const string& id);
    Connection* getOutgoingConnection(Port port, const string& host, const string& id);

  public:
    Communicator(std::string const& name);
    void updateHook();
    bool configureHook();
    void cleanupHook();
    bool addConnection(const std::string& component_name, const std::string& port_name, const std::string& identifier);
    void removeConnection(const std::string& component_name, const std::string& port_name, const std::string& identifier);
    void enable(const std::string& component_name, const std::string& port_name, const std::string& identifier);
    void disable(const std::string& component_name, const std::string& port_name, const std::string& identifier);
};

#endif
