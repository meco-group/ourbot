#ifndef OROCOS_COMMUNICATOR_COMPONENT_HPP
#define OROCOS_COMMUNICATOR_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include "Connection.hpp"
#include "Mailbox.hpp"
#include <zyre.h>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

using namespace RTT;
using namespace RTT::os;

typedef base::PortInterface* Port;
typedef base::PortInterface* Port;
typedef base::PortInterface* Port;

class Communicator : public RTT::TaskContext{
  private:
    std::string _iface;
    int _portnr;
    std::string _host;
    zyre_t* _node;
    zpoller_t* _poller;
    zyre_event_t* _event;
    int _verbose;
    Mailbox* _mailbox;

    std::vector<Connection*> _connections;
    std::map<std::string, Connection*> _con_map;
    std::map<std::string, std::string> _peers;
    std::map<std::string, std::vector<std::string>> _groups;

    bool isInput(Port port);
    bool createNode();
    bool joinGroups();
    void addPeer(const std::string& peer, const std::string& peer_uuid);
    void removePeer(const std::string& peer);
    void addGroup(const std::string& group, const std::string& peer);
    void removeGroup(const std::string& group, const std::string& peer);
    void getMyGroups(std::vector<std::string>& groups);
    bool listen();
    Port clonePort(const string& component_name, const string& port_name, ConnPolicy& policy);
    Connection* getIncomingConnection(Port port, zyre_t* node, const string& id);
    Connection* getOutgoingConnection(Port port, zyre_t* node, const string& id, const string& group);

  public:
    Communicator(std::string const& name);
    void updateHook();
    bool configureHook();
    void cleanupHook();
    bool addOutgoingConnection(const std::string& component_name, const std::string& port_name, const std::string& id, const std::string& group);
    bool addIncomingConnection(const std::string& component_name, const std::string& port_name, const std::string& id);
    bool addBufferedIncomingConnection(const string& component_name, const string& port_name, const string& id, int buffer_size);
    void writeMail(const string& message, const string& peer);
    std::vector<string> readMail(bool remove);
    void removeMail();
    void removeConnection(const std::string& component_name, const std::string& port_name, const std::string& id);
    bool setConnectionGroup(const string& component_name, const string& port_name, const string& id, const string& group);
    bool setConnectionRate(const string& component_name, const string& port_name, const string& id, double rate, double master_rate);
    void enable(const std::string& component_name, const std::string& port_name, const std::string& identifier);
    void disable(const std::string& component_name, const std::string& port_name, const std::string& identifier);
    void wait(int ms);
    bool joinGroup(const std::string& group);
    bool leaveGroup(const std::string& group);
    int getGroupSize(const std::string& group);
    std::string getPeerUUID(const std::string& peer);
    std::string getSender(const string& component_name, const string& port_name, const string& id);
    std::string getHost();
    void setHost(const string& host);
    std::string getUUID();
};

#endif
