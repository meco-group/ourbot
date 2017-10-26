#include "Communicator-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/algorithm/string.hpp>

using namespace std;

Communicator::Communicator(std::string const& name) : TaskContext(name, PreOperational){
  addOperation("addOutgoingConnection", &Communicator::addOutgoingConnection, this).doc("Add outgoing connection");
  addOperation("addIncomingConnection", &Communicator::addIncomingConnection, this).doc("Add incoming connection");
  addOperation("addBufferedIncomingConnection", &Communicator::addBufferedIncomingConnection, this).doc("Add buffered incoming connection");
  addOperation("removeConnection", &Communicator::removeConnection, this).doc("Remove connection.");
  addOperation("setConnectionGroup", &Communicator::setConnectionGroup, this).doc("Change group of outgoing connection.");
  addOperation("setConnectionRate", &Communicator::setConnectionRate, this).doc("Change rate of outgoing connection.");
  addOperation("joinGroup", &Communicator::joinGroup, this).doc("Join a communication group.");
  addOperation("leaveGroup", &Communicator::leaveGroup, this).doc("Leave a communication group.");
  addOperation("getGroupSize", &Communicator::getGroupSize, this).doc("Get size of a communication group.");
  addOperation("getGroupPeers", &Communicator::getGroupPeers, this).doc("Get peers in group.");
  addOperation("getSender", &Communicator::getSender, this).doc("Get sender of last message on connection.");
  addOperation("getHost", &Communicator::getHost, this).doc("Get name of this host.");
  addOperation("setHost", &Communicator::setHost, this).doc("Set name of this host.");
  addOperation("getUUID", &Communicator::getUUID, this).doc("Get uuid of zyre node");
  addOperation("getPeerUUID", &Communicator::getPeerUUID, this).doc("Translate peer name to its uuid");
  addOperation("disable", &Communicator::disable, this).doc("Disable connection.");
  addOperation("enable", &Communicator::enable, this).doc("Enable connection.");
  addOperation("wait", &Communicator::wait, this).doc("Wait a (milli)sec.");
  addOperation("writeMail", &Communicator::writeMail, this).doc("Push string message in outbox.");
  addOperation("readMail", &Communicator::readMail, this).doc("Read latest received string message in inbox.");
  addOperation("removeMail", &Communicator::removeMail, this).doc("Pop latest received string message from inbox.");

  addProperty("iface", _iface).doc("Interface for communication.");
  addProperty("portnr", _portnr).doc("Port number to send discovery beacons on.");
  addProperty("host", _host).doc("Name of this host.");
  addProperty("verbose", _verbose).doc("Verbosity of debug info.");
}

bool Communicator::configureHook(){
  if (!createNode()){
    log(Error) << "Error while creating zyre node." << endlog();
    return false;
  }
  if (!joinGroups()){
    log(Error) << "Error while joining groups." << endlog();
    return false;
  }
  _mailbox = new Mailbox(_node);
  _mailbox->setVerbose(_verbose);
  return true;
}

void Communicator::updateHook(){
  TimeService::ticks timestamp = TimeService::Instance()->getTicks();
  for (uint k=0; k<_connections.size(); k++){
    if (!_connections[k]->speak()){
      log(Error) << "Error in speaking of " << _connections[k]->toString() << endlog();
      error();
    }
  }
  if (!_mailbox->send()){
    log(Error) << "Error in sending mailbox" << endlog();
  }
  // listing to incoming messages
  if (!listen()){
    error();
  }
  if (_verbose >= 4){
    Seconds time_elapsed = TimeService::Instance()->secondsSince(timestamp);
    std::cout << "Communication update took " << time_elapsed << " s" << std::endl;
  }
}

void Communicator::cleanupHook(){
  std::vector<std::string> groups;
  getMyGroups(groups);
  for (uint k=0; k<groups.size(); k++){
    leaveGroup(groups[k]);
  }
  zyre_stop(_node);
  zyre_destroy(&_node);
}

bool Communicator::isInput(Port port){
  if (dynamic_cast<RTT::base::InputPortInterface*>(port) == NULL){
    return false;
  }
  return true;
}

bool Communicator::createNode(){
  _node = zyre_new(_host.c_str());
  if (_verbose >= 3){
    zyre_set_verbose(_node);
  }
  zyre_set_port(_node, _portnr);
  zyre_set_interface(_node, _iface.c_str());
  if (zyre_start(_node) != 0){
      return false;
  }
  std::cout << "node " << _host.c_str() << " created." << std::endl;
  _poller = zpoller_new(zyre_socket(_node));
  zclock_sleep(100);
  return true;
}

bool Communicator::joinGroups(){
  if (!joinGroup(_host)){
    return false;
  }
  if (!joinGroup("all")){
    return false;
  }
  if (!streq(_host.c_str(), "emperor")){
    if (!joinGroup("ourbots")){
      return false;
    }
  }
  return true;
}

bool Communicator::joinGroup(const std::string& group){
  if (zyre_join(_node, group.c_str())){
    return false;
  }
  addGroup(group, _host);
  return true;
}

bool Communicator::leaveGroup(const std::string& group){
  if (zyre_leave(_node, group.c_str())){
    return false;
  }
  removeGroup(group, _host);
  return true;
}

void Communicator::addPeer(const std::string& peer, const std::string& peer_uuid) {
  _peers[peer] = peer_uuid;
}

void Communicator::removePeer(const std::string& peer) {
  if (_peers.find(peer) == _peers.end()) {
    return;
  }
  _peers.erase(peer);
}

void Communicator::addGroup(const std::string& group, const std::string& peer){
  if (_groups.find(group) == _groups.end()){
    _groups[group] = {peer};
  } else {
    if (std::find(_groups[group].begin(), _groups[group].end(), peer) == _groups[group].end()){
      _groups[group].push_back(peer);
    }
  }
}

void Communicator::removeGroup(const std::string& group, const std::string& peer){
  if (_groups.find(group) == _groups.end()){
    return;
  }
  _groups[group].erase(std::remove(_groups[group].begin(), _groups[group].end(), peer), _groups[group].end());
  if (_groups[group].empty()){
    _groups.erase(group);
  }
}

void Communicator::getMyGroups(std::vector<std::string>& groups){
  groups.resize(0);
  std::string group;
  std::vector<std::string> peers;
  for (std::map<std::string,std::vector<std::string>>::iterator iter=_groups.begin(); iter!=_groups.end(); ++iter){
    group = iter->first;
    peers = iter->second;
    if (std::find(peers.begin(), peers.end(), _host) != peers.end()){
      groups.push_back(group);
    }
  }
}

std::string Communicator::getPeerUUID(const std::string& peer){
  return _peers[peer];
}

int Communicator::getGroupSize(const std::string& group){
  if (_groups.find(group) == _groups.end()){
    return 0;
  }
  else {
    return _groups[group].size();
  }
}

std::vector<std::string> Communicator::getGroupPeers(const std::string& group) {
  if (_groups.find(group) == _groups.end()){
    return std::vector<std::string>({});
  } else {
    return _groups[group];
  }
}

bool Communicator::listen(){
  void* which;
  while(true){
    which = zpoller_wait(_poller, 0);
    if (which != zyre_socket(_node)){
      break;
    }
    if (which == NULL){ // pipe is empty
      return true;
    }
    _event = zyre_event_new(_node);
    const char* command = zyre_event_type(_event);
    if (streq(command, "ENTER")){
      std::string peer = std::string(zyre_event_peer_name(_event));
      std::string peer_uuid = std::string(zyre_event_peer_uuid(_event));
      addPeer(peer, peer_uuid);
      if (_verbose >= 0) {
        std::cout << peer << " entered the network." << std::endl;
      }
    }
    if (streq(command, "EXIT")){
      std::string peer = std::string(zyre_event_peer_name(_event));
      std::string peer_uuid = std::string(zyre_event_peer_uuid(_event));
      removePeer(peer);
      if (_verbose >= 0) {
        std::cout << peer << " left the network." << std::endl;
      }
    }
    if (streq(command, "JOIN")){
      std::string group = std::string(zyre_event_group(_event));
      std::string peer = std::string(zyre_event_peer_name(_event));
      addGroup(group, peer);
      if (_verbose >= 0) {
        std::cout << peer << " joined " << group << "." << std::endl;
      }
    }
    if (streq(command, "LEAVE")){
      std::string group = std::string(zyre_event_group(_event));
      std::string peer = std::string(zyre_event_peer_name(_event));
      removeGroup(group, peer);
      if (_verbose >= 0) {
        std::cout << peer << " left " << group << "." << std::endl;
      }
    }
    if (streq(command, "SHOUT") || streq(command, "WHISPER")){
      zmsg_t* msg = zyre_event_msg(_event);
      if (msg == NULL){
          log(Error) << "Received message is NULL." << endlog();
          return false;
      }
      std::string peer = std::string(zyre_event_peer_name(_event));
      std::string peer_uuid = std::string(zyre_event_peer_uuid(_event));

      if (zmsg_size(msg) == 1) {
        zframe_t* data_frame = zmsg_first(msg);
        if (!_mailbox->receive(data_frame, peer, peer_uuid)) {
          log(Error) << "Error while receiving mail." << endlog();
          return false;
        }
      } else if (zmsg_size(msg) == 2) {
        zframe_t* header_frame = zmsg_first(msg);
        zframe_t* data_frame = zmsg_next(msg);
        char* header = (char*)zframe_data(header_frame);
        for (uint k=0; k<_connections.size(); k++) {
          if(!_connections[k]->receive(header, data_frame, peer)) {
            log(Error) << "Error while receiving in " << _connections[k]->toString() << endlog();
            return false;
          }
        }
      } else {
        log(Warning) << "Wrong message size, discarding." << endlog();
      }
      zmsg_destroy(&msg);
    }
  }
  return true;
}

void Communicator::writeMail(const string& message, const string& peer) {
  _mailbox->write(message, peer);
}

std::vector<string> Communicator::readMail(bool remove) {
  string message, peer;
  if (!_mailbox->read(message, peer)) {
    return std::vector<string>{};
  }
  if (remove) {
    removeMail();
  }
  return std::vector<string>{message, peer};
}

void Communicator::removeMail() {
  _mailbox->remove();
}

std::string Communicator::getSender(const string& component_name, const string& port_name, const string& id){
  Connection* connection = _con_map[component_name+port_name+id];
  return connection->getSender();
}

std::string Communicator::getHost(){
  return _host;
}

void Communicator::setHost(const string& host){
  _host = host;
}

std::string Communicator::getUUID(){
  std::string buf = std::string(zyre_uuid(_node));
  std::string uuid = buf.substr(0, 8) + '-' + buf.substr(8, 4) + '-' + buf.substr(12, 4) + '-' + buf.substr(16, 4) + '-' + buf.substr(20, 12);
  boost::to_lower(uuid);
  return uuid;
}

Port Communicator::clonePort(const string& component_name, const string& port_name, ConnPolicy& policy){
  TaskContext* component = getPeer(component_name);
  if (component == NULL){
    log(Error) << "Could not find component " << component_name << "." << endlog();
    return false;
  }
  Port port = component->getPort(port_name);
  if (port == NULL){
    log(Error) << "Could not find port " << port_name << " of " << component_name << "." << endlog();
    return false;
  }
  Port anti_port = this->getPort(port_name);
  if (anti_port == NULL){
    anti_port = port->antiClone();
    addPort(port_name, *anti_port);
  }
  port->connectTo(anti_port, policy);
  return anti_port;
}

bool Communicator::addOutgoingConnection(const string& component_name, const string& port_name, const string& id, const string& group){
  Connection* connection;
  ConnPolicy policy = RTT::ConnPolicy::data();
  Port anti_port = clonePort(component_name, port_name, policy);
  if (_con_map.find(component_name+port_name+id) != _con_map.end()){
    connection = _con_map[component_name+port_name+id];
    connection->addGroup(group);
  } else {
    connection = getOutgoingConnection(anti_port, _node, id, group);
  }
  if (connection == NULL){
    log(Error) << "Type of port is not known!" << endlog();
    return false;
  }
  connection->setVerbose(_verbose);
  _connections.push_back(connection);
  _con_map[component_name+port_name+id] = connection;
  return true;
}

bool Communicator::addIncomingConnection(const string& component_name, const string& port_name, const string& id){
  Connection* connection;
  ConnPolicy policy = RTT::ConnPolicy::data();
  Port anti_port = clonePort(component_name, port_name, policy);
  if (_con_map.find(component_name+port_name+id) != _con_map.end()){
    connection = _con_map[component_name+port_name+id];
  } else {
    connection = getIncomingConnection(anti_port, _node, id);
  }
  if (connection == NULL){
    log(Error) << "Type of port is not known!" << endlog();
    return false;
  }
  connection->setVerbose(_verbose);
  _connections.push_back(connection);
  _con_map[component_name+port_name+id] = connection;
  return true;
}

bool Communicator::addBufferedIncomingConnection(const string& component_name, const string& port_name, const string& id, int buffer_size){
  Connection* connection;
  ConnPolicy policy = RTT::ConnPolicy::buffer(buffer_size);
  Port anti_port = clonePort(component_name, port_name, policy);
  if (_con_map.find(component_name+port_name+id) != _con_map.end()){
    connection = _con_map[component_name+port_name+id];
  } else {
    connection = getIncomingConnection(anti_port, _node, id);
    connection->setBufferSize(buffer_size);
    connection->setVerbose(_verbose);
  }
  if (connection == NULL){
    log(Error) << "Type of port is not known!" << endlog();
    return false;
  }
  _connections.push_back(connection);
  _con_map[component_name+port_name+id] = connection;
  return true;
}

void Communicator::removeConnection(const string& component_name, const string& port_name, const string& id){
  Connection* connection = _con_map[component_name+port_name+id];
  _connections.erase(std::remove(_connections.begin(), _connections.end(), connection), _connections.end());
  _con_map.erase(component_name+port_name+id);
}

bool Communicator::setConnectionGroup(const string& component_name, const string& port_name, const string& id, const string& group) {
  if (_con_map.find(component_name+port_name+id) != _con_map.end()) {
    Connection* connection = _con_map[component_name+port_name+id];
    connection->setGroup(group);
    return true;
  } else {
    log(Error) << "Connection not known!" << endlog();
    return false;
  }
}

bool Communicator::setConnectionRate(const string& component_name, const string& port_name, const string& id, double rate, double master_rate) {
  if (rate > master_rate) {
    log(Error) << "Impossible to choose connection rate higher than " << master_rate << "Hz!" << std::endl;
    return false;
  }
  if (_con_map.find(component_name+port_name+id) != _con_map.end()) {
    Connection* connection = _con_map[component_name+port_name+id];
    connection->setRate(rate, master_rate);
    return true;
  } else {
    log(Error) << "Connection not known!" << endlog();
    return false;
  }
}

void Communicator::disable(const string& component_name, const string& port_name, const string& id){
  Connection* connection = _con_map[component_name+port_name+id];
  connection->disable();
}

void Communicator::enable(const string& component_name, const string& port_name, const string& id){
  Connection* connection = _con_map[component_name+port_name+id];
  connection->enable();
}

void Communicator::wait(int ms){
  zclock_sleep(ms);
}

Connection* Communicator::getIncomingConnection(Port port, zyre_t* node, const string& id){
  string type = port->getTypeInfo()->getTypeName();
  if (type.compare("bool") == 0)
    return new IncomingConnection <bool> (port, node, id);
  if (type.compare("char") == 0)
    return new IncomingConnection <char> (port, node, id);
  if (type.compare("unsigned char") == 0)
    return new IncomingConnection <unsigned char> (port, node, id);
  if (type.compare("signed char") == 0)
    return new IncomingConnection <signed char> (port, node, id);
  if (type.compare("int") == 0)
    return new IncomingConnection <int> (port, node, id);
  if (type.compare("unsigned int") == 0)
    return new IncomingConnection <unsigned int> (port, node, id);
  if (type.compare("signed int") == 0)
    return new IncomingConnection <signed int> (port, node, id);
  if (type.compare("short int") == 0)
    return new IncomingConnection <short int> (port, node, id);
  if (type.compare("unsigned short int") == 0)
    return new IncomingConnection <unsigned short int> (port, node, id);
  if (type.compare("signed short int") == 0)
    return new IncomingConnection <signed short int> (port, node, id);
  if (type.compare("long int") == 0)
    return new IncomingConnection <long int> (port, node, id);
  if (type.compare("signed long int") == 0)
    return new IncomingConnection <signed long int> (port, node, id);
  if (type.compare("unsigned long int") == 0)
    return new IncomingConnection <unsigned long int> (port, node, id);
  if (type.compare("float") == 0)
    return new IncomingConnection <float> (port, node, id);
  if (type.compare("double") == 0)
    return new IncomingConnection <double> (port, node, id);
  if (type.compare("long double") == 0)
    return new IncomingConnection <long double> (port, node, id);
  if (type.compare("wchar_t") == 0)
    return new IncomingConnection <wchar_t> (port, node, id);
  if (type.compare("string") == 0)
    return new IncomingConnection <string, char> (port, node, id);
  if (type.compare("array") == 0)
    return new IncomingConnection <vector<double>, double > (port, node, id);
  return NULL;
}

Connection* Communicator::getOutgoingConnection(Port port, zyre_t* node, const string& id, const string& group){
  string type = port->getTypeInfo()->getTypeName();
  if (type.compare("bool") == 0)
    return new OutgoingConnection <bool> (port, node, id, group);
  if (type.compare("char") == 0)
    return new OutgoingConnection <char> (port, node, id, group);
  if (type.compare("unsigned char") == 0)
    return new OutgoingConnection <unsigned char> (port, node, id, group);
  if (type.compare("signed char") == 0)
    return new OutgoingConnection <signed char> (port, node, id, group);
  if (type.compare("int") == 0)
    return new OutgoingConnection <int> (port, node, id, group);
  if (type.compare("unsigned int") == 0)
    return new OutgoingConnection <unsigned int> (port, node, id, group);
  if (type.compare("signed int") == 0)
    return new OutgoingConnection <signed int> (port, node, id, group);
  if (type.compare("short int") == 0)
    return new OutgoingConnection <short int> (port, node, id, group);
  if (type.compare("unsigned short int") == 0)
    return new OutgoingConnection <unsigned short int> (port, node, id, group);
  if (type.compare("signed short int") == 0)
    return new OutgoingConnection <signed short int> (port, node, id, group);
  if (type.compare("long int") == 0)
    return new OutgoingConnection <long int> (port, node, id, group);
  if (type.compare("signed long int") == 0)
    return new OutgoingConnection <signed long int> (port, node, id, group);
  if (type.compare("unsigned long int") == 0)
    return new OutgoingConnection <unsigned long int> (port, node, id, group);
  if (type.compare("float") == 0)
    return new OutgoingConnection <float> (port, node, id, group);
  if (type.compare("double") == 0)
    return new OutgoingConnection <double> (port, node, id, group);
  if (type.compare("long double") == 0)
    return new OutgoingConnection <long double> (port, node, id, group);
  if (type.compare("wchar_t") == 0)
    return new OutgoingConnection <wchar_t> (port, node, id, group);
  if (type.compare("string") == 0)
    return new OutgoingConnection <string, char> (port, node, id, group);
  if (type.compare("array") == 0)
    return new OutgoingConnection <vector<double>, double > (port, node, id, group);
  return NULL;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Communicator)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Communicator)
