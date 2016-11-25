#include "Communicator-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

Communicator::Communicator(std::string const& name) : TaskContext(name, PreOperational){
  addOperation("addConnection", &Communicator::addConnection, this).doc("Add connection");
  addOperation("removeConnection", &Communicator::removeConnection, this).doc("Remove connection.");
  addOperation("disable", &Communicator::disable, this).doc("Disable connection.");
  addOperation("enable", &Communicator::enable, this).doc("Enable connection.");

  addProperty("iface", _iface).doc("Interface for communication.");
  addProperty("portnr", _portnr).doc("Port number to send discovery beacons on.");
  addProperty("host", _host).doc("Name of this host.");

  _poller = zpoller_new(NULL);
}

bool Communicator::configureHook(){
  return true;
}

void Communicator::updateHook(){
  // let each connection speak
  for (uint k=0; k<_connections.size(); k++){
    if (!_connections[k]->speak()){
      log(Error) << "Error in speaking of " << _connections[k]->toString() << endlog();
      error();
    }
  }
  // listing to incoming messages
  void* which;
  while(true){
    which = zpoller_wait(_poller, 0);
    if (which == NULL){ // pipe is empty
        break;
    }
    for (uint k=0; k<_connections.size(); k++){
      if (!_connections[k]->listen(which)){
        log(Error) << "Error in listening of " << _connections[k]->toString() << endlog();
        error();
      }
    }
  }
}

void Communicator::cleanupHook(){
  for (uint k=0; k<_connections.size(); k++){
    _connections[k]->close();
  }
}

bool Communicator::isInput(Port port){
  if (dynamic_cast<RTT::base::InputPortInterface*>(port) == NULL){
    return false;
  }
  return true;
}

bool Communicator::addConnection(const string& component_name, const string& port_name, const string& id){
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
  Port anti_port = port->antiClone();
  addPort(port_name, *anti_port);
  port->connectTo(anti_port);
  Connection* connection;
  if (isInput(anti_port)){
    connection = getOutgoingConnection(anti_port, _host, id);
  } else {
    connection = getIncomingConnection(anti_port, _host, id);
  }
  if (connection == NULL){
    log(Error) << "Type of port is not known!" << endlog();
    return false;
  }
  if (!connection->createNode(_iface, _portnr, _poller)){
    log(Error) << "Could not create zyre node!" << endlog();
    return false;
  }
  _connections.push_back(connection);
  _con_map[component_name+port_name+id] = connection;
  return true;
}

void Communicator::removeConnection(const string& component_name, const string& port_name, const string& id){
  Connection* connection = _con_map[component_name+port_name+id];
  connection->close();
  uint index=0;
  for (uint k=0; k<_connections.size(); k++){
    if (_connections[k] == connection){
      index = k;
      break;
    }
  }
  _connections.erase(_connections.begin()+index);
  _con_map.erase(component_name+port_name+id);
}

void Communicator::disable(const string& component_name, const string& port_name, const string& id){
  Connection* connection = _con_map[component_name+port_name+id];
  connection->disable();
}

void Communicator::enable(const string& component_name, const string& port_name, const string& id){
  Connection* connection = _con_map[component_name+port_name+id];
  connection->enable();
}

Connection* Communicator::getIncomingConnection(Port port, const string& host, const string& id){
  string type = port->getTypeInfo()->getTypeName();
  if (type.compare("char") == 0)
    return new IncomingConnection <char> (port, host, id);
  if (type.compare("unsigned char") == 0)
    return new IncomingConnection <unsigned char> (port, host, id);
  if (type.compare("signed char") == 0)
    return new IncomingConnection <signed char> (port, host, id);
  if (type.compare("int") == 0)
    return new IncomingConnection <int> (port, host, id);
  if (type.compare("unsigned int") == 0)
    return new IncomingConnection <unsigned int> (port, host, id);
  if (type.compare("signed int") == 0)
    return new IncomingConnection <signed int> (port, host, id);
  if (type.compare("short int") == 0)
    return new IncomingConnection <short int> (port, host, id);
  if (type.compare("unsigned short int") == 0)
    return new IncomingConnection <unsigned short int> (port, host, id);
  if (type.compare("signed short int") == 0)
    return new IncomingConnection <signed short int> (port, host, id);
  if (type.compare("long int") == 0)
    return new IncomingConnection <long int> (port, host, id);
  if (type.compare("signed long int") == 0)
    return new IncomingConnection <signed long int> (port, host, id);
  if (type.compare("unsigned long int") == 0)
    return new IncomingConnection <unsigned long int> (port, host, id);
  if (type.compare("float") == 0)
    return new IncomingConnection <float> (port, host, id);
  if (type.compare("double") == 0)
    return new IncomingConnection <double> (port, host, id);
  if (type.compare("long double") == 0)
    return new IncomingConnection <long double> (port, host, id);
  if (type.compare("wchar_t") == 0)
    return new IncomingConnection <wchar_t> (port, host, id);
  if (type.compare("string") == 0)
    return new IncomingConnection <string, char> (port, host, id);
  if (type.compare("array") == 0)
    return new IncomingConnection <vector<double>, double > (port, host, id);
  return NULL;
}

Connection* Communicator::getOutgoingConnection(Port port, const string& host, const string& id){
  string type = port->getTypeInfo()->getTypeName();
  if (type.compare("char") == 0)
    return new OutgoingConnection <char> (port, host, id);
  if (type.compare("unsigned char") == 0)
    return new OutgoingConnection <unsigned char> (port, host, id);
  if (type.compare("signed char") == 0)
    return new OutgoingConnection <signed char> (port, host, id);
  if (type.compare("int") == 0)
    return new OutgoingConnection <int> (port, host, id);
  if (type.compare("unsigned int") == 0)
    return new OutgoingConnection <unsigned int> (port, host, id);
  if (type.compare("signed int") == 0)
    return new OutgoingConnection <signed int> (port, host, id);
  if (type.compare("short int") == 0)
    return new OutgoingConnection <short int> (port, host, id);
  if (type.compare("unsigned short int") == 0)
    return new OutgoingConnection <unsigned short int> (port, host, id);
  if (type.compare("signed short int") == 0)
    return new OutgoingConnection <signed short int> (port, host, id);
  if (type.compare("long int") == 0)
    return new OutgoingConnection <long int> (port, host, id);
  if (type.compare("signed long int") == 0)
    return new OutgoingConnection <signed long int> (port, host, id);
  if (type.compare("unsigned long int") == 0)
    return new OutgoingConnection <unsigned long int> (port, host, id);
  if (type.compare("float") == 0)
    return new OutgoingConnection <float> (port, host, id);
  if (type.compare("double") == 0)
    return new OutgoingConnection <double> (port, host, id);
  if (type.compare("long double") == 0)
    return new OutgoingConnection <long double> (port, host, id);
  if (type.compare("wchar_t") == 0)
    return new OutgoingConnection <wchar_t> (port, host, id);
  if (type.compare("string") == 0)
    return new OutgoingConnection <string, char> (port, host, id);
  if (type.compare("array") == 0)
    return new OutgoingConnection <vector<double>, double > (port, host, id);
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
