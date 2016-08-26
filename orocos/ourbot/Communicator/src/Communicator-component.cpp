#include "Communicator-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

Communicator::Communicator(std::string const& name) : TaskContext(name, PreOperational){
  addOperation("addOutgoing", &Communicator::addOutgoing, this).doc("Adds an outgoing connection.");
  addOperation("addIncoming", &Communicator::addIncoming, this).doc("Adds an incoming connection.");
  addProperty("trusted_hosts", _trusted_hosts).doc("From which hosts should received packets be interpreted.");
}

bool Communicator::configureHook(){
  for (uint k=0; k<_connections.size(); k++){
    _connections[k]->setTrustedHosts(_trusted_hosts);
  }
  return true;
}

void Communicator::updateHook(){
  for (uint k=0; k<_connections.size(); k++){
    if (!_connections[k]->checkPort()){
      log(Error) << "Error in checking " << _connections[k]->toString() << endlog();
      error();
    }
  }
}

void Communicator::cleanupHook(){
  for (uint k=0; k<_connections.size(); k++){
    _connections[k]->closeConnection();
  }
}

bool Communicator::addOutgoing(const string& component_name, const string& port_name, int port_nr, const vector<string>& remote_addresses){
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
  Port anti_port;
  if (getPort(port_name) == NULL){
    anti_port = port->antiClone();
    addPort(port_name, *anti_port);
  }
  else {
    anti_port = getPort(port_name);
  }
  port->connectTo(anti_port);
  Connection* connection = getOutgoingConnection(anti_port, port_nr, remote_addresses);
  if (connection == NULL){
    log(Error) << "Type of port is not known!" << endlog();
    return false;
  }
  _connections.push_back(connection);
  connection->setTrustedHosts(_trusted_hosts);
  if (!retrieveSocket(connection, port_nr)){
    return false;
  }
  return true;
}

bool Communicator::addIncoming(const string& component_name, const string& port_name, int port_nr){
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
  Port anti_port;
  if (getPort(port_name) == NULL){
    anti_port = port->antiClone();
    addPort(port_name, *anti_port);
  }
  else {
    anti_port = getPort(port_name);
  }
  port->connectTo(anti_port);
  Connection* connection = getIncomingConnection(anti_port, port_nr);
  if (connection == NULL){
    log(Error) << "Type of port is not known!" << endlog();
    return false;
  }
  _connections.push_back(connection);
  connection->setTrustedHosts(_trusted_hosts);
  if(!retrieveSocket(connection, port_nr)){
    return false;
  }
  return true;
}

bool Communicator::retrieveSocket(Connection* connection, int port_nr){
  int socket_nr;
  if(_sockets.find(port_nr) == _sockets.end()){
    if(!connection->createSocket(socket_nr)){
      log(Error) << "Could not create socket!" << endlog();
      return false;
    }
  }
  else{
    socket_nr = _sockets[port_nr];
    if(!connection->setSocket(socket_nr)){
      log(Error) << "Could not set socket " << socket_nr << endlog();
      return false;
    }
  }
  _sockets[port_nr] = socket_nr;
  return true;
}

Connection* Communicator::getIncomingConnection(Port port, int port_nr){
  string type = port->getTypeInfo()->getTypeName();
  if (type.compare("char") == 0)
    return new IncomingConnection <char> (port, port_nr);
  if (type.compare("unsigned char") == 0)
    return new IncomingConnection <unsigned char> (port, port_nr);
  if (type.compare("signed char") == 0)
    return new IncomingConnection <signed char> (port, port_nr);
  if (type.compare("int") == 0)
    return new IncomingConnection <int> (port, port_nr);
  if (type.compare("unsigned int") == 0)
    return new IncomingConnection <unsigned int> (port, port_nr);
  if (type.compare("signed int") == 0)
    return new IncomingConnection <signed int> (port, port_nr);
  if (type.compare("short int") == 0)
    return new IncomingConnection <short int> (port, port_nr);
  if (type.compare("unsigned short int") == 0)
    return new IncomingConnection <unsigned short int> (port, port_nr);
  if (type.compare("signed short int") == 0)
    return new IncomingConnection <signed short int> (port, port_nr);
  if (type.compare("long int") == 0)
    return new IncomingConnection <long int> (port, port_nr);
  if (type.compare("signed long int") == 0)
    return new IncomingConnection <signed long int> (port, port_nr);
  if (type.compare("unsigned long int") == 0)
    return new IncomingConnection <unsigned long int> (port, port_nr);
  if (type.compare("float") == 0)
    return new IncomingConnection <float> (port, port_nr);
  if (type.compare("double") == 0)
    return new IncomingConnection <double> (port, port_nr);
  if (type.compare("long double") == 0)
    return new IncomingConnection <long double> (port, port_nr);
  if (type.compare("wchar_t") == 0)
    return new IncomingConnection <wchar_t> (port, port_nr);
  if (type.compare("string") == 0)
    return new IncomingConnection <string, char> (port, port_nr);
  if (type.compare("array") == 0)
    return new IncomingConnection <vector<double>, double > (port, port_nr);
  return NULL;
}

Connection* Communicator::getOutgoingConnection(Port port, int port_nr, const vector<string>& remote_addresses){
  string type = port->getTypeInfo()->getTypeName();
  if (type.compare("char") == 0)
    return new OutgoingConnection <char> (port, port_nr, remote_addresses);
  if (type.compare("unsigned char") == 0)
    return new OutgoingConnection <unsigned char> (port, port_nr, remote_addresses);
  if (type.compare("signed char") == 0)
    return new OutgoingConnection <signed char> (port, port_nr, remote_addresses);
  if (type.compare("int") == 0)
    return new OutgoingConnection <int> (port, port_nr, remote_addresses);
  if (type.compare("unsigned int") == 0)
    return new OutgoingConnection <unsigned int> (port, port_nr, remote_addresses);
  if (type.compare("signed int") == 0)
    return new OutgoingConnection <signed int> (port, port_nr, remote_addresses);
  if (type.compare("short int") == 0)
    return new OutgoingConnection <short int> (port, port_nr, remote_addresses);
  if (type.compare("unsigned short int") == 0)
    return new OutgoingConnection <unsigned short int> (port, port_nr, remote_addresses);
  if (type.compare("signed short int") == 0)
    return new OutgoingConnection <signed short int> (port, port_nr, remote_addresses);
  if (type.compare("long int") == 0)
    return new OutgoingConnection <long int> (port, port_nr, remote_addresses);
  if (type.compare("signed long int") == 0)
    return new OutgoingConnection <signed long int> (port, port_nr, remote_addresses);
  if (type.compare("unsigned long int") == 0)
    return new OutgoingConnection <unsigned long int> (port, port_nr, remote_addresses);
  if (type.compare("float") == 0)
    return new OutgoingConnection <float> (port, port_nr, remote_addresses);
  if (type.compare("double") == 0)
    return new OutgoingConnection <double> (port, port_nr, remote_addresses);
  if (type.compare("long double") == 0)
    return new OutgoingConnection <long double> (port, port_nr, remote_addresses);
  if (type.compare("wchar_t") == 0)
    return new OutgoingConnection <wchar_t> (port, port_nr, remote_addresses);
  if (type.compare("string") == 0)
    return new OutgoingConnection <string, char> (port, port_nr, remote_addresses);
  if (type.compare("array") == 0)
    return new OutgoingConnection <vector<double>, double > (port, port_nr, remote_addresses);
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
