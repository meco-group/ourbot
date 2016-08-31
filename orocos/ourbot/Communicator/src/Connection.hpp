#ifndef CONNECTION_HPP
#define CONNECTION_HPP

// #define DEBUG

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <stdlib.h> /* defines exit and other sys calls */
#include <stdio.h>
#include <string.h> // needed for memset
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#define BUFFERSIZE 1024

using namespace RTT;
typedef base::PortInterface* Port;

class Connection {
    protected:
        int _socket;
        int _port_nr;
        std::vector<std::string> _trusted_hosts;
        std::vector<sockaddr_in> _rem_addresses;
        socklen_t _rem_address_len;

    public:
        Connection(int port_nr){
            _rem_address_len = sizeof(sockaddr);
            _port_nr = port_nr;
        }

        bool createSocket(int& socket_nr){
            // create socket
            struct sockaddr_in myaddr;
            if ((_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
                return false;
            }
            if (fcntl(_socket, F_SETFL, fcntl(_socket, F_GETFL, 0) |O_NONBLOCK) < 0) {
                return false;
            }
            // bind it to all local addresses and pick a port number
            memset((char *)&myaddr, 0, sizeof(myaddr));
            myaddr.sin_family = AF_INET;
            myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
            myaddr.sin_port = htons(_port_nr);
            if (bind(_socket, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
                return false;
            }
            socket_nr = _socket;
            return true;
        }

        bool setSocket(int socket_nr){
            _socket = socket_nr;
            return true;
        }

        void setTrustedHosts(const std::vector<std::string>& trusted_hosts){
            _trusted_hosts = trusted_hosts;
        }

        bool checkHost(const sockaddr_in& host_address){
            for (uint k=0; k<_trusted_hosts.size(); k++){
                if (_trusted_hosts[k].compare(readAddress(host_address)) == 0){
                    return true;
                }
            }
            return false;
        }

        std::string readAddress(const sockaddr_in& address){
            char address_string[INET_ADDRSTRLEN];
            if (inet_ntop(AF_INET, &address.sin_addr, address_string, INET_ADDRSTRLEN)==0){
                return "not found";
            }
            std::string ret(address_string);
            return ret;
        }

        bool createRemoteAddresses(const std::vector<std::string>& remote_addresses){
            _rem_addresses.resize(remote_addresses.size());
            for (uint k=0; k<remote_addresses.size(); k++){
                memset((char *) &_rem_addresses[k], 0, sizeof(_rem_addresses[k]));
                _rem_addresses[k].sin_family = AF_INET;
                _rem_addresses[k].sin_port = htons(_port_nr);
                if (inet_pton(AF_INET, remote_addresses[k].c_str(), &_rem_addresses[k].sin_addr)==0) {
                  return false;
                }
            }
            return true;
        }

        int getSocket(){
            return _socket;
        }

        void closeConnection(){
            close(_socket);
        }

        virtual bool checkPort()=0;
        virtual std::string toString()=0;
};

template <class C, typename T=void> class OutgoingConnection : public Connection {
    private:
        InputPort<C>* _port;
        C _data;
        size_t _size;

    public:
        OutgoingConnection(Port& port, int port_nr, const std::vector<std::string>& remote_addresses):Connection(port_nr){
            createRemoteAddresses(remote_addresses);
            _port = (InputPort<C>*)port;
            _size = sizeof(T);
        }

        bool checkPort(){
            if (_port->read(_data) == RTT::NewData){
                #ifdef DEBUG
                std::cout << "sending "  << _data.size()*_size << " bytes to ";
                std::cout << readAddress(_rem_addresses[0]);
                for (uint k=1; k<_rem_addresses.size(); k++){
                    std::cout << ", " << readAddress(_rem_addresses[k]);
                }
                std::cout << std::endl;
                std::cout << "[" << _data[0];
                for (uint k=1; k<_data.size(); k++){
                    std::cout << "," << _data[k];
                }
                std::cout << "]" << std::endl;
                #endif
                for (uint k=0; k<_rem_addresses.size(); k++){
                    if (sendto(_socket, (void*)&_data[0], _data.size()*_size, 0, (struct sockaddr *)&_rem_addresses[k], _rem_address_len)==-1){
                        return false;
                    }
                }
            }
            return true;
        }

        std::string toString(){
            return "outgoing connection " + _port->getName();
        }
};

template <class C> class OutgoingConnection<C, void> : public Connection {
    private:
        InputPort<C>* _port;
        C _data;

    public:
        OutgoingConnection(Port& port, int port_nr, const std::vector<std::string>& remote_addresses):Connection(port_nr){
            createRemoteAddresses(remote_addresses);
            _port = (InputPort<C>*)port;
        }

        bool checkPort(){
            if (_port->read(_data) == RTT::NewData){
                #ifdef DEBUG
                std::cout << "sending "  << sizeof(_data) << " bytes to ";
                std::cout << readAddress(_rem_addresses[0]);
                for (uint k=1; k<_rem_addresses.size(); k++){
                    std::cout << ", " << readAddress(_rem_addresses[k]);
                }
                std::cout << std::endl;
                std::cout << _data << std::endl;
                #endif
                for (uint k=0; k<_rem_addresses.size(); k++){
                    if (sendto(_socket, (void*)&_data, sizeof(_data), 0, (struct sockaddr *)&_rem_addresses[k], _rem_address_len)==-1){
                        return false;
                    }
                }
            }
            return true;
        }

        std::string toString(){
            return "outgoing connection " + _port->getName();
        }
};

template <class C, typename T=void> class IncomingConnection : public Connection {
    private:
        OutputPort<C>* _port;
        int _rcv_len;
        size_t _size;
        unsigned char _buffer[BUFFERSIZE];
        C _data;
        sockaddr_in _rcv_address;

    public:
        IncomingConnection(Port& port, int port_nr):Connection(port_nr){
            _port = (OutputPort<C>*)port;
            _size = sizeof(T);
        }

        bool checkPort(){
            _rcv_len = recvfrom(_socket, _buffer, BUFFERSIZE, 0, (struct sockaddr*)&_rcv_address, &_rem_address_len);
            if (_rcv_len > 0 && checkHost(_rcv_address)) {
                _data.resize(_rcv_len/_size);
                memcpy(&_data[0], _buffer, _rcv_len);
                _port->write(_data);
                #ifdef DEBUG
                std::cout << "receiving "<< _rcv_len << " bytes from ";
                std::cout << readAddress(_rcv_address) << std::endl;
                 std::cout << "[" << _data[0];
                for (uint k=1; k<_data.size(); k++){
                    std::cout << "," << _data[k];
                }
                std::cout << "]" << std::endl;
                #endif
            }
            else if (errno != EAGAIN)
                return false;
            return true;
        }

        std::string toString(){
            return "incoming connection " + _port->getName();
        }
};

template <class C> class IncomingConnection<C, void> : public Connection {
    private:
        OutputPort<C>* _port;
        int _rcv_len;
        C _data;
        sockaddr_in _rcv_address;

    public:
        IncomingConnection(Port& port, int port_nr):Connection(port_nr){
            _port = (OutputPort<C>*)port;
        }

        bool checkPort(){
            _rcv_len = recvfrom(_socket, &_data, sizeof(_data), 0, (struct sockaddr *)&_rcv_address, &_rem_address_len);
            if (_rcv_len > 0 && checkHost(_rcv_address)) {
                _port->write(_data);
                #ifdef DEBUG
                std::cout << "receiving " << _rcv_len << " bytes from ";
                std::cout << readAddress(_rcv_address) << std::endl;
                std::cout << _data << std::endl;
                #endif
            }
            else if (errno != EAGAIN)
                return false;
            return true;
        }

        std::string toString(){
            return "incoming connection " + _port->getName();
        }
};

#endif
