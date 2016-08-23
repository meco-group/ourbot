#ifndef CONNECTION_HPP
#define CONNECTION_HPP

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

using namespace RTT;
typedef base::PortInterface* Port;

class Connection {
    protected:
        int _socket;
        int _port_nr;
        sockaddr_in _rem_address;
        socklen_t _rem_address_len;

    public:
        Connection(int port_nr){
            _rem_address_len = sizeof(_rem_address);
            if (!createSocket(port_nr)){
                log(Error) << "Could not create socket!" << endlog();
            }
        }

        bool createSocket(int port_nr){
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
            myaddr.sin_port = htons(port_nr);
            if (bind(_socket, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
                return false;
            }
            _port_nr = port_nr;
            return true;
        }

        void closeConnection(){
            close(_socket);
        }

        virtual bool checkPort()=0;
        virtual std::string toString()=0;
};

template <class T> class OutgoingConnection : public Connection {
    private:
        InputPort<T>* _port;
        T _data;

    public:
        OutgoingConnection(Port& port, int port_nr, const std::string& remote_address):Connection(port_nr){
            createRemoteAddress(remote_address);
            _port = (InputPort<T>*)port;
        }

        bool createRemoteAddress(const std::string remote_address){
            memset((char *) &_rem_address, 0, sizeof(_rem_address));
            _rem_address.sin_family = AF_INET;
            _rem_address.sin_port = htons(_port_nr);
            if (inet_aton(remote_address.c_str(), &_rem_address.sin_addr)==0) {
              return false;
            }
            return true;
        }

        bool checkPort(){
            if (_port->read(_data) == RTT::NewData){
                if(sendto(_socket, (void*)&_data, sizeof(_data), 0, (struct sockaddr *)&_rem_address, _rem_address_len)==-1){
                    return false;
                }
            }
            return true;
        }

        std::string toString(){
            return "outgoing connection " + _port->getName();
        }
};

template <class T> class IncomingConnection : public Connection {
    private:
        OutputPort<T>* _port;
        int _rcv_len;
        T _data;

    public:
        IncomingConnection(Port& port, int port_nr):Connection(port_nr){
            _port = (OutputPort<T>*)port;
        }

        bool checkPort(){
            _rcv_len = recvfrom(_socket, &_data, sizeof(_data), 0, (struct sockaddr *)&_rem_address, &_rem_address_len);
            if (_rcv_len > 0) {
                _port->write(_data);
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
