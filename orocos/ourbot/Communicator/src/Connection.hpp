#ifndef CONNECTION_HPP
#define CONNECTION_HPP

#define DEBUG

#include <zyre.h>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <string.h>

#define BUFFERSIZE 1024

using namespace RTT;
using namespace std;

typedef base::PortInterface* Port;

class Connection {
    protected:
        zyre_t* _node;
        zyre_event_t* _event;
        const char* _host;
        const char* _id;
        zlist_t* _peers;

    public:
        Connection(const string& host, const string& id) : _host(host.c_str()), _id(id.c_str()){
        }

        bool createNode(const string& iface, int portnr, zpoller_t* poller){
            _node = zyre_new(_host);
            zyre_set_verbose(_node);
            zyre_set_port(_node, portnr);
            zyre_set_interface(_node, iface.c_str());
            if (zyre_start(_node) != 0){
                return false;
            }
            // zclock_sleep(250);
            if (zyre_join(_node, _id) != 0){
                return false;
            }
            zpoller_add(poller, zyre_socket(_node));
            return true;
        }

        void close(){
            zlist_destroy(&_peers);
            zyre_leave(_node, _id);
            zyre_stop(_node);
            zyre_destroy(&_node);
        }

        bool disable(){ //??
            if (zyre_leave(_node, _id) != 0){
                return false;
            }
            zyre_stop(_node);
            return true;
        }

        bool enable(){ //??
            if (zyre_start(_node) != 0){
                return false;
            }
            if (zyre_join(_node, _id) != 0){
                return false;
            }
            return true;
        }

        bool listen(void* which){
            if (which != zyre_socket(_node)){
                return true;
            }
            _event = zyre_event_new(_node);
            const char* command = zyre_event_type(_event);
            #ifdef DEBUG
            if (streq(command, "JOIN") && streq(zyre_event_group(_event), _id)){
                std::cout << _host << ":" << _id << " connected to " << zyre_event_peer_name(_event) << ":" << _id << std::endl;
            }
            #endif
            if (streq(command, "SHOUT") && streq(zyre_event_group(_event), _id)){
                if(!receive(_event)){
                    return false;
                }
            }
            return true;
        }

        virtual bool speak(){
            return true;
        }

        virtual bool receive(zyre_event_t* event){
            return true;
        }

        std::string toString(){
            char str[80];
            strcpy(str, _host);
            strcat(str, ":");
            strcat(str, _id);
            return str;
        }

        // void check_entering_node(zyre_event_t* event){
        //     char* id = zyre_event_header(event, "id");
        //     if (streq(id, _id)){
        //         _peers.push_back(zyre_event_peer_uuid(event));
        //         #ifdef DEBUG
        //         std::cout << _host << ":" << _id << " added peer " << zyre_event_peer_name(event) << ":" << id << std::endl;
        //         #endif
        //     }
        // }
};


template <class C, typename T=void> class OutgoingConnection : public Connection {
    private:
        InputPort<C>* _port;
        C _data;
        unsigned char _data_str[BUFFERSIZE];
        size_t _type_size;
        size_t _size;

    public:
        OutgoingConnection(Port& port, const string& host, const string& id):Connection(host, id){
            _port = (InputPort<C>*)port;
            _type_size = sizeof(T);
        }

        bool speak(){
            if (_port->read(_data) == RTT::NewData){
                _size = _data.size()*_type_size;
                zmsg_t* msg = zmsg_new();
                zmsg_pushmem(msg, &_data[0], _size);
                if (zyre_shout(_node, _id, &msg) != 0){
                    return false;
                }
                zmsg_destroy(&msg);

                // _peers = zyre_peers_by_group(_node, _id);
                // if (_peers != NULL){
                //     std::cout << "peers size: " << zlist_size(_peers) << std::endl;
                // }
                // if (_peers != NULL && zlist_size(_peers) > 0){
                //     _size = _data.size()*_type_size;
                //     zmsg_t* msg = zmsg_new();
                //     zmsg_pushmem(msg, &_data[0], _size);
                //     if (zyre_shout(_node, _id, &msg) != 0){
                //         return false;
                //     }
                //     zmsg_destroy(&msg);
                //     #ifdef DEBUG
                //     std::cout << _host << ":" << _id << " (" << _port->getName() << ") sending " << _size << " bytes" << std::endl;
                //     #endif
                // }
            }
            return true;
        }
};

template <class C> class OutgoingConnection<C, void> : public Connection {
    private:
        InputPort<C>* _port;
        C _data;
        char _data_str[BUFFERSIZE];
        size_t _size;

    public:
        OutgoingConnection(Port& port, const string& host, const string& id):Connection(host, id){
            _port = (InputPort<C>*)port;
            _size = sizeof(C);
        }

        bool speak(){
            if (_port->read(_data) == RTT::NewData){
                zmsg_t* msg = zmsg_new();
                zmsg_pushmem(msg, &_data, _size);
                if (zyre_shout(_node, _id, &msg) != 0){
                    return false;
                }
                zmsg_destroy(&msg);

                // _peers = zyre_peers_by_group(_node, _id);
                // if (_peers != NULL){
                //     std::cout << "peers size: " << zlist_size(_peers) << std::endl;
                // }
                // if (_peers != NULL && zlist_size(_peers) > 0){
                //     zmsg_t* msg = zmsg_new();
                //     zmsg_pushmem(msg, &_data, _size);
                //     if (zyre_shout(_node, _id, &msg) != 0){
                //         return false;
                //     }
                //     zmsg_destroy(&msg);
                //     #ifdef DEBUG
                //     std::cout << _host << ":" << _id << " (" << _port->getName() << ") sending " << _size << " bytes" << std::endl;
                //     #endif
                // }
            }
            return true;
        }
};

template <class C, typename T=void> class IncomingConnection : public Connection {
    private:
        OutputPort<C>* _port;
        C _data;
        size_t _type_size;
        size_t _size;

    public:
        IncomingConnection(Port& port, const string& host, const string& id):Connection(host, id){
            _port = (OutputPort<C>*)port;
            _type_size = sizeof(T);
        }

        bool receive(zyre_event_t* event){
            zmsg_t* msg = zyre_event_msg(event);
            if (msg == NULL){
                return false;
            }
            _size = zmsg_content_size(msg);
            _data.resize(_size/_type_size);
            memcpy(&_data[0], zmsg_popstr(msg), _size);
            zmsg_destroy(&msg);
            _port->write(_data);
            #ifdef DEBUG
            std::cout << _host << ":" << _id << " (" << _port->getName() << ") received " << _size << " bytes from " << zyre_event_peer_name(event) << std::endl;
            #endif
            return true;
        }
};

template <class C> class IncomingConnection<C, void> : public Connection {
    private:
        OutputPort<C>* _port;
        C _data;
        size_t _size;

    public:
        IncomingConnection(Port& port, const string& host, const string& id):Connection(host, id){
            _port = (OutputPort<C>*)port;
            _size = sizeof(C);
        }

        bool receive(zyre_event_t* event){
            zmsg_t* msg = zyre_event_msg(event);
            if (msg == NULL){
                return false;
            }
            memcpy(&_data, zmsg_popstr(msg), _size);
            zmsg_destroy(&msg);
            _port->write(_data);
            #ifdef DEBUG
            std::cout << _host << ":" << _id << " (" << _port->getName() << ") received " << _size << " bytes from " << zyre_event_peer_name(event) << std::endl;
            #endif
            return true;
        }
};

#endif
