#ifndef CONNECTION_HPP
#define CONNECTION_HPP

#include <zyre.h>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <string.h>
#include <queue>

#define BUFFERSIZE 1024

using namespace RTT;
using namespace std;

typedef base::PortInterface* Port;

class Connection {
    protected:
        zyre_t* _node;
        string _id;
        zlist_t* _peers;
        bool _enable;
        int _verbose;

    public:
        Connection(zyre_t* node, const string& id): _id(id), _enable(true){
            _node = node;
            _verbose = 0;
        }

        void disable(){
            _enable = false;
        }

        void enable(){
            _enable = true;
        }

        void setVerbose(int verbose){
            _verbose = verbose;
        }

        virtual bool speak(){
            return true;
        }

        virtual bool receive(char* header, zframe_t* data_frame, const string& peer){
            return true;
        }

        virtual void addGroup(const std::string& group){
            return;
        }

        virtual std::string toString(){
            std::string node_name(zyre_name(_node));
            return zyre_name(_node);
        }

        virtual std::string getSender(){
            return NULL;
        }
};

template <class C, typename T=void> class OutgoingConnection : public Connection {
    private:
        InputPort<C>* _port;
        C _data;
        std::vector<string> _groups;
        size_t _type_size;
        size_t _size;

    public:
        OutgoingConnection(Port& port, zyre_t* node, const string& id, const string& group):Connection(node, id){
            _port = (InputPort<C>*)port;
            _type_size = sizeof(T);
            addGroup(group);
        }

        void addGroup(const std::string& group){
            if (std::find(_groups.begin(), _groups.end(), group) == _groups.end()){
                _groups.push_back(group);
            }
        }

        bool speak(){
            if (_port->read(_data) == RTT::NewData){
                if (!_enable){
                    return true;
                }
                _size = _data.size()*_type_size;
                zmsg_t* msg = zmsg_new();
                zmsg_pushmem(msg, &_data[0], _size);
                const char* header = _id.c_str();
                zmsg_pushmem(msg, header, strlen(header)+1);
                for (int k=0; k<_groups.size(); k++){
                    _peers = zyre_peers_by_group(_node, _groups[k].c_str());
                    if (_peers != NULL && zlist_size(_peers) > 0){
                        if (zyre_shout(_node, _groups[k].c_str(), &msg) != 0){
                            return false;
                        }
                        if (_verbose >= 1){
                            std::cout << "[" << _id << "] " << toString() << " sending " << _size << " bytes to " << _groups[k];
                            if (_verbose >= 2){
                                std::cout << ": " << std::endl << "(";
                                for (int l=0; l<_data.size(); l++){
                                    std::cout << _data[l];
                                    if (l != _data.size()-1){
                                        std::cout << ",";
                                    }
                                }
                                std::cout << ")" << std::endl;
                            } else {
                                std::cout << std::endl;
                            }
                        }
                    }
                }
                zmsg_destroy(&msg);
            }
            return true;
        }

        std::string toString(){
            return Connection::toString() + ":" + _port->getName();
        }
};

template <class C> class OutgoingConnection<C, void> : public Connection {
    private:
        InputPort<C>* _port;
        C _data;
        std::vector<string> _groups;
        size_t _size;

    public:
        OutgoingConnection(Port& port, zyre_t* node, const string& id, const string& group):Connection(node, id){
            _port = (InputPort<C>*)port;
            _size = sizeof(C);
            addGroup(group);
        }

        void addGroup(const std::string& group){
            if (std::find(_groups.begin(), _groups.end(), group) == _groups.end()){
                _groups.push_back(group);
            }
        }

        bool speak(){
            if (_port->read(_data) == RTT::NewData){
                zmsg_t* msg = zmsg_new();
                zmsg_pushmem(msg, &_data, _size);
                const char* header = _id.c_str();
                zmsg_pushmem(msg, header, strlen(header)+1);
                for (int k=0; k<_groups.size(); k++){
                    _peers = zyre_peers_by_group(_node, _groups[k].c_str());
                    if (_peers != NULL && zlist_size(_peers) > 0){
                        if (zyre_shout(_node, _groups[k].c_str(), &msg) != 0){
                            return false;
                        }
                        if (_verbose >= 1){
                            std::cout << "[" << _id << "] " << toString() << " sending " << _size << " bytes to " << _groups[k];
                            if (_verbose >= 2){
                                std::cout << ": " << _data << std::endl;
                            } else {
                                std::cout << std::endl;
                            }
                        }
                    }
                }
                zmsg_destroy(&msg);
            }
            return true;
        }

        std::string toString(){
            return Connection::toString() + ":" + _port->getName();
        }
};

template <class C, typename T=void> class IncomingConnection : public Connection {
    private:
        OutputPort<C>* _port;
        C _data;
        size_t _type_size;
        size_t _size;
        string _sender;

    public:
        IncomingConnection(Port& port, zyre_t* node, const string& id):Connection(node, id){
            _port = (OutputPort<C>*)port;
            _type_size = sizeof(T);
        }

        bool receive(char* header, zframe_t* data_frame, const string& peer){
            if (!streq(header, _id.c_str())){
                return true;
            }
            _size = zframe_size(data_frame);
            _data.resize(_size/_type_size);
            memcpy(&_data[0], zframe_data(data_frame), _size);
            _port->write(_data);
            _sender = peer;
            if (_verbose >= 1){
                std::cout << "[" << _id << "] " << toString() << " receiving " << _size << " bytes from " << peer;
                if (_verbose >= 2){
                    std::cout << ": " << std::endl << "(";
                    for (int l=0; l<_data.size(); l++){
                        std::cout << _data[l];
                        if (l != _data.size()-1){
                            std::cout << ",";
                        }
                    }
                    std::cout << ")" << std::endl;
                } else {
                    std::cout << std::endl;
                }
            }
            return true;
        }

        std::string toString(){
            return Connection::toString() + ":" + _port->getName();
        }

        std::string getSender(){
            return _sender;
        }
};

template <class C> class IncomingConnection<C, void> : public Connection {
    private:
        OutputPort<C>* _port;
        C _data;
        size_t _size;
        string _sender;

    public:
        IncomingConnection(Port& port, zyre_t* node, const string& id):Connection(node, id){
            _port = (OutputPort<C>*)port;
            _size = sizeof(C);
        }

        bool receive(char* header, zframe_t* data_frame, const string& peer){
            if (!streq(header, _id.c_str())){
                return true;
            }
            memcpy(&_data, zframe_data(data_frame), _size);
            _port->write(_data);
            _sender = peer;
            if (_verbose >= 1){
                std::cout << "[" << _id << "] " << toString() << " receiving " << _size << " bytes from " << peer;
                if (_verbose >= 2){
                    std::cout << ": " << _data << std::endl;
                } else {
                    std::cout << std::endl;
                }
            }
            return true;
        }

        std::string toString(){
            return Connection::toString() + ":" + _port->getName();
        }

        std::string getSender(){
            return _sender;
        }
};

#endif
