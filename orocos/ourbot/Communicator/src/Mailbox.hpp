#ifndef MAILBOX_HPP
#define MAILBOX_HPP

#include <zyre.h>
#include <string.h>
#include <vector>
#include <queue>
#include <unistd.h>

using namespace std;

class Mailbox {
    private:
        zyre_t* _node;
        string _message;
        size_t _size;
        vector<string> _inbox;
        vector<string> _senders;
        queue<string> _outbox;
        queue<string> _receivers;
        int _msg_index;

        zlist_t* _peers;
        int _verbose;

    public:
        Mailbox(zyre_t* node) {
            _node = node;
            _verbose = 0;
            _msg_index = -1;
        }

        void setVerbose(int verbose) {
            _verbose = verbose;
        }

        bool receive(zframe_t* data_frame, const string& peer, const string& peer_uuid) {
            _size = zframe_size(data_frame);
            _message.resize(_size/sizeof(char));
            memcpy(&_message[0], zframe_data(data_frame), _size);
            _inbox.push_back(_message);
            _senders.push_back(peer_uuid);
            if (_verbose >= 1) {
                std::cout << "[mail] receiving " << _size << " bytes from " << peer;
                if (_verbose >= 2) {
                    std::cout << ": " << _message << std::endl;
                } else {
                    std::cout << std::endl;
                }
            }
            return true;
        }

        bool send() {
            while (!_outbox.empty()) {
                _message = _outbox.front();
                string peer = _receivers.front();
                _outbox.pop();
                _receivers.pop();
                _size = _message.size()*sizeof(char);
                zmsg_t* msg = zmsg_new();
                zmsg_pushmem(msg, &_message[0], _size);
                if (zyre_whisper(_node, peer.c_str(), &msg) != 0) {
                    return false;
                }
                if (_verbose >= 1) {
                    std::cout << "[mail] sending " << _size << " bytes to " << peer;
                    if (_verbose >= 2) {
                        std::cout << ": " << _message << std::endl;
                    } else {
                        std::cout << std::endl;
                    }
                }
                zmsg_destroy(&msg);
            }
            return true;
        }

        bool read(string& message, string& peer) {
            if (_inbox.empty()) {
                return false;
            }
            _msg_index = (_msg_index+1)%_inbox.size();
            message = _inbox.at(_msg_index);
            peer = _senders.at(_msg_index);
            return true;
        }

        void remove() {
            if (_msg_index == -1) {
                return;
            }
            _inbox.erase(_inbox.begin() + _msg_index);
            _senders.erase(_senders.begin() + _msg_index);
            _msg_index--;
        }

        void write(const string& message, const string& peer) {
            _outbox.push(message);
            _receivers.push(peer);
        }
};


#endif
