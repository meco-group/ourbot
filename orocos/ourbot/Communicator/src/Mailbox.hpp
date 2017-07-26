#ifndef MAILBOX_HPP
#define MAILBOX_HPP

#include <zyre.h>
#include <string.h>
#include <queue>
#include <unistd.h>

using namespace std;

class Mailbox {
    private:
        zyre_t* _node;
        string _message;
        size_t _size;
        queue<string> _inbox;
        queue<string> _senders;
        queue<string> _outbox;
        queue<string> _receivers;

        zlist_t* _peers;
        int _verbose;

    public:
        Mailbox(zyre_t* node) {
            _node = node;
            _verbose = 0;
        }

        void setVerbose(int verbose) {
            _verbose = verbose;
        }

        bool receive(zframe_t* data_frame, const string& peer, const string& peer_uuid) {
            _size = zframe_size(data_frame);
            _message.resize(_size/sizeof(char));
            memcpy(&_message[0], zframe_data(data_frame), _size);
            _inbox.push(_message);
            _senders.push(peer_uuid);
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
                if (zyre_whisper(_node, peer.c_str(), &msg)) {
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
            message = _inbox.front();
            peer = _senders.front();
            return true;
        }

        void popInbox() {
            _inbox.pop();
            _senders.pop();
        }

        void write(const string& message, const string& peer) {
            _outbox.push(message);
            _receivers.push(peer);
        }
};


#endif
