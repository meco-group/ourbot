import getch
import pyre
import time
import json
import collections as col
GROUP = 'ourbots'

node = pyre.Pyre('SH1')
node.set_interface('wlan0')
node.set_port('5671')
node.start()
# node.set_verbose()
node.join(GROUP)
node.join('kurt')
node.join('dave')

header = col.OrderedDict()
header['version'] = '1.0.0'
header['type'] = 'task_request'
header['model'] = ''
header['uuid'] = str(node.uuid())
header['timestamp'] = '2017-07-07T02:20:00.000Z'
msg = col.OrderedDict()
msg['header'] = header

kp = 0
# kp = getch.getch()
# print ord(kp)

current_task_id = '71c7e94b-6ac0-4912-b707-5ef9df7b4302'

while(kp != 'q'):
    kp = getch.getch()
    if (ord(kp) == 49): # keypress 1
        msg['header']['type'] = 'task_request'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = '71c7e94b-6ac0-4912-b707-5ef9df7b4302'
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameters'] = 'A'
        msg['payload']['finished_by'] = '2017-08-07T02:20:00.000Z'
        node.shout(GROUP, json.dumps(msg))
        print 'sent task_request msg'
        m = node.recv()
        while(m[0] != 'WHISPER'):
            m = node.recv()
        data = json.loads(m[3])
        print(data)
    elif (ord(kp) == 50): # keypress 2
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = '71c7e94b-6ac0-4912-b707-5ef9df7b4302'
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameters'] = 'A'
        node.shout('dave', json.dumps(msg))
        print 'sent execute msg task A'
    elif (ord(kp) == 51): # keypress 3
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = '71c7e94b-6ac0-4912-b707-5ef9df7b4303'
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameters'] = 'B'
        node.shout('dave', json.dumps(msg))
        print 'sent execute msg task B'
    elif (ord(kp) == 52): # keypress 4
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = '71c7e94b-6ac0-4912-b707-5ef9df7b4303'
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameters'] = 'C'
        node.shout('dave', json.dumps(msg))
        print 'sent execute msg task C'
    elif (ord(kp) == 53): # keypress 5
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = '71c7e94b-6ac0-4912-b707-5ef9df7b4303'
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameters'] = 'D'
        node.shout('dave', json.dumps(msg))
        print 'sent execute msg task D'
    elif (ord(kp) == 54): # keypress 6
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = '71c7e94b-6ac0-4912-b707-5ef9df7b4302'
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameters'] = 'A'
        node.shout('kurt', json.dumps(msg))
        print 'sent execute msg task A'
    elif (ord(kp) == 55): # keypress 7
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = '71c7e94b-6ac0-4912-b707-5ef9df7b4303'
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameters'] = 'B'
        node.shout('kurt', json.dumps(msg))
        print 'sent execute msg task B'
    elif (ord(kp) == 56): # keypress 8
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = '71c7e94b-6ac0-4912-b707-5ef9df7b4303'
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameters'] = 'C'
        node.shout('kurt', json.dumps(msg))
        print 'sent execute msg task C'
    elif (ord(kp) == 57): # keypress 9
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = '71c7e94b-6ac0-4912-b707-5ef9df7b4303'
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameters'] = 'D'
        node.shout('kurt', json.dumps(msg))
        print 'sent execute msg task D'
    time.sleep(0.1)
node.stop()
