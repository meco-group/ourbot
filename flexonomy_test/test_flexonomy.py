import getch
import pyre
import time
import json
import collections as col
GROUP = 'ourbots'

node = pyre.Pyre('SH1')
node.set_interface('wlan0')
# node.set_port('5670')
node.set_port('5671')
node.start()
# node.set_verbose()
node.join(GROUP)
node.join('erveepee_flex')
node.join('kurt_flex')
node.join('dave_flex')

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

host1 = 'kurt'
host2 = 'krist'

# zones
zone_A = [2.7, 2.2, 3.141]
zone_B = [4.2, 1.2, 3.1415]
zone_C = [3.2, 0.5, 1.5702]
zone_D = [1, 1.2, 0]

while(kp != 'q'):
    kp = getch.getch()
    if (ord(kp) == 49): # keypress 1
        msg['header']['type'] = 'task_request'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = current_task_id
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameter_key'] = 'robot'
        msg['payload']['task_parameters'] = 'None'
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
        msg['payload']['task_uuid'] = current_task_id
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameter_key'] = 'robot'
        msg['payload']['task_parameters'] = 'None'
        node.shout(host1+'_flex', json.dumps(msg))
        print 'sent execute msg task Robot to ' + host1
    elif (ord(kp) == 51): # keypress 3
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = current_task_id
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameter_key'] = 'None'
        msg['payload']['task_parameters'] = zone_B
        node.shout(host1+'_flex', json.dumps(msg))
        print 'sent execute msg task B to ' + host1
    elif (ord(kp) == 52): # keypress 4
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = current_task_id
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameter_key'] = 'None'
        msg['payload']['task_parameters'] = zone_C
        node.shout(host1+'_flex', json.dumps(msg))
        print 'sent execute msg task to C ' + host1
    elif (ord(kp) == 53): # keypress 5
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = current_task_id
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameter_key'] = 'None'
        msg['payload']['task_parameters'] = zone_D
        node.shout(host1+'_flex', json.dumps(msg))
        print 'sent execute msg task to D ' + host1
    elif (ord(kp) == 54): # keypress 6
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = current_task_id
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameter_key'] = 'robot'
        msg['payload']['task_parameters'] = 'None'
        node.shout(host2+'_flex', json.dumps(msg))
        print 'sent execute msg task Robot to ' + host2
    elif (ord(kp) == 55): # keypress 7
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = current_task_id
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameter_key'] = 'None'
        msg['payload']['task_parameters'] = zone_B
        node.shout(host2+'_flex', json.dumps(msg))
        print 'sent execute msg task B to ' + host2
    elif (ord(kp) == 56): # keypress 8
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = current_task_id
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameter_key'] = 'None'
        msg['payload']['task_parameters'] = zone_C
        node.shout(host2+'_flex', json.dumps(msg))
        print 'sent execute msg task C to ' + host2
    elif (ord(kp) == 57): # keypress 9
        msg['header']['type'] = 'execute'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = current_task_id
        msg['payload']['task_type'] = 'move_to'
        msg['payload']['task_parameter_key'] = 'None'
        msg['payload']['task_parameters'] = zone_D
        node.shout(host2+'_flex', json.dumps(msg))
        print 'sent execute msg task D to ' + host2
    elif (ord(kp) == 13): # enter
        msg['header']['type'] = 'cancel'
        msg['payload'] = col.OrderedDict()
        msg['payload']['task_uuid'] = current_task_id
        node.shout('ourbots', json.dumps(msg))
        print 'remove task'
    time.sleep(0.2)
node.stop()
