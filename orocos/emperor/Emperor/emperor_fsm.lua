local tc = rtt.getTC()

local communicator = tc:getPeer('communicator')
local gamepad = tc:getPeer('gamepad')

local state_index = 1
local states = {'trajectoryfollowing', 'motionplanning', 'motionplanning_dynamic_obstacle', 'formation', 'flexonomy'}

local velcmd_index = 1
local velcmd_receivers = {'ourbots', 'dave', 'kurt', 'krist'}

function state_toggle()
  local fs_up, data_up = gamepad_up_port:read()
  local fs_down, data_down = gamepad_down_port:read()
  local fs_A, data_A = gamepad_A_port:read()
  local fs_B, data_B = gamepad_B_port:read()
  if ((fs_up == 'NewData') and data_up) then
    state_index = (state_index)%table.getn(states)+1
    print('selected state: ' .. states[state_index])
  end
  if ((fs_down == 'NewData') and data_down) then
    state_index = (state_index-2)%table.getn(states)+1
    print('selected state: ' .. states[state_index])
  end
  if ((fs_A == 'NewData') and data_A) then
    print('entering state ' .. states[state_index])
    emperor_send_event_port:write('e_'..states[state_index])
    return true
  elseif ((fs_B == 'NewData') and data_B) then
    emperor_send_event_port:write('e_back')
  end
  return false
end

function substate_toggle()
  local fs_A, data_A = gamepad_A_port:read()
  local fs_B, data_B = gamepad_B_port:read()
  if ((fs_A == 'NewData') and data_A) then
    emperor_send_event_port:write('e_next')
  elseif ((fs_B == 'NewData') and data_B) then
    emperor_send_event_port:write('e_back')
  end
end

function velcmd_toggle()
  local fs_lb, data_lb = gamepad_lb_port:read()
  if ((fs_lb == 'NewData') and data_lb) then
    velcmd_index = (velcmd_index)%table.getn(velcmd_receivers)+1
    communicator:setConnectionGroup('gamepad', 'cmd_velocity_port', 'cmd_velocity', velcmd_receivers[velcmd_index])
    print('sending velocity command to ' .. velcmd_receivers[velcmd_index])
  end
end

local _target_pose_port = rtt.OutputPort('array')
tc:addPort(_target_pose_port, 'target_pose_port', 'Target pose')
if not communicator:addOutgoingConnection('emperor', 'target_pose_port', 'target_pose', 'ourbots') then
  rfsm.send_events(fsm, 'e_failed')
end


return rfsm.state {

  rfsm.trans{src = 'initial', tgt = 'idle'},
  rfsm.trans{src = 'idle', tgt = 'init', events = {'e_done'}},
  rfsm.trans{src = 'init', tgt = 'state', events = {'e_done'}},
  rfsm.trans{src = 'state', tgt = 'idle', events = {'e_idle'}},
  rfsm.trans{src = 'init', tgt = 'failure', events = {'e_failure'}},
  rfsm.trans{src = 'idle', tgt = 'failure', events = {'e_failure'}},
  rfsm.trans{src = 'idle', tgt = 'failure', events = {'e_failure'}},
  rfsm.trans{src = 'state', tgt = 'failure', events = {'e_failure'}},

  initial = rfsm.conn{},

  init = rfsm.state{},

  idle = rfsm.state{
    doo = function(fsm)
      print('selected state: ' .. states[state_index])
      while(true) do
        velcmd_toggle()
        if state_toggle() then
          break
        end
        rfsm.yield(true)
      end
    end,
  },

  state = rfsm.state{
    doo = function(fsm)
      while(true) do
        velcmd_toggle()
        substate_toggle()
        rfsm.yield(true)
      end
    end,
  },

  failure = rfsm.state{
    entry = function(fsm)
      reporter:stop()
      emperor_failure_event_port:write('e_failed')
      rtt.logl('Error', 'System in Failure!')
    end
  },

}
