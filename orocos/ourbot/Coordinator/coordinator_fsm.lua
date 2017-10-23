local fun = require './Coordinator/functions'
local tc = rtt.getTC()
local teensy = tc:getPeer('teensy')

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'idle', tgt = 'motionplanning', events = {'e_motionplanning'}},
  rfsm.trans{src = 'idle', tgt = 'trajectoryfollowing', events = {'e_trajectoryfollowing'}},

  rfsm.trans{src = 'init', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'idle', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'motionplanning', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'trajectoryfollowing', tgt = 'failure', events = {'e_failed'}},

  rfsm.trans{src = 'motionplanning.stop', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'trajectoryfollowing.stop', tgt = 'idle', events = {'e_done'}},

  initial = rfsm.conn{},

  init = rfsm.state{},

  idle = rfsm.state{
    entry = function(fsm)
      print('select a state (motionplanning, trajectoryfollowing)')
      _coordinator_send_event_port:write('e_idle') -- to inform emperor
      teensy:setVelocity(0, 0, 0)
      fun:enable_manualcommand()
    end,

    doo = function(fsm)
      while true do
        if not fun:control_hook(false) then
          rfsm.send_events(fsm, 'e_failed')
        end
        rfsm.yield(true)
      end
    end,
  },

  failure = rfsm.state{
    entry = function()
      _coordinator_failure_event_port:write('e_failed')
    end
  },

  motionplanning = rfsm.load('Coordinator/motionplanning_fsm.lua'),
  trajectoryfollowing = rfsm.load('Coordinator/trajectoryfollowing_fsm.lua'),
}
