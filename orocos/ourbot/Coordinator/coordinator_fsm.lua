return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'idle', tgt = 'trajectoryfollowing', events = {'e_trajectoryfollowing'}},
  rfsm.trans{src = 'idle', tgt = 'motionplanning', events = {'e_motionplanning'}},
  rfsm.trans{src = 'idle', tgt = 'flexonomy', events = {'e_flexonomy'}},

  rfsm.trans{src = 'init', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'idle', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'trajectoryfollowing', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'motionplanning.failure', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'flexonomy', tgt = 'failure', events = {'e_failed'}},

  rfsm.trans{src = 'trajectoryfollowing.stop', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'motionplanning.stop', tgt = 'idle', events = {'e_done'}},

  initial = rfsm.conn{},

  init = rfsm.state{},

  failure = rfsm.state{
    entry = function()
      zero_velocity()
      coordinator_failure_event_port:write('e_failed')
    end
  },

  idle = rfsm.state{
    entry = function(fsm)
      print('select a state (trajectoryfollowing, motionplanning, flexonomy)')
      coordinator_send_event_port:write('e_idle') -- to inform emperor
      zero_velocity()
      enable_manualcommand()
    end,

    doo = function(fsm)
      while true do
        if not control_hook(false) then
          rfsm.send_events(fsm, 'e_failed')
        end
        rfsm.yield(true)
      end
    end,
  },

  trajectoryfollowing = rfsm.load('Coordinator/trajectoryfollowing_fsm.lua'),
  motionplanning = rfsm.load('Coordinator/motionplanning_fsm.lua'),
  flexonomy = rfsm.load('Coordinator/flexonomy_fsm.lua'),
}


