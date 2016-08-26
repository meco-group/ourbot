return rfsm.state {

  rfsm.trans{src = 'initial',               tgt = 'idle'},
  rfsm.trans{src = 'idle',                  tgt = 'motionplanning', events = {'e_motionplanning'}},
  rfsm.trans{src = 'idle',                  tgt = 'velocitycmd',    events = {'e_velocitycmd'}},
  rfsm.trans{src = 'motionplanning',        tgt = 'failure',        events = {'e_failed'}},
  rfsm.trans{src = 'velocitycmd',           tgt = 'failure',        events = {'e_failed'}},
  rfsm.trans{src = 'motionplanning.idle',   tgt = 'idle',           events = {'e_idle'}},
  rfsm.trans{src = 'velocitycmd.idle',      tgt = 'idle',           events = {'e_idle'}},
  rfsm.trans{src = 'failure',               tgt = 'idle',           events = {'e_recover'}},
    --add more state transitions here

  initial = rfsm.conn{},

  idle = rfsm.state{
    entry=function()
      print('\nWaiting for command...\nPossibilities: VelocityControl, MotionPlanning')
    end
  },

  failure = rfsm.state{
    entry = function()
      _coordinator_failure_event_port:write('e_failed')
    end
  },

  motionplanning = rfsm.load("Coordinator/motionplanning_fsm.lua"),
  velocitycmd    = rfsm.load("Coordinator/velocitycmd_fsm.lua"),
    --add more state descriptions here

}
