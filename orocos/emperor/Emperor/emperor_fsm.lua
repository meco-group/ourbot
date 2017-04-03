local tc        = rtt.getTC()
local communicator  = tc:getPeer('communicator')

return rfsm.state {

  rfsm.trans{src = 'initial',                 tgt = 'idle'},
  rfsm.trans{src = 'idle',                    tgt = 'motionplanning',     events = {'e_motionplanning'}},
  rfsm.trans{src = 'idle',                    tgt = 'velocitycmd',        events = {'e_velocitycmd'}},
  rfsm.trans{src = 'idle',                    tgt = 'trajectoryfollowing',events = {'e_trajectoryfollowing'}},
  rfsm.trans{src = 'motionplanning',          tgt = 'failure',            events = {'e_failed'}},
  rfsm.trans{src = 'velocitycmd',             tgt = 'failure',            events = {'e_failed'}},
  rfsm.trans{src = 'trajectoryfollowing',     tgt = 'failure',            events = {'e_failed'}},
  rfsm.trans{src = 'motionplanning.idle',     tgt = 'idle',               events = {'e_done'}},
  rfsm.trans{src = 'velocitycmd.idle',        tgt = 'idle',               events = {'e_done'}},
  rfsm.trans{src = 'trajectoryfollowing.idle',tgt = 'idle',               events = {'e_done'}},
  rfsm.trans{src = 'failure',                 tgt = 'idle',               events = {'e_recover'}},

  initial = rfsm.conn{},

  idle = rfsm.state{
    entry=function()
      main_state='idle'
      print('\nSelect mode...\n')
    end
  },

  failure = rfsm.state{
    entry = function()
      main_state='failure'
      _emperor_failure_event_port:write('e_failed')
      rtt.logl("Error","System in Failure!")
    end
  },

  motionplanning      = rfsm.load("Emperor/motionplanning_fsm.lua"),
  velocitycmd         = rfsm.load("Emperor/velocitycmd_fsm.lua"),
  trajectoryfollowing = rfsm.load("Emperor/trajectoryfollowing_fsm.lua")
}
