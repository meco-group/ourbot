
return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'idle'},

  idle = rfsm.state{entry=function() main_state='idle' print('\nSelect mode...\n') end},

  rfsm.trans{src = 'idle', tgt = 'motionplanning', events={'e_motionplanning'}},
  rfsm.trans{src = 'idle', tgt = 'velocitycmd', events={'e_velocitycmd'}},
    --add more state transitions here

  --If an 'e_failed' event is raised, we drop out to the failure state
  rfsm.trans{src = 'motionplanning', tgt = 'failure', events={'e_failed'}},
  rfsm.trans{src = 'velocitycmd', tgt = 'failure', events={'e_failed'}},
    --add more state transitions here

  --Get back to idle state
  rfsm.trans{src = 'motionplanning.idle', tgt = 'idle', events={'e_idle'}},
  rfsm.trans{src = 'velocitycmd.idle', tgt = 'idle', events={'e_idle'}},
    --add more state transitions here

  --There is currently no logic to recover from a failure except telling others of failure
  failure = rfsm.state{entry = function() main_state='failure' rtt.logl("Error","System in Failure!") _emperor_failure_event_port:write('e_failed') end},

  motionplanning    = rfsm.load("Emperor/motionplanning_fsm.lua"),
  velocitycmd       = rfsm.load("Emperor/velocitycmd_fsm.lua")
    --add more state descriptions here

}
