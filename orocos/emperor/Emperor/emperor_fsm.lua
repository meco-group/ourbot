
return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'idle'},

  idle = rfsm.state{entry=function() main_state='idle' print('\nSelect mode...\n') end},

  rfsm.trans{src = 'idle', tgt = 'updpathfollowing', events={'e_updpathfollowing'}},
  rfsm.trans{src = 'idle', tgt = 'fixedpathfollowing', events={'e_fixedpathfollowing'}},
  rfsm.trans{src = 'idle', tgt = 'velocitycmdintern', events={'e_velocitycmdintern'}},
  rfsm.trans{src = 'idle', tgt = 'velocitycmdextern', events={'e_velocitycmdextern'}},
    --add more state transitions here

  --If an 'e_failed' event is raised, we drop out to the failure state
  rfsm.trans{src = 'updpathfollowing', tgt = 'failure', events={'e_failed'}},
  rfsm.trans{src = 'fixedpathfollowing', tgt = 'failure', events={'e_failed'}},
  rfsm.trans{src = 'velocitycmdintern', tgt = 'failure', events={'e_failed'}},
  rfsm.trans{src = 'velocitycmdextern', tgt = 'failure', events={'e_failed'}},
    --add more state transitions here

  --Get back to idle state
  rfsm.trans{src = 'updpathfollowing.idle', tgt = 'idle', events={'e_idle'}},
  rfsm.trans{src = 'fixedpathfollowing.idle', tgt = 'idle', events={'e_idle'}},
  rfsm.trans{src = 'velocitycmdintern.idle', tgt = 'idle', events={'e_idle'}},
  rfsm.trans{src = 'velocitycmdextern.idle', tgt = 'idle', events={'e_idle'}},
    --add more state transitions here

  --There is currently no logic to recover from a failure except telling others of failure
  failure = rfsm.state{entry = function() main_state='failure' rtt.logl("Error","System in Failure!") _emperor_failure_event_port:write('e_failed') end},

  updpathfollowing  = rfsm.load("Emperor/updpathfollowing_fsm.lua"),
  fixedpathfollowing= rfsm.load("Emperor/fixedpathfollowing_fsm.lua"),
  velocitycmdintern = rfsm.load("Emperor/velocitycmdintern_fsm.lua"),
  velocitycmdextern = rfsm.load("Emperor/velocitycmdextern_fsm.lua")
    --add more state descriptions here

}
