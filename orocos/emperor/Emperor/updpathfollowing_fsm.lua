
return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'idle'},
  rfsm.trans{src = 'idle', tgt = 'init', events = {'e_init'}},
  rfsm.trans{src = 'init', tgt = 'run', events = {'e_run'}},
  rfsm.trans{src = 'run', tgt = 'stop', events = {'e_stop'}},
  rfsm.trans{src = 'stop', tgt = 'run', events = {'e_restart'}},
  rfsm.trans{src = 'stop', tgt = 'reset', events = {'e_reset'}},
  rfsm.trans{src = 'reset', tgt = 'idle'},

  idle  = rfsm.state{ entry = function() print("Waiting on Initialize...") end },
  init  = rfsm.state{ },
  run   = rfsm.state{ },
  stop  = rfsm.state{ },
  reset = rfsm.state{ },

}
