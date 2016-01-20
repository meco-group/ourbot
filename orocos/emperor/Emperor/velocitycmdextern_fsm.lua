local tc        = rtt.getTC()

local gamepad     = tc:getPeer('gamepad')
local reporter    = tc:getPeer('reporter')
local snapshot    = reporter:getOperation("snapshot")

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'idle'},
  rfsm.trans{src = 'idle',    tgt = 'init',   events = {'e_init'}},
  rfsm.trans{src = 'init',    tgt = 'run',    events = {'e_run'}},
  rfsm.trans{src = 'run',     tgt = 'stop',   events = {'e_stop'}},
  rfsm.trans{src = 'stop',    tgt = 'run',    events = {'e_restart'}},
  rfsm.trans{src = 'stop',    tgt = 'reset',  events = {'e_reset'}},
  rfsm.trans{src = 'reset',   tgt = 'idle'},

  idle  = rfsm.state{ entry = function() main_state = 'velocitycmdextern' sub_state='idle' print("Waiting on Init (Button A)...") end },
  init  = rfsm.state{ entry = function() sub_state='init' print("Waiting on Run (Button A)...") end},
  run   = rfsm.state{
    entry = function()
      sub_state='run'
      if (not reporter:start()) then
        rtt.log("Error","Could not start reporter component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
      print("System started. Abort by using Break (Button B).")
    end,

    doo = function()
      while true do
        snapshot:send()
        rfsm.yield(true)
      end
    end,
  },

  stop = rfsm.state{
    entry = function(fsm)
      sub_state='stop'
      reporter:stop()
      print("System stopped. Waiting on Restart (Button A) or Reset (Button B)...")
    end,
  },

  reset = rfsm.state{ },

}
