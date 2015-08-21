local tc        = rtt.getTC()

local reporter  = tc:getPeer('reporter')
local snapshot  = reporter:getOperation("snapshot")

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'idle'},
  rfsm.trans{src = 'idle',    tgt = 'init',   events = {'e_init'}},
  rfsm.trans{src = 'init',    tgt = 'run',    events = {'e_run'}},
  rfsm.trans{src = 'run',     tgt = 'stop',   events = {'e_stop'}},
  rfsm.trans{src = 'stop',    tgt = 'run',    events = {'e_restart'}},
  rfsm.trans{src = 'stop',    tgt = 'reset',  events = {'e_reset'}},
  rfsm.trans{src = 'reset',   tgt = 'idle'},

  idle  = rfsm.state{ entry = function() print("Waiting on Init...") end },
  init  = rfsm.state{ entry = function() print("Waiting on Run...") end},
  run   = rfsm.state{
    entry = function()
      if (not reporter:start()) then
        rtt.log("Error","Could not start reporter component")
        rfsm.send_events(fsm,'e_failed')
        return
      end

      print("System started. Abort by using Break.")
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
      reporter:stop()
      print("System stopped. Waiting on Restart or Reset...")
    end,
  },

  reset = rfsm.state{ },

}
