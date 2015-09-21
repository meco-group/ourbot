local tc        = rtt.getTC()

local velocitycmd = tc:getPeer('velocitycmd')
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

  idle  = rfsm.state{ entry = function() state='velocitycmdintern.idle' print("Waiting on Init...") end },
  init  = rfsm.state{ entry = function() state='velocitycmdintern.init' print("Waiting on Run...") end},
  run   = rfsm.state{
    entry = function()
      state='velocitycmdintern.run'
      if (not reporter:start()) then
        rtt.log("Error","Could not start reporter component")
        rfsm.send_events(fsm,'e_failed')
        return
      end

      if not gamepad:start() then
        rtt.log("Error","Could not start gamepad component")
        rfsm.send_events(fsm,'e_failed')
        return
      end

      if not velocitycmd:start() then
        rtt.logl("Error","Could not start velocitycmd component")
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
      state='velocitycmdintern.stop'
      reporter:stop()
      velocitycmd:stop()

      print("System stopped. Waiting on Restart or Reset...")
    end,
  },

  reset = rfsm.state{ },

}
