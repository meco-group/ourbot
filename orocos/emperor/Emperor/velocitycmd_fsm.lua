local tc        = rtt.getTC()

local gamepad               = tc:getPeer('gamepad')
local hawkeye               = tc:getPeer('hawkeye')
local reporter              = tc:getPeer('reporter')
local snapshot              = reporter:getOperation("snapshot")
local gamepadInRunTimeError = gamepad:getOperation("inRunTimeError")
local hawkeyeInRunTimeError = hawkeye:getOperation("inRunTimeError")

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init',    tgt = 'run',    events = {'e_done'}},
  rfsm.trans{src = 'init',    tgt = 'stop',   events = {'e_stop'}},
  rfsm.trans{src = 'run',     tgt = 'stop',   events = {'e_stop'}},
  rfsm.trans{src = 'stop',    tgt = 'init',   events = {'e_restart'}},
  rfsm.trans{src = 'stop',    tgt = 'idle',   events = {'e_reset'}},

  initial = rfsm.conn{},

  init  = rfsm.state{
    entry = function(fsm)
      main_state = 'velocitycmd'
      sub_state = 'run'
      print("Initializing...")
      if (not reporter:start()) then
        rtt.log("Error","Could not start reporter component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
    end
  },

  run   = rfsm.state{
    entry = function(fsm)
      print("Let's roll... Abort by using Break (Button B).")
    end,

    doo = function(fsm)
      period = tc:getPeriod()
      max_cnt = 1/(reporter_sample_rate*period)
      snapshot_cnt = max_cnt
      while true do
        -- take snapshot for logger
        if snapshot_cnt >= max_cnt then
          snapshot:send()
          snapshot_cnt = 1
        else
          snapshot_cnt = snapshot_cnt + 1
        end

        if gamepadInRunTimeError() then
          rtt.logl("Error","RunTimeError in gamepad component")
          rfsm.send_events(fsm,'e_failed')
          return
        end
        if hawkeyeInRunTimeError() then
          rtt.logl("Error","RunTimeError in hawkeye component")
          rfsm.send_events(fsm,'e_failed')
          return
        end

        rfsm.yield(true)
      end
    end,
  },

  stop = rfsm.state{
    entry = function(fsm)
      sub_state = 'stop'
      reporter:stop()
      print("System stopped. Waiting on Restart (Button A) or Reset (Button B)...")
    end,
  },

  idle = rfsm.state{}

}
