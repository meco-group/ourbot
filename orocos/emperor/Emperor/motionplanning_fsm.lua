local tc        = rtt.getTC()

local communicator          = tc:getPeer('communicator')
local gamepad               = tc:getPeer('gamepad')
local reporter              = tc:getPeer('reporter')
local hawkeye               = tc:getPeer('hawkeye')
local snapshot              = reporter:getOperation("snapshot")
local gamepadInRunTimeError = gamepad:getOperation("inRunTimeError")
local hawkeyeInRunTimeError = hawkeye:getOperation("inRunTimeError")

function connectPorts()
  local addOutgoing = communicator:getOperation("addOutgoing")
  if not addOutgoing('hawkeye', 'obstacle_port', 6070, robots) then rfsm.send_events(fsm, 'e_failed') return end
end

function disconnectPorts()
  local removeConnection = communicator:getOperation("removeConnection")
  removeConnection(6070)
end

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'idle'},
  rfsm.trans{src = 'idle',    tgt = 'init',   events = {'e_init'}},
  rfsm.trans{src = 'init',    tgt = 'run',    events = {'e_run'}},
  rfsm.trans{src = 'run',     tgt = 'stop',   events = {'e_stop'}},
  rfsm.trans{src = 'stop',    tgt = 'run',    events = {'e_restart'}},
  rfsm.trans{src = 'stop',    tgt = 'reset',  events = {'e_reset'}},
  rfsm.trans{src = 'reset',   tgt = 'idle'},

  initial = rfsm.conn{},
  idle  = rfsm.state{
    entry = function(fsm)
      main_state = 'motionplanning'
      sub_state='idle'
      print("Waiting on Init (Button A)...")
    end
  },

  init  = rfsm.state{
    entry = function(fsm)
      connectPorts() -- connect obstacle port
      sub_state='init'
      print("Waiting on Run (Button A)...")
      if (not reporter:start()) then
        rtt.log("Error","Could not start reporter component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
    end
  },

  run   = rfsm.state{
    entry = function(fsm)
      sub_state='run'
      print("System started. Abort by using Break (Button B).")
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
      sub_state='stop'
      print("System stopped. Waiting on Restart (Button A) or Reset (Button B)...")
    end,
  },

  reset = rfsm.state{
    entry = function(fsm)
      reporter:stop()
      disconnectPorts()
    end,
  },

}
