local tc = rtt.getTC()

local scanmatcher   = tc:getPeer('scanmatcher'..tostring(index))
local estimator     = tc:getPeer('estimator'..tostring(index))
local reporter      = tc:getPeer('reporter'..tostring(index))
local io            = tc:getPeer('io'..tostring(index))

local estimatorUpdate           = estimator:getOperation("update")
local estimatorInRunTimeError   = estimator:getOperation("inRunTimeError")
local scanmatcherInRunTimeError = scanmatcher:getOperation("inRunTimeError")
local snapshot                  = reporter:getOperation("snapshot")

-- variables for the timing diagnostics
local jitter    = 0
local duration  = 0

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'idle'},
  rfsm.trans{src = 'idle',    tgt = 'init',   events = {'e_init'}},
  rfsm.trans{src = 'init',    tgt = 'run',    events = {'e_run'}},
  rfsm.trans{src = 'run',     tgt = 'stop',   events = {'e_stop'}},
  rfsm.trans{src = 'stop',    tgt = 'run',    events = {'e_restart'}},  --No reinitiliaziation
  rfsm.trans{src = 'stop',    tgt = 'reset',  events = {'e_reset'}},    --With reinitialization
  rfsm.trans{src = 'reset',   tgt = 'idle',   events = {'e_done'}},

  idle = rfsm.state{entry=function() print("Waiting on Initialize...") end},

  init = rfsm.state{
    entry = function(fsm)
      if not io:start() then
        rtt.logl("Error","Could not start io component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
      print("Waiting on Run...")
    end
  },

  run = rfsm.state{
    entry = function(fsm)
      if not scanmatcher:start() then
        rtt.logl("Error","Could not start scanmatcher component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
      if not reporter:start() then
        rtt.logl("Error","Could not start reporter component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
      if not estimator:start() then
        rtt.logl("Error","Could not start estimator component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
      print("System started. Abort by using Break.")
    end,

    doo = function(fsm)
      period     = tc:getPeriod()
      start_time = get_sec()
      prev_start_time = start_time
      end_time   = start_time
      init       = 0
      while true do
        prev_start_time = start_time
        start_time      = get_sec()

        -- update estimator
        estimatorUpdate()

        -- take snapshot for logger
        snapshot:send()

        if estimatorInRunTimeError() then
          rtt.logl("Error","RunTimeError in estimator component")
          rfsm.send_events(fsm,'e_failed')
          return
        end
        if scanmatcherInRunTimeError() then
          rtt.logl("Error","RunTimeError in scanmatcher component")
          rfsm.send_events(fsm,'e_failed')
          return
        end

        -- check timings of previous iteration
        -- ditch the first two calculations due to the initially wrongly calculated prev_start_time
        if init > 2 then
          duration = (end_time - prev_start_time) * 1000
          jitter = (start_time - prev_start_time - period) * 1000
          if print_level >= 1 then
            if duration > 900*period then
              rtt.logl('Warning','ControlLoop: Duration of calculation exceeded 90% of sample period')
            end
            if jitter > 100.*period then
              rtt.logl('Warning','ControlLoop: Jitter exceeded 10% of sample period')
            end
          end
          _controlloop_duration:write(duration)
          _controlloop_jitter:write(jitter)
        else
          init = init+1
        end

        end_time = get_sec()

        rfsm.yield(true) -- sleep until next sample period
      end
    end,
  },

  stop = rfsm.state{
    entry = function(fsm)
      scanmatcher:stop()
      estimator:stop()
      reporter:stop()
      print("System stopped. Waiting on Restart or Reset...")
    end,
  },

  reset = rfsm.state{
    entry = function(fsm)
      if not io:stop() then
        rtt.logl("Error","Could not stop io component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
    end,
  }
}
