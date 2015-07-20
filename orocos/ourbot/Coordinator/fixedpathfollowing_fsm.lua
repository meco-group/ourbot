local tc = rtt.getTC()

local controller    = tc:getPeer('controller'..tostring(index))
local estimator     = tc:getPeer('estimator'..tostring(index))
local reference     = tc:getPeer('reference'..tostring(index))
local reporter      = tc:getPeer('reporter'..tostring(index))
-- dummy/add here sensor+actuator components
local io            = tc:getPeer('io'..tostring(index))

local estimatorUpdate           = estimator:getOperation("update")
local referenceUpdate           = reference:getOperation("update")
local controllerUpdate          = controller:getOperation("update")
local estimatorInRunTimeError   = estimator:getOperation("inRunTimeError")
local controllerInRunTimeError  = controller:getOperation("inRunTimeError")
local referenceInRunTimeError   = reference:getOperation("inRunTimeError")
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
    end
  },

  run = rfsm.state{
    entry = function(fsm)
      if not reporter:start() then
        rtt.logl("Error","Could not start reporter component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
      if not reference:start() or not estimator:start() or not controller:start() then
        rtt.logl("Error","Could not start reference/estimator/controller component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
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

        -- update reference/estimator/controller
        referenceUpdate()
        estimatorUpdate()
        controllerUpdate()

        -- take snapshot for logger
        snapshot:send()

        if controllerInRunTimeError() or estimatorInRunTimeError() or referenceInRunTimeError() then
          rtt.logl("Error","RunTimeError in controller/estimator/reference component")
          rfsm.send_events(fsm,'e_failed')
          return
        end

        -- check timings of previous iteration
        -- ditch the first two calculations due to the initially wrongly calculated prev_start_time
        if init > 2 then
          duration = (end_time - prev_start_time) * 1000
          if duration > 900*period then
            rtt.logl('Warning','ControlLoop: Duration of calculation exceeded 90% of sample period')
          end
          jitter = (start_time - prev_start_time - period) * 1000
          if jitter > 100.*period then
            rtt.logl('Warning','ControlLoop: Jitter exceeded 10% of sample period')
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
      estimator:stop()
      reference:stop()
      controller:stop()
      reporter:stop()
    end,
  },

  reset = rfsm.state{
    entry = function(fsm)
      io:stop()
    end,
  }
}
