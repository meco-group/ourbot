local tc = rtt.getTC()

-- local scanmatcher   = tc:getPeer('scanmatcher')
local communicator  = tc:getPeer('communicator')
local estimator     = tc:getPeer('estimator')
local reporter      = tc:getPeer('reporter')
local io            = tc:getPeer('io')
local teensy        = tc:getPeer('teensy')

local estimatorUpdate           = estimator:getOperation("update")
local estimatorInRunTimeError   = estimator:getOperation("inRunTimeError")
-- local scanmatcherInRunTimeError = scanmatcher:getOperation("inRunTimeError")
local snapshot                  = reporter:getOperation("snapshot")

-- variables for the timing diagnostics
local jitter    = 0
local duration  = 0

function connectPorts()
  if not communicator:addIncomingConnection('io', 'cmd_velocity_port', 'cmd_velocity') then rfsm.send_events(fsm, 'e_failed') return end
end

function disconnectPorts()
  communicator:removeConnection('io', 'cmd_velocity_port', 'cmd_velocity')
end

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init',    tgt = 'run',    events = {'e_done'}},
  rfsm.trans{src = 'init',    tgt = 'stop',   events = {'e_stop'}},
  rfsm.trans{src = 'run',     tgt = 'stop',   events = {'e_stop'}},
  rfsm.trans{src = 'stop',    tgt = 'init',   events = {'e_restart'}},
  rfsm.trans{src = 'stop',    tgt = 'idle',   events = {'e_reset'}},

  initial = rfsm.conn{},

  init = rfsm.state{
    entry = function(fsm)
      print("Initializing...")
      connectPorts() -- connect gamepad velocity command
      if not io:start() then
        rtt.logl("Error","Could not start io component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
      -- if not scanmatcher:start() then
      --   rtt.logl("Error","Could not start scanmatcher component")
      --   rfsm.send_events(fsm,'e_failed')
      --   return
      -- end
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
      teensy:softVelocityControl()
    end
  },

  run = rfsm.state{
    entry = function(fsm)
      print "Let's roll..."
    end,

    doo = function(fsm)
      period = tc:getPeriod()
      max_cnt = 1/(reporter_sample_rate*period)
      snapshot_cnt = max_cnt
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
        if snapshot_cnt >= max_cnt then
          snapshot:send()
          snapshot_cnt = 1
        else
          snapshot_cnt = snapshot_cnt + 1
        end

        if estimatorInRunTimeError() then
          rtt.logl("Error","RunTimeError in estimator component")
          rfsm.send_events(fsm,'e_failed')
          return
        end
        -- if scanmatcherInRunTimeError() then
        --   rtt.logl("Error","RunTimeError in scanmatcher component")
        --   rfsm.send_events(fsm,'e_failed')
        --   return
        -- end

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
      -- scanmatcher:stop()
      estimator:stop()
      reporter:stop()
      io:stop()
      disconnectPorts()
      print("System stopped. Waiting on Restart or Reset...")
    end,
  },

  idle = rfsm.state{},
}
