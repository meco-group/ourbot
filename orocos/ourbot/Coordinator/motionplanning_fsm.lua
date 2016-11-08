local tc = rtt.getTC()

local communicator    = tc:getPeer('communicator')
local motionplanning  = tc:getPeer('motionplanning')
local controller      = tc:getPeer('controller')
local estimator       = tc:getPeer('estimator')
local reference       = tc:getPeer('reference')
local reporter        = tc:getPeer('reporter')
local io              = tc:getPeer('io')
local teensy          = tc:getPeer('teensy')

local estimatorUpdate              = estimator:getOperation("update")
local referenceUpdate              = reference:getOperation("update")
local controllerUpdate             = controller:getOperation("update")
local validEstimation              = estimator:getOperation("validEstimation")
local validReference               = reference:getOperation("ready")
local disablePort                  = communicator:getOperation("disablePort")
local estimatorInRunTimeError      = estimator:getOperation("inRunTimeError")
local controllerInRunTimeError     = controller:getOperation("inRunTimeError")
local referenceInRunTimeError      = reference:getOperation("inRunTimeError")
local motionplanningInRunTimeError = motionplanning:getOperation("inRunTimeError")
local strongVelocityControl        = teensy:getOperation("strongVelocityControl")
local snapshot                     = reporter:getOperation("snapshot")

-- variables for the timing diagnostics
local jitter    = 0
local duration  = 0

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'idle'},
  rfsm.trans{src = 'idle',    tgt = 'init',   events = {'e_init'}},
  rfsm.trans{src = 'init',    tgt = 'run',    events = {'e_run'}},
  rfsm.trans{src = 'run',     tgt = 'stop',   events = {'e_stop', 'e_failed'}},
  rfsm.trans{src = 'stop',    tgt = 'run',    events = {'e_restart'}},  -- no reinitiliaziation
  rfsm.trans{src = 'stop',    tgt = 'reset',  events = {'e_reset'}},    -- with reinitialization
  rfsm.trans{src = 'reset',   tgt = 'idle',   events = {'e_done'}},

  initial = rfsm.conn{},

  idle = rfsm.state{
    entry=function()
      print("Waiting on Initialize...")
    end
  },

  init = rfsm.state{
    entry = function(fsm)
      disablePort(4002) -- disable velocity command via gamepad
      if not io:start() then
        rtt.logl("Error","Could not start io component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
      strongVelocityControl()
      print("Waiting on Run...")
    end
  },

  run = rfsm.state{
    entry = function(fsm)
      if not motionplanning:start() then
        rtt.logl("Error","Could not start motionplanning component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
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
      print("System started. Abort by using Break.")
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

        -- update reference/estimator/controller
        estimatorUpdate()
        if validEstimation() then
          referenceUpdate()
          if validReference() then
            controllerUpdate()
          end
        else
          print "Estimate not valid!"
        end

        -- take snapshot for logger
        if snapshot_cnt >= max_cnt then
          snapshot:send()
          snapshot_cnt = 1
        else
          snapshot_cnt = snapshot_cnt + 1
        end

        if controllerInRunTimeError() or estimatorInRunTimeError() or referenceInRunTimeError() then
          rtt.logl("Error","RunTimeError in controller/estimator/reference component")
          rfsm.send_events(fsm,'e_failed')
          return
        end
        if motionplanningInRunTimeError() then
          rtt.logl("Error","RunTimeError in motionplanning")
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
      motionplanning:stop()
      estimator:stop()
      reference:stop()
      controller:stop()
      reporter:stop()
      print("System stopped. Waiting on Restart or Reset...")
    end,
  },

  reset = rfsm.state{
    entry = function(fsm)
      io:stop()
    end,
  }
}
