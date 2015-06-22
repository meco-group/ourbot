local tc = rtt.getTC()

local pathgenerator = tc:getPeer(pathgenerator)
local controller    = tc:getPeer(controller)
local estimator     = tc:getPeer(estimator)
local reference     = tc:getPeer(reference)
local reporter      = tc:getPeer(reporter)
--dummy/add here sensor+actuator components
local sensors       = tc:getPeer(sensors)

local estimatorUpdate           = estimator:getOperation("update")
local referenceUpdate           = reference:getOperation("update")
local controllerUpdate          = controller:getOperation("update")
local estimatorInRunTimeError   = estimator:getOperation("inRunTimeError")
local controllerInRunTimeError  = controller:getOperation("inRunTimeError")
local referenceInRunTimeError   = reference:getOperation("inRunTimeError")
local snapshot                  = reporter:getOperation("snapshot")

--Variable for the timing diagnostics
local jitter = 0
local duration = 0
local jitter_max = 0
local jitter_total = 0

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'idle'},

  idle = rfsm.state{entry=function() print("Waiting on Initialize...") end},

  rfsm.trans{src = 'idle', tgt = 'init', events = {'e_init'}},

  init = rfsm.state{
    entry = function(fsm)
      --Write here start command of sensor and actuator components
      if not sensors:start() then
        rtt.logl("Error","Could not start sensors component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
    end
  },

  rfsm.trans{src = 'init', tgt = 'run', events = {'e_run'}},

  run = rfsm.state{
    entry = function(fsm)
      if not pathgenerator:start() then
        rtt.logl("Error","Could not start pathgenerator component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
      if not reporter:start() then
        rtt.log("Error","Could not start reporter component")
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
      init       = 0
      while true do
        prev_start_time = start_time
        start_time      = get_sec()
        --Update the reference value
        referenceUpdate()
        --Update estimate
        estimatorUpdate()
        --Update the control algorithm
        controllerUpdate()

        --Take snapshot for logger
        snapshot:send()

        end_time = get_sec()
        if controllerInRunTimeError() or estimatorInRunTimeError() or referenceInRunTimeError() then
          rtt.logl("Error","RunTimeError in controller/estimator/reference component")
          rfsm.send_events(fsm,'e_failed')
        end
        rfsm.yield(true)--Setting true means sleeping until next sample period
        --We ditch the first two calculations due to the initially wrongly calculated prev_start_time
        if init > 2 then
          duration = (end_time - start_time) * 1000
          if duration/1000 > period*0.9 then
            rtt.logl('Warning','ControlLoop: Duration of calculation exceeded 90% of sample period')
          end

          jitter = (start_time - prev_start_time - period) * 1000
          if jitter/1000 > 0.1 * period then
            rtt.logl('Warning','ControlLoop: Jitter exceeded 10% of sample period')
          end
          jitter_max = math.max(jitter_max,math.abs(jitter))
        else
          init = init+1
        end
        rfsm.yield()
      end
    end,
  },

  rfsm.trans{src = 'run', tgt = 'stop', events = {'e_stop'}},

  stop = rfsm.state{
    entry = function(fsm)
      pathgenerator:stop()
      estimator:stop()
      reference:stop()
      controller:stop()
      reporter:stop()
    end,
  },
  --You can restart the control loop by sending 'e_restart', it will not reinitialize the sensors
  rfsm.trans{src = 'stop', tgt = 'run', events = {'e_restart'}},
  --If you want to reinitialize the plant you need to call 'e_reset'
  rfsm.trans{src = 'stop', tgt = 'reset', events = {'e_reset'}},

  reset = rfsm.state{
    entry = function(fsm)
      --Call stop functions of sensors and actuators here.
      sensors:stop()
    end,
  },
  rfsm.trans{src='reset',tgt='idle'},
}
