local tc = rtt.getTC()

--dummy, load teensy and sensors
local sensors     = tc:getPeer(sensors)
local velocitycmd = tc:getPeer(velocitycmd)
local estimator   = tc:getPeer(estimator)
local reporter    = tc:getPeer(reporter)

local estimatorUpdate           = estimator:getOperation("update")
local estimatorInRunTimeError   = estimator:getOperation("inRunTimeError")
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
      --start teensy and sensors
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
      if not reporter:start() then
        rtt.log("Error","Could not start reporter component")
        return
      end
      if not velocitycmd:start() then
        rtt.logl("Error","Could not start velocitycmd component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
      if not estimator:start() then
        rtt.logl("Error","Could not start estimator component")
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

        --Update estimator
        estimatorUpdate()

        --Take snapshot for logger
        snapshot:send()

        end_time = get_sec()
        if estimatorInRunTimeError() then
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
      estimator:stop()
      velocitycmd:stop()
      reporter:stop()
    end,
  },

  rfsm.trans{src = 'stop', tgt = 'run', events = {'e_restart'}},

  rfsm.trans{src = 'stop', tgt = 'reset', events = {'e_reset'}},

  reset = rfsm.state{
    entry = function(fsm)
      --Call stop functions of sensors and actuators here.
      sensors:stop()
    end,
  },

  rfsm.trans{src = 'reset', tgt = 'idle'},
}

