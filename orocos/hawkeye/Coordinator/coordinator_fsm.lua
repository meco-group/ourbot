local tc = rtt.getTC()

local reporter      = tc:getPeer('reporter')
local hawkeye       = tc:getPeer('hawkeye')

local hawkeyeUpdate           = hawkeye:getOperation("update")
local hawkeyeInRunTimeError   = hawkeye:getOperation("inRunTimeError")
local snapshot                = reporter:getOperation("snapshot")

return rfsm.state {

  rfsm.trans{src = 'initial',               tgt = 'main'},
  rfsm.trans{src = 'main',                  tgt = 'failure',        events = {'e_failed'}},
  rfsm.trans{src = 'main.idle',             tgt = 'main',           events = {'e_idle'}},
  rfsm.trans{src = 'failure',               tgt = 'main',           events = {'e_recover'}},

  rfsm.trans{src = 'initial',               tgt = 'idle'},
  rfsm.trans{src = 'idle',                  tgt = 'init',           events = {'e_init'}},
  rfsm.trans{src = 'init',                  tgt = 'run',            events = {'e_run'}},
  rfsm.trans{src = 'run',                   tgt = 'stop',           events = {'e_stop'}},
  rfsm.trans{src = 'stop',                  tgt = 'run',            events = {'e_restart'}},
  rfsm.trans{src = 'stop',                  tgt = 'reset',          events = {'e_reset'}},
  rfsm.trans{src = 'reset',                 tgt = 'idle',           events = {'e_idle'}},
  rfsm.trans
  rfsm.trans{src = 'failure',               tgt = 'idle',           events = {'e_recover'}},
    --add more state transitions here

  initial = rfsm.conn{},

  failure = rfsm.state{
    entry = function()
      _coordinator_failure_event_port:write('e_failed')
    end
  },

  main = rfsm.state{
    rfsm.trans{src = 'initial', tgt = 'idle'},
    rfsm.trans{src = 'idle',    tgt = 'init',   events = {'e_init'}},
    rfsm.trans{src = 'init',    tgt = 'run',    events = {'e_run'}},
    rfsm.trans{src = 'run',     tgt = 'stop',   events = {'e_stop'}},
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
      entry=function()
        print("Waiting on Run...")
      end
    },

    run = rfsm.state{
      entry = function(fsm)
        if not hawkeye:start() then
          rtt.logl("Error","Could not start hawkeye component")
          rfsm.send_events(fsm,'e_failed')
          return
        end
        if not reporter:start() then
          rtt.logl("Error","Could not start reporter component")
          rfsm.send_events(fsm,'e_failed')
          return
        end
        print("System started. Abort by using Break.")
      end,

      doo = function(fsm)
        snapshot_cnt = 0
        period = tc:getPeriod()
        max_cnt = 1/(reporter_sample_rate*period)
        start_time = get_sec()
        prev_start_time = start_time
        end_time   = start_time
        init       = 0
        while true do
          prev_start_time = start_time
          start_time      = get_sec()

          -- update hawkeye
          hawkeyeUpdate()

          -- take snapshot for logger
          if snapshot_cnt > max_cnt then
            snapshot:send()
            snapshot_cnt = 0
          else
            snapshot_cnt = snapshot_cnt + 1
          end

          if hawkeyeInRunTimeError() then
            rtt.logl("Error","RunTimeError in hawkeye component")
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
        hawkeye:stop()
        reporter:stop()
        print("System stopped. Waiting on Restart or Reset...")
      end,
    },

    reset = rfsm.state{
      entry = function(fsm)
      end,
    }
  }
}
