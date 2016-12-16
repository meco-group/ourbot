local tc = rtt.getTC()

local motionplanning      = tc:getPeer('motionplanning')
local distrmotionplanning = tc:getPeer('distrmotionplanning')
local controller          = tc:getPeer('controller')
local estimator           = tc:getPeer('estimator')
local reference           = tc:getPeer('reference')
local reporter            = tc:getPeer('reporter')
local io                  = tc:getPeer('io')
local teensy              = tc:getPeer('teensy')

local estimatorUpdate              = estimator:getOperation("update")
local referenceUpdate              = reference:getOperation("update")
local controllerUpdate             = controller:getOperation("update")
local validEstimation              = estimator:getOperation("validEstimation")
local validReference               = reference:getOperation("ready")
local estimatorInRunTimeError      = estimator:getOperation("inRunTimeError")
local controllerInRunTimeError     = controller:getOperation("inRunTimeError")
local referenceInRunTimeError      = reference:getOperation("inRunTimeError")
local snapshot                     = reporter:getOperation("snapshot")

-- variables for the timing diagnostics
local jitter    = 0
local duration  = 0

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init',    tgt = 'home',   events = {'e_done'}},
  rfsm.trans{src = 'home',    tgt = 'run',    events = {'e_run'}},
  rfsm.trans{src = 'init',    tgt = 'stop',   events = {'e_stop'}},
  rfsm.trans{src = 'home',    tgt = 'stop',   events = {'e_stop'}},
  rfsm.trans{src = 'run',     tgt = 'stop',   events = {'e_stop'}},
  rfsm.trans{src = 'stop',    tgt = 'init',   events = {'e_restart'}},
  rfsm.trans{src = 'stop',    tgt = 'idle',   events = {'e_reset'}},

  initial = rfsm.conn{},

  init = rfsm.state{
    entry = function(fsm)
      nr_coop_ourbots = communicator:getGroupSize("ourbots") - communicator:getGroupSize("obstacle")
      print(nr_coop_ourbots)
      if (nr_coop_ourbots == 1) then
        mp = motionplanning
      else
        mp = distrmotionplanning
      end
      if not io:start() then
        rtt.logl("Error","Could not start io component")
        rfsm.send_events(fsm,'e_failed')
        return
      end
      if not mp:start() then
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
      teensy:strongVelocityControl()
    end,

    doo = function(fsm)
      -- wait on valid estimate
      while true do
        estimatorUpdate()
        if validEstimation() then
          break
        else
          print "Estimate not valid!"
        end
        if estimatorInRunTimeError() then
          rtt.logl("Error","RunTimeError in estimator component")
          rfsm.send_events(fsm,'e_failed')
          return
        end
        rfsm.yield(true)
      end
      pose0 = mp:setConfiguration(nr_coop_ourbots)
    end
  },

  home = rfsm.state{
    entry = function(fsm)
      mp:setTargetPose(pose0[0], pose0[1], pose0[2])
    end,

    doo = function(fsm)
      while true do
        if not mp:gotTarget() then
          break
        end
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
        if controllerInRunTimeError() or estimatorInRunTimeError() or referenceInRunTimeError() then
          rtt.logl("Error","RunTimeError in controller/estimator/reference component")
          rfsm.send_events(fsm,'e_failed')
          return
        end
        if mp:inRunTimeError() then
          rtt.logl("Error","RunTimeError in motionplanning")
          rfsm.send_events(fsm,'e_failed')
          return
        end
        rfsm.yield(true) -- sleep until next sample period
      end
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
        if mp:inRunTimeError() then
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
      mp:stop()
      estimator:stop()
      reference:stop()
      controller:stop()
      reporter:stop()
      io:stop()
      print("System stopped. Waiting on Restart or Reset...")
    end,
  },

  idle = rfsm.state{}
}
