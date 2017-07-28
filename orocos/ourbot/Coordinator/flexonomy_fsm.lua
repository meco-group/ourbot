require "./Coordinator/flexonomy_fun"

local tc = rtt.getTC()

local jitter = 0
local duration = 0

local reference = tc:getPeer('reference')
local motionplanning = tc:getPeer('motionplanning')
local teensy = tc:getPeer('teensy')

local setVelocity = teensy:getOperation('setVelocity')
local setTargetPose = motionplanning:getOperation('setTargetPose')
local disableMotionPlanning = motionplanning:getOperation('disable')
local referenceUpdate = reference:getOperation("update")

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init',    tgt = 'home',   events = {'e_done'}},
  rfsm.trans{src = 'home',    tgt = 'idle',   events = {'e_done'}},
  rfsm.trans{src = 'idle',    tgt = 'p2p',    events = {'e_p2p'}},
  rfsm.trans{src = 'p2p',     tgt = 'idle',   events = {'e_done'}},

  rfsm.trans{src = 'p2p',     tgt = 'stoptask', events = {'e_stop'}},
  rfsm.trans{src = 'stoptask',tgt = 'idle',     events = {'e_done'}},

  rfsm.trans{src = 'init',    tgt = 'failure',events = {'e_failure'}},
  rfsm.trans{src = 'home',    tgt = 'failure',events = {'e_failure'}},
  rfsm.trans{src = 'idle',    tgt = 'failure',events = {'e_failure'}},
  rfsm.trans{src = 'p2p',     tgt = 'failure',events = {'e_failure'}},

  rfsm.trans{src = 'failure',  tgt = 'init',   events = {'e_done'}},


  initial = rfsm.conn{},

  init = rfsm.state{
    entry = function(fsm)
      init(fsm)
    end,

    doo = function(fsm)
      -- wait for valid estimate
      waitForValidEstimate(fsm)
    end
  },

  home = rfsm.state{

    doo = function(fsm)
      home(fsm)
    end
  },

  idle = rfsm.state{

    entry = function(fsm)
      setVelocity(0, 0, 0)
    end,

    doo = function(fsm)
      while true do
        -- perform default loop update
        if not update(fsm, 'ready', false) then
          return
        end
        setVelocity(0, 0, 0) -- why??
        -- check for new task
        local t = executeNextTask()
        if t ~= nil then
          rfsm.send_events(fsm,'e_p2p')
        end
        rfsm.yield(true)
      end
    end,
  },

  p2p = rfsm.state{

    entry = function(fsm)
      referenceUpdate() -- necessary for reset()'ing reference comp
      local pose = getTargetPose(current_task)
      setTargetPose(pose[1], pose[2], pose[3])
    end,

    doo = function(fsm)
      local period = tc:getPeriod()
      local start_time = get_sec()
      local prev_start_time = start_time
      local end_time   = start_time
      local init       = 0
      task_started = false
      while true do
        prev_start_time = start_time
        start_time = get_sec()
        -- perform default loop update
        if not update(fsm, 'busy', true) then
          return
        end
        -- check status of motion planning
        if not checkMotionPlanning(fsm) then
          return
        end
        -- check timings of previous iteration
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

  stoptask = rfsm.state{
    entry = function(fsm)
      disableMotionPlanning()
    end
  },

  failure = rfsm.state{
    entry = function(fsm)
      stop()
      sendTaskStatus(current_task, 'failed', get_sec())
    end,
  },

}
