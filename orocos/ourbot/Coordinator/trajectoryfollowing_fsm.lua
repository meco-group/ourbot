local fun = require './Coordinator/functions'

local tc = rtt.getTC()
local reference = tc:getPeer('reference')
local teensy = tc:getPeer('teensy')
local estimator = tc:getPeer('estimator')

local ref_reset = reference:getOperation('reset')
local ref_ready = reference:getOperation('ready')
local load_trajectory = reference:getOperation('loadTrajectory')

local trajectory_path = reference:getProperty('trajectory_path'):get()

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'idle', tgt = 'following', events = {'e_run'}},
  rfsm.trans{src = 'following', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'idle', tgt = 'stop', events = {'e_stop'}},
  rfsm.trans{src = 'following', tgt = 'stop', events = {'e_stop'}},

  initial = rfsm.conn{},

  init = rfsm.state{
    entry = function(fsm)
      if not fun:start() then
        rfsm.send_events(fsm, 'e_failed')
      end
      if not load_trajectory(trajectory_path) then
        rtt.logl('Warning', 'Trajectory could not be loaded!')
        rfsm.send_events(fsm, 'e_failed')
      end
    end,
  },

  idle = rfsm.state{
    entry = function(fsm)
      teensy:setVelocity(0, 0, 0)
      fun:enable_manualcommand()
      ref_reset()
    end,

    doo = function(fsm)
      while true do
        if not fun:control_hook(false) then
          rfsm.send_events(fsm, 'e_failed')
        end
        rfsm.yield(true)
      end
    end,
  },

  following = rfsm.state{
    entry = function(fsm)
      fun:disable_manualcommand()
      local pose = estimator:getEstimatedPose()
      reference:setPoseOffset(-pose[0], -pose[1], -pose[2]) -- start from local pose
    end,

    doo = function(fsm)
      local start_time = fun:get_sec()
      local prev_start_time = start_time
      local end_time = start_time
      while not ref_ready() do
        prev_start_time = start_time
        start_time = fun:get_sec()
        if not fun:control_hook(true) then
          rfsm.send_events(fsm, 'e_failed')
        end
        fun:guard_time(start_time, prev_start_time, end_time)
        end_time = fun:get_sec()
        rfsm.yield(true)
      end
    end,
  },

  stop = rfsm.state {
    entry = function(fsm)
      fun:stop()
    end,
  },

}
