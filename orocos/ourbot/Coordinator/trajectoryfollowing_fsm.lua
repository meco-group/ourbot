local load_trajectory = reference:getOperation('loadTrajectory')
local ref_reset = reference:getOperation('reset')
local ref_ready = reference:getOperation('ready')
local trajectory_path = reference:getProperty('trajectory_path'):get()

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'idle', tgt = 'following', events = {'e_next'}},
  rfsm.trans{src = 'following', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'idle', tgt = 'stop', events = {'e_back'}},
  rfsm.trans{src = 'following', tgt = 'idle', events = {'e_back'}},

  initial = rfsm.conn{},

  init = rfsm.state{
    entry = function(fsm)
      if not start_control_components() then
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
      zero_velocity()
      enable_manualcommand()
      ref_reset()
    end,

    doo = function(fsm)
      while true do
        if not control_hook(false) then
          rfsm.send_events(fsm, 'e_failed')
        end
        rfsm.yield(true)
      end
    end,
  },

  following = rfsm.state{
    entry = function(fsm)
      disable_manualcommand()
      local pose = estimator:getEstimatedPose()
      reference:setPoseOffset(pose[0], pose[1], pose[2]) -- start from local pose
      print('started following trajectory')
    end,

    doo = function(fsm)
      while not ref_ready() do
        if not control_hook(true) then
          rfsm.send_events(fsm, 'e_failed')
        end
        rfsm.yield(true)
      end
    end,

    exit = function(fsm)
      print('stopped following trajectory')
    end,
  },

  stop = rfsm.state {
    entry = function(fsm)
      stop_control_components()
      rfsm.send_events(fsm, 'e_leave')
    end,
  },
}
