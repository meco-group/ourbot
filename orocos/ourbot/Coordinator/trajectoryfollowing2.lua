require './Coordinator/subcoordinator'

TrajectoryFollowingCoordinator = {}
TrajectoryFollowingCoordinator.__index = TrajectoryFollowingCoordinator

setmetatable(TrajectoryFollowingCoordinator, {
    __index = SubCoordinator,
    __call = function (cls, ...)
    local self = setmetatable({}, cls)
    self:_init(...)
    return self
  end,
})

function TrajectoryFollowingCoordinator._init(self, init)
  SubCoordinator._init(self)
  self.load_trajectory = self.reference:getOperation('loadTrajectory')
  self.ref_reset = self.reference:getOperation('reset')
  self.ref_ready = self.reference:getOperation('ready')
  self.trajectory_path = self.reference:getProperty('trajectory_path'):get()
end

function TrajectoryFollowingCoordinator.get_fsm(self)
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
        if not self:start_control_components() then
          rfsm.send_events(fsm, 'e_failed')
        end
        if not self.load_trajectory(self.trajectory_path) then
          rtt.logl('Warning', 'Trajectory could not be loaded!')
          rfsm.send_events(fsm, 'e_failed')
        end
      end,
    },

    idle = rfsm.state{
      entry = function(fsm)
        self.teensy:setVelocity(0, 0, 0)
        self:enable_manualcommand()
        self.ref_reset()
      end,

      doo = function(fsm)
        while true do
          if not self:control_hook(false) then
            rfsm.send_events(fsm, 'e_failed')
          end
          rfsm.yield(true)
        end
      end,
    },

    following = rfsm.state{
      entry = function(fsm)
        self:disable_manualcommand()
        local pose = self.estimator:getEstimatedPose()
        self.reference:setPoseOffset(pose[0], pose[1], pose[2]) -- start from local pose
        print('started following trajectory')
      end,

      doo = function(fsm)
        while not self.ref_ready() do
          if not self:control_hook(true) then
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
        self:stop_control_components()
        rfsm.send_events(fsm, 'e_leave')
      end,
    },
  }
end
