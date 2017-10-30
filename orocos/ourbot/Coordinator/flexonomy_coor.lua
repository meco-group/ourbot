FlexonomyCoordinator = {}
FlexonomyCoordinator.__index = FlexonomyCoordinator

setmetatable(FlexonomyCoordinator, {
    __index = MotionPlanningCoordinator,
    __call = function (cls, ...)
    local self = setmetatable({}, cls)
    self:_init(...)
    return self
  end,
})

function FlexonomyCoordinator._init(self, tc, control_coor, motionplanning_coor)
  self.tc = tc
  self.control_coor = control_coor
  self.motionplanning_coor = motionplanning_coor
  -- self.current_task = nil
  -- self.current_eta = nil
  -- self.task_queue = {}
  -- self.time_tbl = {}
  -- self.current_task_canceled = false
  -- self.priority = false
  -- self.state = 'busy'

  -- self.peer_uuid = self.communicator:getOperation('getPeerUUID')
  -- self.write_mail = self.communicator:getOperation('writeMail')
  -- self.read_mail = self.communicator:getOperation('readMail')
  -- self.remove_mail = self.communicator:getOperation('removeMail')
  -- self.msg_tbl = {
  --   header = {
  --     version = self.header_version,
  --     type = '',
  --     model = '',
  --     uuid = self.communicator:getUUID(),
  --     timestamp = ''
  --   },
  --   payload = {}
  -- }
end

function FlexonomyCoordinator.get_fsm(self)
  local fsm = self.motionplanning_coor:get_fsm()

  fsm.initial = rfsm.conn{}

  fsm.init = rfsm.state{
      entry = function(fsm)
        if not self.motionplanning_coor.deployer:loadComponent('motionplanning', 'MotionPlanning') then
          rfsm.send_events(fsm, 'e_failed')
        end
        if not self.motionplanning_coor:set_motionplanner(self.deployer:getPeer('motionplanning')) then
          rfsm.send_events(fsm, 'e_failed')
          return
        end
        self.motionplanning_coor.motionplanning:start()
        if not self.control_coor:start() then
          rfsm.send_events(fsm, 'e_failed')
        end
      end
    }

  fsm.home = rfsm.state{
      -- wait until valid estimate
      doo = function(fsm)
        while true do
          if not self.control_coor:control_hook(false) then
            rfsm.send_events(fsm, 'e_failed')
          end
          if true or self.control_coor.estimator_valid() then
            return
          end
          rfsm.yield(true)
        end
      end
    }

  fsm.idle = rfsm.state{
      entry = function(fsm)
        self.control_coor:zero_velocity()
        self.control_coor:enable_manualcommand()
      end,

      doo = function(fsm)
        while true do
          if not self.control_coor:control_hook(false) or self.motionplanning_coor.motionplanning_error() then
            rfsm.send_events(fsm, 'e_failed')
          end
          local fs, target_pose = self.motionplanning_coor.target_pose_port:read()
          if fs == 'NewData' then
            self.motionplanning_coor.mp_reset()
            self.motionplanning_coor.motionplanning:setTargetPose(target_pose)
            rfsm.send_events(fsm, 'e_p2p')
          end
          rfsm.yield(true)
        end
      end,
    }

    fsm.p2p0 = rfsm.state{

      doo = function(fsm)
        self.control_coor:disable_manualcommand()
        local busy = self.motionplanning_coor.mp_busy()
        local busy_n
        self.motionplanning_coor:trigger_motionplanning(0)
        self.motionplanning_coor.ref_cnt = 0
        self.motionplanning_coor.mp_failure_cnt = 0
        self.motionplanning_coor.mp_recover_cnt = 0
        while true do
          if not self.control_coor:control_hook(false) or self.motionplanning_coor.motionplanning_error() then
            rfsm.send_events(fsm, 'e_failed')
          end
          busy_n = self.mp_busy()
          if (busy or busy_n and busy ~= busy_n) and not busy_n then -- decreasing flank detection
            if self.mp_ready() then
              print 'Target reached.'
              rfsm.send_events(fsm, 'e_idle')
              return
            end
            if self.motionplanning_coor.mp_valid() then
              self.motionplanning_coor.load_new_trajectory(0)
              return
            else
              rtt.logl('Error', 'Motionplanning could not find a trajectory.')
              rfsm.send_events(fsm, 'e_idle')
              return
            end
          end
          busy = busy_n
          rfsm.yield(true)
        end
      end,
    }

    fsm.p2p = rfsm.state{
      doo = function(fsm)
        local predict_shift = 0
        while true do

          if self.motionplanning_coor.ref_cnt == 0 then
            self.motionplanning_coor:trigger_motionplanning(predict_shift)
          end
          if not self.control_coor:control_hook(true) or self.motionplanning_coor.motionplanning_error() then
            rfsm.send_events(fsm, 'e_failed')
          end
          -- check motion planning
          if self.motionplanning_coor.mp_ready() then
            print 'Target reached.'
            return
          end
          -- motion planning check is related to next update
          self.motionplanning_coor.ref_cnt = self.motionplanning_coor.ref_cnt + 1
          -- e.g. new trajectories are followed, beginning in the nex update
          -- This behavior is similar as putting thte motion planning check in
          -- start of update. But using the current implementation,  the
          -- control_hook update is synced/clocked better.
          if not self.motionplanning_coor.mp_busy() and self.motionplanning_coor.ref_cnt >= self.motionplanning_coor.mp_period then
            if self.motionplanning_coor.mp_valid() then
              predict_shift = math.max(0, self.motionplanning_coor.ref_cnt - self.motionplanning_coor.mp_period)
              self.motionplanning_coor.load_new_trajectory(predict_shift)
              self.motionplanning_coor.ref_cnt = 0
            else
              -- try again
              self.motionplanning_coor.mp_failure_cnt = self.motionplanning_coor.mp_failure_cnt + 1
              if self.motionplanning_coor.mp_failure_cnt < self.motionplanning_coor.mp_max_failures then
                self.motionplanning_coor:trigger_motionplanning(math.max(0, self.motionplanning_coor.ref_cnt - self.motionplanning_coor.mp_period))
              else
                rtt.logl('Error', 'Motionplanning got ' .. self.motionplanning_coor.mp_max_failures .. ' consecutive invalid solutions. Recover...')
                rfsm.send_events(fsm, 'e_recover')
                return
              end
            end
          end
          if self.motionplanning_coor.ref_cnt > self.motionplanning_coor.mp_max_periods*self.mp_period then
            rtt.logl('Error', 'Motionplanning takes too long. Recover...')
            rfsm.send_events(fsm, 'e_recover')
            return
          end
          if self.motionplanning_coor.ref_cnt >= self.motionplanning_coor.mp_period and math.fmod(self.motionplanning_coor.ref_cnt, self.motionplanning_coor.mp_period) == 0 then
            rtt.logl('Warning', 'Motionplanning takes longer than '.. math.floor(self.motionplanning_coor.ref_cnt/self.motionplanning_coor.mp_period) ..' period(s)!')
          end
          rfsm.yield(true)
        end
      end,
    }

    fsm.recover = rfsm.state{
      doo = function(fsm)
        self.motionplanning_coor.mp_recover_cnt = self.motionplanning_coor.mp_recover_cnt + 1
        if self.motionplanning_coor.mp_recover_cnt >= self.motionplanning_coor.mp_max_recovers then
          rtt.logl('Error', 'Motionplanning recovered ' .. self.motionplanning_coor.mp_max_recovers .. ' times. Giving up...')
          return
        end
        local time0 = get_sec()
        local recover_time = 1.
        while true do
          if not self.control_coor:control_hook(false) or self.motionplanning_coor.motionplanning_error() then
            rfsm.send_events(fsm, 'e_failed')
          end
          -- wait a little bit and try again
          if get_sec() - time0 >= recover_time then
            rfsm.send_events(fsm, 'e_p2p')
          end
          rfsm.yield(true)
        end
      end,
    }

    fsm.stop = rfsm.state {
      entry = function(fsm)
        self.motionplanning_coor:rm_motionplanner()
        self.control_coor:stop()
      end,
    }

  return fsm
  -- return self.motionplanning_coor:get_fsm()
end
