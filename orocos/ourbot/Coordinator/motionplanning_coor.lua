MotionPlanningCoordinator = {}
MotionPlanningCoordinator.__index = MotionPlanningCoordinator

setmetatable(MotionPlanningCoordinator, {
    __call = function (cls, ...)
    local self = setmetatable({}, cls)
    self:_init(...)
    return self
  end,
})

function MotionPlanningCoordinator._init(self, tc, control_coor)
  self.tc = tc
  self.control_coor = control_coor
  self.deployer = self.tc:getPeer('Deployer')
  self.reference = self.tc:getPeer('reference')
  self.communicator = self.tc:getPeer('communicator')
  self.load_new_trajectory = self.reference:getOperation('receiveTrajectory')
  self.control_rate = self.tc:getProperty('control_rate'):get()
  self.motionplanning_rate = self.tc:getProperty('motionplanning_rate'):get()
  self.mp_period = math.floor(self.control_rate/self.motionplanning_rate)
  self.est_pose_port = self.tc:getPort('est_pose_port')
end

function MotionPlanningCoordinator.set_motionplanner(self, mp)
  self.motionplanning = mp
  self.motionplanning:loadService('marshalling')
  if not self.motionplanning:provides('marshalling'):updateProperties('Configuration/system-config.cpf') then
    return false
  end
  if not self.motionplanning:provides('marshalling'):updateProperties('Configuration/motionplanning-config.cpf') then
    return false
  end
  self.mp_trigger_port = rtt.OutputPort('array')
  self.tc:addPort(self.mp_trigger_port, 'mp_trigger_port', 'Trigger for motion planning: is composed of current estimated pose and start index of internal reference input vector')
  self.target_pose_port = rtt.InputPort('array')
  self.tc:addPort(self.target_pose_port, 'target_pose_port', 'Target pose')
  if not self.communicator:addIncomingConnection('coordinator', 'target_pose_port', 'target_pose') then
    rfsm.send_events(fsm, 'e_failed')
  end
  if not self.deployer:connectPorts('motionplanning', 'estimator') then
    return false
  end
  if not self.deployer:connectPorts('motionplanning', 'reference') then
    return false
  end
  if not self.deployer:connectPorts('motionplanning', 'coordinator') then
    return false
  end
  if not self.deployer:addPeer('communicator', 'motionplanning') then
    return false
  end
  if not self.motionplanning:configure() then
    return false
  end
  self.deployer:setActivity('motionplanning', 0, 0, rtt.globals.ORO_SCHED_OTHER)

  self.motionplanning_error = self.motionplanning:getOperation('inRunTimeError')
  self.mp_reset = self.motionplanning:getOperation('reset')
  self.mp_busy = self.motionplanning:getOperation('busy')
  self.mp_valid = self.motionplanning:getOperation('valid')
  self.mp_ready = self.motionplanning:getOperation('ready')
  self.get_coeffs = self.motionplanning:getOperation('getCoefficients')
  self.get_basis_length = self.motionplanning:getOperation('getBasisLength')
  self.reset_obstacles = self.motionplanning:getOperation('resetObstacles')
  self.add_static_obstacle = self.motionplanning:getOperation('addStaticObstacle')
  self.add_dynamic_obstacle = self.motionplanning:getOperation('addDynamicObstacle')
  self.add_peer_obstacle = self.motionplanning:getOperation('addPeerObstacle')
  self.mp_max_failures = self.motionplanning:getProperty('max_failures'):get()
  self.mp_max_recovers = self.motionplanning:getProperty('max_recovers'):get()
  self.mp_max_periods = self.motionplanning:getProperty('max_periods'):get()
  return true
end

function MotionPlanningCoordinator.rm_motionplanner(self)
  self.motionplanning:stop()
  self.deployer:unloadComponent('motionplanning')
end

function MotionPlanningCoordinator.trigger_motionplanning(self, predict_shift)
  local _, pose0 = self.est_pose_port:read()
  local trigger_data = rtt.Variable('array')
  trigger_data:resize(4)
  trigger_data:fromtab{pose0[0], pose0[1], pose0[2], predict_shift}
  self.mp_trigger_port:write(trigger_data)
end

function MotionPlanningCoordinator.load_obstacles(self)
  self.reset_obstacles()
  -- self.add_static_obstacle()
end


-- function MotionPlanningCoordinator.get_fsm(self)
--   return rfsm.state {
--     rfsm.trans{src = 'initial', tgt = 'init'},
--     rfsm.trans{src = 'init', tgt = 'home', events = {'e_done'}},
--     rfsm.trans{src = 'home', tgt = 'idle', events = {'e_done'}},
--     rfsm.trans{src = 'idle', tgt = 'p2p0', events = {'e_p2p'}},
--     rfsm.trans{src = 'p2p0', tgt = 'idle', events = {'e_idle'}},
--     rfsm.trans{src = 'p2p0', tgt = 'p2p', events = {'e_done'}},
--     rfsm.trans{src = 'p2p', tgt = 'idle', events = {'e_done'}},
--     rfsm.trans{src = 'p2p', tgt = 'recover', events = {'e_recover'}},
--     rfsm.trans{src = 'recover', tgt = 'p2p0', events = {'e_p2p'}},
--     rfsm.trans{src = 'recover', tgt = 'idle', events = {'e_done'}},
--     rfsm.trans{src = 'home', tgt = 'stop', events = {'e_back'}},
--     rfsm.trans{src = 'idle', tgt = 'stop', events = {'e_back'}},
--     rfsm.trans{src = 'p2p0', tgt = 'idle', events = {'e_back'}},
--     rfsm.trans{src = 'p2p', tgt = 'idle', events = {'e_back'}},
--     rfsm.trans{src = 'recover', tgt = 'idle', events = {'e_back'}},

--     initial = rfsm.conn{},

--     init = rfsm.state{
--       entry = function(fsm)
--         if not self.deployer:loadComponent('motionplanning', 'MotionPlanning') then
--           rfsm.send_events(fsm, 'e_failed')
--         end
--         if not self:set_motionplanner(self.deployer:getPeer('motionplanning')) then
--           rfsm.send_events(fsm, 'e_failed')
--           return
--         end
--         self.motionplanning:start()
--         if not self.control_coor:start() then
--           rfsm.send_events(fsm, 'e_failed')
--         end
--       end
--     },

--     home = rfsm.state{
--       -- wait until valid estimate
--       doo = function(fsm)
--         while true do
--           if not self.control_coor:control_hook(false) then
--             rfsm.send_events(fsm, 'e_failed')
--           end
--           if true or self.control_coor.estimator_valid() then
--             return
--           end
--           rfsm.yield(true)
--         end
--       end
--     },

--     idle = rfsm.state{
--       entry = function(fsm)
--         self.control_coor:zero_velocity()
--         self.control_coor:enable_manualcommand()
--       end,

--       doo = function(fsm)
--         while true do
--           if not self.control_coor:control_hook(false) or self.motionplanning_error() then
--             rfsm.send_events(fsm, 'e_failed')
--           end
--           local fs, target_pose = self.target_pose_port:read()
--           if fs == 'NewData' then
--             self.mp_reset()
--             self.motionplanning:setTargetPose(target_pose)
--             rfsm.send_events(fsm, 'e_p2p')
--           end
--           rfsm.yield(true)
--         end
--       end,
--     },

--     p2p0 = rfsm.state{

--       doo = function(fsm)
--         self.control_coor:disable_manualcommand()
--         local busy = self.mp_busy()
--         local busy_n
--         self:trigger_motionplanning(0)
--         self.ref_cnt = 0
--         self.mp_failure_cnt = 0
--         self.mp_recover_cnt = 0
--         while true do
--           if not self.control_coor:control_hook(false) or self.motionplanning_error() then
--             rfsm.send_events(fsm, 'e_failed')
--           end
--           busy_n = self.mp_busy()
--           if (busy or busy_n and busy ~= busy_n) and not busy_n then -- decreasing flank detection
--             if self.mp_ready() then
--               print 'Target reached.'
--               rfsm.send_events(fsm, 'e_idle')
--               return
--             end
--             if self.mp_valid() then
--               self.load_new_trajectory(0)
--               return
--             else
--               rtt.logl('Error', 'Motionplanning could not find a trajectory.')
--               rfsm.send_events(fsm, 'e_idle')
--               return
--             end
--           end
--           busy = busy_n
--           rfsm.yield(true)
--         end
--       end,
--     },

--     p2p = rfsm.state{
--       doo = function(fsm)
--         local predict_shift = 0
--         while true do

--           if self.ref_cnt == 0 then
--             self:trigger_motionplanning(predict_shift)
--           end
--           if not self.control_coor:control_hook(true) or self.motionplanning_error() then
--             rfsm.send_events(fsm, 'e_failed')
--           end
--           -- check motion planning
--           if self.mp_ready() then
--             print 'Target reached.'
--             return
--           end
--           -- motion planning check is related to next update
--           self.ref_cnt = self.ref_cnt + 1
--           -- e.g. new trajectories are followed, beginning in the nex update
--           -- This behavior is similar as putting thte motion planning check in
--           -- start of update. But using the current implementation,  the
--           -- control_hook update is synced/clocked better.
--           if not self.mp_busy() and self.ref_cnt >= self.mp_period then
--             if self.mp_valid() then
--               predict_shift = math.max(0, self.ref_cnt - self.mp_period)
--               self.load_new_trajectory(predict_shift)
--               self.ref_cnt = 0
--             else
--               -- try again
--               self.mp_failure_cnt = self.mp_failure_cnt + 1
--               if self.mp_failure_cnt < self.mp_max_failures then
--                 self:trigger_motionplanning(math.max(0, self.ref_cnt - self.mp_period))
--               else
--                 rtt.logl('Error', 'Motionplanning got ' .. self.mp_max_failures .. ' consecutive invalid solutions. Recover...')
--                 rfsm.send_events(fsm, 'e_recover')
--                 return
--               end
--             end
--           end
--           if self.ref_cnt > self.mp_max_periods*self.mp_period then
--             rtt.logl('Error', 'Motionplanning takes too long. Recover...')
--             rfsm.send_events(fsm, 'e_recover')
--             return
--           end
--           if self.ref_cnt >= self.mp_period and math.fmod(self.ref_cnt, self.mp_period) == 0 then
--             rtt.logl('Warning', 'Motionplanning takes longer than '.. math.floor(self.ref_cnt/self.mp_period) ..' period(s)!')
--           end
--           rfsm.yield(true)
--         end
--       end,
--     },

--     recover = rfsm.state{
--       doo = function(fsm)
--         self.mp_recover_cnt = self.mp_recover_cnt + 1
--         if self.mp_recover_cnt >= self.mp_max_recovers then
--           rtt.logl('Error', 'Motionplanning recovered ' .. self.mp_max_recovers .. ' times. Giving up...')
--           return
--         end
--         local time0 = get_sec()
--         local recover_time = 1.
--         while true do
--           if not self.control_coor:control_hook(false) or self.motionplanning_error() then
--             rfsm.send_events(fsm, 'e_failed')
--           end
--           -- wait a little bit and try again
--           if get_sec() - time0 >= recover_time then
--             rfsm.send_events(fsm, 'e_p2p')
--           end
--           rfsm.yield(true)
--         end
--       end,
--     },

--     stop = rfsm.state {
--       entry = function(fsm)
--         self:rm_motionplanner()
--         self.control_coor:stop()
--       end,
--     },
--   }
-- end

function get_sec()
  local sec, nsec = rtt.getTime()
  return sec + nsec*1e-9
end

