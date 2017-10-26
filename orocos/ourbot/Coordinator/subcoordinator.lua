SubCoordinator = {}
SubCoordinator.__index = SubCoordinator

setmetatable(SubCoordinator, {
    __call = function (cls, ...)
    local self = setmetatable({}, cls)
    self:_init(...)
    return self
  end,
})

function SubCoordinator._init(self, init)
  self.tc = rtt.getTC()
  self.deployer = self.tc:getPeer('Deployer')
  self.controller = self.tc:getPeer('controller')
  self.estimator = self.tc:getPeer('estimator')
  self.reference = self.tc:getPeer('reference')
  self.reporter = self.tc:getPeer('reporter')
  self.io = self.tc:getPeer('io')
  self.teensy = self.tc:getPeer('teensy')
  self.communicator = self.tc:getPeer('communicator')
  self.period = self.tc:getPeriod()
  self.io_update = self.io:getOperation('update')
  self.estimator_update = self.estimator:getOperation('update')
  self.reference_update = self.reference:getOperation('update')
  self.controller_update = self.controller:getOperation('update')
  self.estimator_valid = self.estimator:getOperation('valid')
  self.set_velocity = self.teensy:getOperation('setVelocity')
  self.control_rate = self.tc:getProperty('control_rate'):get()
  self.est_pose_port = self.tc:getPort('est_pose_port')
  self.coordinator_send_event_port = self.tc:getPort('coordinator_send_event_port')
end

function SubCoordinator.start_control_components(self)
  if not self.reference:start() then
    rtt.logl('Error', 'Could not start reference component!')
    return false
  end
  if not self.controller:start() then
    rtt.logl('Error', 'Could not start controller component!')
    return false
  end
  return true
end

function SubCoordinator.stop_control_components(self)
  self.reference:stop()
  self.controller:stop()
end

function SubCoordinator.control_hook(self, control)
  self.io_update()
  self.estimator_update()
  if control then
    if true or self.estimator_valid() then
      self.reference_update()
      self.controller_update()
    else
      rtt.logl('Warning', 'Estimate not valid!')
    end
  end
  return true
end

function SubCoordinator.disable_manualcommand(self, fsm)
  self.teensy:strongVelocityControl()
  self.communicator:removeConnection('io', 'cmd_velocity_port', 'cmd_velocity')
end

function SubCoordinator.enable_manualcommand(self, fsm)
  self.teensy:softVelocityControl()
  if not self.communicator:addIncomingConnection('io', 'cmd_velocity_port', 'cmd_velocity') then
    rfsm.send_events(fsm, 'e_failed')
    return
  end
end

function SubCoordinator.get_fsm(self) -- default fsm
  return rfsm.state{
    entry = function(fsm)
      print('select a state (trajectoryfollowing, motionplanning, formation)')
      self.coordinator_send_event_port:write('e_idle') -- to inform emperor
      self.teensy:setVelocity(0, 0, 0)
      self:enable_manualcommand()
    end,

    doo = function(fsm)
      while true do
        if not self:control_hook(false) then
          rfsm.send_events(fsm, 'e_failed')
        end
        rfsm.yield(true)
      end
    end,
  }
end
