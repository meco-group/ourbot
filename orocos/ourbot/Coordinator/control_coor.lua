ControlCoordinator = {}
ControlCoordinator.__index = ControlCoordinator

setmetatable(ControlCoordinator, {
    __call = function (cls, ...)
    local self = setmetatable({}, cls)
    self:_init(...)
    return self
  end,
})

function ControlCoordinator._init(self, init)
  self.tc = rtt.getTC()
  self.deployer = self.tc:getPeer('Deployer')
  self.controller = self.tc:getPeer('controller')
  self.estimator = self.tc:getPeer('estimator')
  self.reference = self.tc:getPeer('reference')
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
  self.current_state_port = self.tc:getPort('coordinator_current_state_port')
  self.host = self.tc:getProperty('host'):get()
end

function ControlCoordinator.start(self)
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

function ControlCoordinator.stop(self)
  self.reference:stop()
  self.controller:stop()
end

function ControlCoordinator.control_hook(self, control)
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

function ControlCoordinator.disable_manualcommand(self, fsm)
  self.teensy:strongVelocityControl()
  self.communicator:removeConnection('io', 'cmd_velocity_port', 'cmd_velocity')
end

function ControlCoordinator.enable_manualcommand(self, fsm)
  self.teensy:softVelocityControl()
  if not self.communicator:addIncomingConnection('io', 'cmd_velocity_port', 'cmd_velocity') then
    rfsm.send_events(fsm, 'e_failed')
    return
  end
end

function ControlCoordinator.zero_velocity(self)
  self.teensy:setVelocity(0, 0, 0)
end

function ControlCoordinator.get_fsm(self) -- default fsm
  return rfsm.state{
    entry = function(fsm)
      print('select a state (trajectoryfollowing, motionplanning, flexonomy)')
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
