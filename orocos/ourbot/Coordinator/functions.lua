Functions = {}

local tc = rtt.getTC()
local dp = tc:getPeer('Deployer')

local period = tc:getPeriod()

local controller = tc:getPeer('controller')
local estimator = tc:getPeer('estimator')
local reference = tc:getPeer('reference')
local reporter = tc:getPeer('reporter')
local io = tc:getPeer('io')
local teensy = tc:getPeer('teensy')
local communicator = tc:getPeer('communicator')

local io_update = io:getOperation('update')
local estimator_update = estimator:getOperation('update')
local reference_update = reference:getOperation('update')
local controller_update = controller:getOperation('update')
local io_error = io:getOperation('inRunTimeError')
local estimator_error = estimator:getOperation('inRunTimeError')
local reference_error = reference:getOperation('inRunTimeError')
local controller_error = controller:getOperation('inRunTimeError')
estimator_valid = estimator:getOperation('valid')


-- add ports
local _controlloop_duration_port = rtt.OutputPort('double')
local _controlloop_jitter_port = rtt.OutputPort('double')

tc:addPort(_controlloop_duration_port, 'controlloop_duration_port','Duration of executing the control loop')
tc:addPort(_controlloop_jitter_port, 'controlloop_jitter_port','Jitter of the control loop')

function Functions:start_sensing_components()
  if not io:start() then
    rtt.logl('Error', 'Could not start io component!')
    -- return false
  end
  if not estimator:start() then
    rtt.logl('Error', 'Could not start estimator component!')
    return false
  end
  if not reporter:start() then
    rtt.logl('Error', 'Could not start reporter component!')
    return false
  end
  return true
end

function Functions:stop_sensing_components()
  estimator:stop()
  reporter:stop()
  io:stop()
end

function Functions:start_control_components()
  if not reference:start() then
    rtt.logl('Error', 'Could not start reference component!')
    return false
  end
  if not controller:start() then
    rtt.logl('Error', 'Could not start controller component!')
    return false
  end
  return true
end

function Functions:stop_control_components()
  reference:stop()
  controller:stop()
end

function Functions:control_hook(control)
  io_update()
  estimator_update()
  if control then
    if true or estimator_valid() then
      reference_update()
      controller_update()
    else
      rtt.logl('Warning', 'Estimate not valid!')
    end
  end
  if tc:inRunTimeError() then
    rtt.logl('Error', 'RunTimeError in coordinator component!')
    return false
  end
  if io_error() then
    rtt.logl('Error', 'RunTimeError in io component!')
    return false
  end
  if estimator_error() then
    rtt.logl('Error', 'RunTimeError in estimator component!')
    return false
  end
  if reference_error() then
    rtt.logl('Error', 'RunTimeError in reference component!')
    return false
  end
  if controller_error() then
    rtt.logl('Error', 'RunTimeError in controller component!')
    return false
  end
  return true
end

function Functions:disable_manualcommand(fsm)
  teensy:strongVelocityControl()
  communicator:removeConnection('io', 'cmd_velocity_port', 'cmd_velocity')
end

function Functions:enable_manualcommand(fsm)
  teensy:softVelocityControl()
  if not communicator:addIncomingConnection('io', 'cmd_velocity_port', 'cmd_velocity') then rfsm.send_events(fsm, 'e_failed') return end
end


function Functions:get_sec()
  local sec, nsec = rtt.getTime()
  return sec + nsec*1e-9
end

function Functions:guard_time(start_time, prev_start_time, end_time)
  local duration = (end_time - prev_start_time) * 1000
  local jitter = (start_time - prev_start_time - period) * 1000
  if duration > 900*period then
    rtt.logl('Warning','ControlLoop: Duration of calculation exceeded 90% of sample period!')
  end
  if jitter > 100*period then
    rtt.logl('Warning','ControlLoop: Jitter exceeded 10% of sample period')
  end
  _controlloop_duration_port:write(duration)
  _controlloop_jitter_port:write(jitter)
end

return Functions
