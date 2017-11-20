
-- global functions
function init_globals()
  -- components
  tc = rtt.getTC()
  deployer = tc:getPeer('Deployer')
  controller = tc:getPeer('controller')
  estimator = tc:getPeer('estimator')
  reference = tc:getPeer('reference')
  io = tc:getPeer('io')
  teensy = tc:getPeer('teensy')
  eaglebridge = tc:getPeer('eaglebridge')
  communicator = tc:getPeer('communicator')
  reporter = tc:getPeer('reporter')
  -- operations
  communicator_update = communicator:getOperation('update')
  io_update = io:getOperation('update')
  estimator_update = estimator:getOperation('update')
  controller_update = controller:getOperation('update')
  reference_update = reference:getOperation('update')
  reporter_snapshot = reporter:getOperation('snapshot')
  communicator_error = communicator:getOperation('inRunTimeError')
  io_error = io:getOperation('inRunTimeError')
  estimator_error = estimator:getOperation('inRunTimeError')
  controller_error = controller:getOperation('inRunTimeError')
  reference_error = reference:getOperation('inRunTimeError')
  estimator_valid = estimator:getOperation('valid')
  set_velocity = teensy:getOperation('setVelocity')
  -- ports
  est_pose_port = tc:getPort('est_pose_port')
  coordinator_send_event_port = tc:getPort('coordinator_send_event_port')
  current_state_port = tc:getPort('coordinator_current_state_port')
  host = tc:getProperty('host'):get()
  -- properties and variables
  period = tc:getPeriod()
  control_rate = tc:getProperty('control_rate'):get()
  motionplanning_rate = tc:getProperty('motionplanning_rate'):get()
  reporting_rate = tc:getProperty('reporting_rate'):get()
  garbagecollect_rate = tc:getProperty('garbagecollect_rate'):get()
  snapshot_cnt = 1./(reporting_rate*period)
  garbagecollect_cnt = 1./(garbagecollect_rate*period)
end

function start_control_components()
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

function stop_control_components()
  reference:stop()
  controller:stop()
end

function start_sensing_components()
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

function stop_sensing_components()
  estimator:stop()
  reporter:stop()
  io:stop()
end

function control_hook(control)
  estimator_update()
  if control then
    if estimator_valid() then
      reference_update()
      controller_update()
    else
      rtt.logl('Warning', 'Estimate not valid!')
    end
  end
  io_update()
  return true
end

function disable_manualcommand(fsm)
  teensy:strongVelocityControl()
  communicator:removeConnection('io', 'cmd_velocity_port', 'cmd_velocity')
end

function enable_manualcommand(fsm)
  teensy:softVelocityControl()
  if not communicator:addIncomingConnection('io', 'cmd_velocity_port', 'cmd_velocity') then
    rfsm.send_events(fsm, 'e_failed')
    return
  end
end

function zero_velocity(self)
  teensy:setVelocity(0, 0, 0)
end

function garbage_collect()
  if garbagecollect_cnt >= 1./(garbagecollect_rate*period) then
    -- print(collectgarbage("count")*1024)
    collectgarbage("collect")
    garbagecollect_cnt = 1
  else
    garbagecollect_cnt = garbagecollect_cnt + 1
  end
end

function snapshot()
  if snapshot_cnt >= 1./(reporting_rate*period) then
      reporter_snapshot:send()
      snapshot_cnt = 1
  else
      snapshot_cnt = snapshot_cnt + 1
  end
end

function get_sec()
  local sec, nsec = rtt.getTime()
  return sec + nsec*1e-9
end

function guard_time(start_time, prev_start_time, end_time)
  local duration = (end_time - prev_start_time) * 1000
  local jitter = (start_time - prev_start_time - period) * 1000
  -- print('duration: ' .. duration .. ' ms')
  -- if duration > 1000*period then
  --   rtt.logl('Warning','ControlLoop: Duration of calculation exceeded sample period!')
  -- end
  -- if jitter > 100*period then
  --   rtt.logl('Info','ControlLoop: Jitter exceeded 10% of sample period')
  -- end
end

