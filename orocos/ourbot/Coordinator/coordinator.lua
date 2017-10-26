require 'rttlib'
require 'rfsm'
require 'rfsm_rtt'
require 'rfsmpp'

local tc = rtt.getTC();
local fsm
local fqn_out, events_in

-- create properties
local _print_level = rtt.Property('int', 'print_level','Level of output printing')
local _control_rate = rtt.Property('double', 'control_rate', 'Frequency to update controller')
local _motionplanning_rate = rtt.Property('double', 'motionplanning_rate', 'Frequency to update motion planning')
local _reporting_rate = rtt.Property('double', 'reporting_rate', 'Frequency to take snapshots for the reporter')
local _obstacle_mode = rtt.Property('bool', 'obstacle_mode','Robot is acting as a moving obstacle')
local _host = rtt.Property('string','host', 'Name of host')

tc:addProperty(_print_level)
tc:addProperty(_control_rate)
tc:addProperty(_motionplanning_rate)
tc:addProperty(_reporting_rate)
tc:addProperty(_obstacle_mode)
tc:addProperty(_host)

-- create/connect ports
local _coordinator_fsm_event_port = rtt.InputPort('string')
local _coordinator_send_event_port = rtt.OutputPort('string')
local _coordinator_failure_event_port = rtt.OutputPort('string')
local _coordinator_current_state_port = rtt.OutputPort('string')
local _controlloop_duration_port = rtt.OutputPort('double')
local _controlloop_jitter_port = rtt.OutputPort('double')
local _est_pose_port = rtt.InputPort('array')

tc:addEventPort(_coordinator_fsm_event_port, 'coordinator_fsm_event_port', 'Event port for driving the coordinator FSM')
tc:addPort(_coordinator_send_event_port, 'coordinator_send_event_port', 'Port to send events to the coordinator FSM from the coordinator')
tc:addPort(_coordinator_failure_event_port,'coordinator_failure_event_port','Port to send indicate a failure in the coordinator')
tc:addPort(_coordinator_current_state_port, 'coordinator_current_state_port', 'current active state of the coordinator FSM')
tc:addPort(_controlloop_duration_port, 'controlloop_duration_port','Duration of executing the control loop')
tc:addPort(_controlloop_jitter_port, 'controlloop_jitter_port','Jitter of the control loop')
tc:addPort(_est_pose_port, 'est_pose_port', 'Estimated pose wrt to initial frame')

_coordinator_send_event_port:connect(_coordinator_fsm_event_port)

-- variables to use in updateHook
local communicator
local io
local estimator
local reference
local controller
local reporter
local teensy

local communicator_update
local communicator_error
local io_error
local estimator_error
local reference_error
local controller_error
local reporter_snapshot
local period
local reporting_rate
local snapshot_cnt
local start_time
local prev_start_time
local end_time

local fsm

function configureHook()
  -- create local copies of the property values
  -- obstacle_mode = _obstacle_mode:get()
  -- if obstacle_mode then
  --   communicator:joinGroup('obstacle')
  -- end

  communicator = tc:getPeer('communicator')
  io = tc:getPeer('io')
  estimator = tc:getPeer('estimator')
  reference = tc:getPeer('reference')
  controller = tc:getPeer('controller')
  reporter = tc:getPeer('reporter')
  teensy = tc:getPeer('teensy')

  communicator_update = communicator:getOperation('update')
  communicator_error = communicator:getOperation('inRunTimeError')
  io_error = io:getOperation('inRunTimeError')
  estimator_error = estimator:getOperation('inRunTimeError')
  reference_error = reference:getOperation('inRunTimeError')
  controller_error = controller:getOperation('inRunTimeError')
  reporter_snapshot = reporter:getOperation('snapshot')
  period = tc:getPeriod()
  reporting_rate = _reporting_rate:get()
  snapshot_cnt = 1./(reporting_rate*period)

  -- load state machine
  fsm = rfsm.init(rfsm.load('Coordinator/coordinator_fsm.lua'))
  if not fsm then
    rtt.logl('Error','Could not initialize coordinator state machine')
    return false
  end

  -- connect event ports to state machine
  fsm.getevents = rfsm_rtt.gen_read_str_events(_coordinator_fsm_event_port)
  rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(_coordinator_current_state_port))

  if _print_level:get() >= 2 then
    -- enable state entry and exit dbg output
    fsm.dbg=rfsmpp.gen_dbgcolor('Coordinator FSM', { STATE_ENTER=true, STATE_EXIT=true}, false)
    -- redirect FSM output to rtt log
    fsm.info=function(...) rtt.logl('Info', table.concat({...}, ' ')) end
    fsm.warn=function(...) rtt.logl('Warning', table.concat({...}, ' ')) end
    fsm.err=function(...) rtt.logl('Error', table.concat({...}, ' ')) end
  end
  return true
end

function startHook()
  if not start_sensing_components() then
    return false
  end
  start_time = get_sec()
  prev_start_time = start_time
  end_time = start_time
  return true
end

function updateHook()
  prev_start_time = start_time
  start_time = get_sec()
  -- update communication
  communicator_update()
  -- snapshot for reporter
  snapshot()
  -- error check
  if io_error() then
    rtt.logl('Error', 'RunTimeError in gamepad component!')
    rfsm.send_events(fsm, 'e_failed')
    return
  end
  if estimator_error() then
    rtt.logl('Error', 'RunTimeError in hawkeye component!')
    rfsm.send_events(fsm, 'e_failed')
    return
  end
  if reference_error() then
    rtt.logl('Error', 'RunTimeError in emperor component!')
    rfsm.send_events(fsm, 'e_failed')
    return
  end
  if controller_error() then
    rtt.logl('Error', 'RunTimeError in emperor component!')
    rfsm.send_events(fsm, 'e_failed')
    return
  end
  if communicator_error() then
    rtt.logl('Error','RunTimeError in communicator component!')
    rfsm.send_events(fsm, 'e_failed')
    return
  end
  if tc:inRunTimeError() then
    rtt.logl('Error', 'RunTimeError in coordinator component!')
    return false
  end
  -- run state machine
  rfsm.run(fsm)
  -- guard time
  guard_time(start_time, prev_start_time, end_time)
  end_time = get_sec()
end

function stopHook()
  stop_sensing_components()
end

function cleanupHook()
   rttlib.tc_cleanup()
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
  if duration > 900*period then
    rtt.logl('Warning','ControlLoop: Duration of calculation exceeded 90% of sample period!')
  end
  if jitter > 100*period then
    rtt.logl('Warning','ControlLoop: Jitter exceeded 10% of sample period')
  end
  _controlloop_duration_port:write(duration)
  _controlloop_jitter_port:write(jitter)
end
