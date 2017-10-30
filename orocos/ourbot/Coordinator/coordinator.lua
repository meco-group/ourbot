require 'rttlib'
require 'rfsm'
require 'rfsm_rtt'
require 'rfsmpp'

require './Coordinator/globals'

local tc = rtt.getTC();
local fsm
local fqn_out, events_in

-- create properties
tc:addProperty(rtt.Property('int', 'print_level','Level of output printing'))
tc:addProperty(rtt.Property('double', 'control_rate', 'Frequency to update controller'))
tc:addProperty(rtt.Property('double', 'motionplanning_rate', 'Frequency to update motion planning'))
tc:addProperty(rtt.Property('double', 'reporting_rate', 'Frequency to take snapshots for the reporter'))
tc:addProperty(rtt.Property('bool', 'obstacle_mode','Robot is acting as a moving obstacle'))
tc:addProperty(rtt.Property('string','host', 'Name of host'))
tc:addProperty(rtt.Property('array', 'ourbot_size'))

-- create/connect ports
local coordinator_fsm_event_port = rtt.InputPort('string')
local coordinator_send_event_port = rtt.OutputPort('string')
local coordinator_failure_event_port = rtt.OutputPort('string')
local coordinator_current_state_port = rtt.OutputPort('string')

tc:addEventPort(coordinator_fsm_event_port, 'coordinator_fsm_event_port', 'Event port for driving the coordinator FSM')
tc:addPort(coordinator_send_event_port, 'coordinator_send_event_port', 'Port to send events to the coordinator FSM from the coordinator')
tc:addPort(coordinator_failure_event_port, 'coordinator_failure_event_port','Port to send indicate a failure in the coordinator')
tc:addPort(coordinator_current_state_port, 'coordinator_current_state_port', 'current active state of the coordinator FSM')
tc:addPort(rtt.InputPort('array'), 'est_pose_port', 'Estimated pose wrt to initial frame')
coordinator_send_event_port:connect(coordinator_fsm_event_port)

local fsm

function configureHook()
  -- init global variables and functions
  init_globals()

  -- obstacle_mode = _obstacle_mode:get()
  -- if obstacle_mode then
  --   communicator:joinGroup('obstacle')
  -- end

  -- load state machine
  fsm = rfsm.init(rfsm.load('Coordinator/coordinator_fsm.lua'))
  if not fsm then
    rtt.logl('Error','Could not initialize coordinator state machine')
    return false
  end

  -- connect event ports to state machine
  fsm.getevents = rfsm_rtt.gen_read_str_events(coordinator_fsm_event_port)
  rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(coordinator_current_state_port))

  if tc:getProperty('print_level'):get() >= 2 then
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
