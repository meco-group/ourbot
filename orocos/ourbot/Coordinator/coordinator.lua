require 'rttlib'
require 'rfsm'
require 'rfsm_rtt'
require 'rfsmpp'

local tc = rtt.getTC();
local fsm
local fqn_out, events_in

-- create properties
_print_level = rtt.Property('int', 'print_level','Level of output printing')
_control_rate = rtt.Property('double', 'control_rate', 'Frequency to update controller')
_motionplanning_rate = rtt.Property('double', 'motionplanning_rate', 'Frequency to update motion planning')
_reporting_rate = rtt.Property('double', 'reporting_rate', 'Frequency to take snapshots for the reporter')
_obstacle_mode = rtt.Property('bool', 'obstacle_mode','Robot is acting as a moving obstacle')
_host = rtt.Property('string','host', 'Name of host')

tc:addProperty(_print_level)
tc:addProperty(_control_rate)
tc:addProperty(_motionplanning_rate)
tc:addProperty(_reporting_rate)
tc:addProperty(_obstacle_mode)
tc:addProperty(_host)

-- ports that drive/read the FSM
_coordinator_fsm_event_port = rtt.InputPort('string')
_coordinator_send_event_port = rtt.OutputPort('string')
_coordinator_failure_event_port = rtt.OutputPort('string')
_coordinator_current_state_port = rtt.OutputPort('string')

tc:addEventPort(_coordinator_fsm_event_port, 'coordinator_fsm_event_port', 'Event port for driving the coordinator FSM')
tc:addPort(_coordinator_send_event_port, 'coordinator_send_event_port', 'Port to send events to the coordinator FSM from the coordinator')
tc:addPort(_coordinator_failure_event_port,'coordinator_failure_event_port','Port to send indicate a failure in the coordinator')
tc:addPort(_coordinator_current_state_port, 'coordinator_current_state_port', 'current active state of the coordinator FSM')

_coordinator_send_event_port:connect(_coordinator_fsm_event_port)

function configureHook()
   fun = require './Coordinator/functions'
   dp = tc:getPeer('Deployer')

   -- create local copies of the property values
   print_level = _print_level:get()
   control_rate = _control_rate:get()
   motionplanning_rate = _motionplanning_rate:get()
   reporting_rate = _reporting_rate:get()
   obstacle_mode = _obstacle_mode:get()
   host = _host:get()

   -- variables to use in updateHook
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
   snapshot_cnt = 1./(reporting_rate*period)

   if obstacle_mode then
      communicator:joinGroup('obstacle')
   end

   -- load state machine
   fsm = rfsm.init(rfsm.load('Coordinator/coordinator_fsm.lua'))
   if not fsm then
      rtt.logl('Error','Could not initialize coordinator state machine')
      return false
   end

   -- connect event ports to state machine
   fsm.getevents = rfsm_rtt.gen_read_str_events(_coordinator_fsm_event_port)
   rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(_coordinator_current_state_port))

   if print_level >= 2 then
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
   return fun:start_sensing_components()
end

function updateHook()
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
end

function stopHook()
   fun:stop_sensing_components()
end

function cleanupHook()
   rttlib.tc_cleanup()
end

function snapshot()
  if snapshot_cnt >= 1./(reporting_rate*period) then
      reporter_snapshot:send()
      snapshot_cnt = 1
  else
      snapshot_cnt = snapshot_cnt + 1
  end
end

