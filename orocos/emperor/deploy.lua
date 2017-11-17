require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'
require 'os'

tc = rtt.getTC()

-- property definitions
_print_level  = rtt.Property('int', 'print_level', 'Level of output printing')
_velcmd_rate = rtt.Property('double', 'velcmd_rate', 'Frequency to update velocity commander')
_emperor_rate = rtt.Property('double', 'emperor_rate', 'Frequency to update communicator')
_camera_rate = rtt.Property('double', 'camera_rate', 'Frequency to update camera')

tc:addProperty(_print_level)
tc:addProperty(_velcmd_rate)
tc:addProperty(_emperor_rate)
tc:addProperty(_camera_rate)

-- ports that drive/read the FSM
_deployer_fsm_event_port = rtt.InputPort('string')
_deployer_failure_event_port = rtt.OutputPort('string')
_deployer_current_state_port = rtt.OutputPort('string')

tc:addEventPort(_deployer_fsm_event_port, 'deployer_fsm_event_port', 'Event port for driving the deployer FSM')
tc:addPort(_deployer_failure_event_port, 'deployer_failure_event_port', 'Port to send indicate a failure in the deployer')
tc:addPort(_deployer_current_state_port, 'deployer_current_state_port', 'current active state of the deployer FSM')

function configureHook()
   -- create local copies of the property values
   print_level = _print_level:get()
   velcmd_rate = _velcmd_rate:get()
   emperor_rate = _emperor_rate:get()
   camera_rate = _camera_rate:get()

   -- create some variables referering to files
   emperor_file = 'Emperor/emperor.lua'
   app_file = 'app.ops'

   -- initialize the deployment state machine, drop out if it fails
   fsm = rfsm.init(rfsm.load('deploy_fsm.lua'))
   if not fsm then
      rtt.logl('Error','Could not initialize deployement state machine')
      return false
   end

   if print_level >= 2 then
      -- add console printing for the state entry and exit
      fsm.dbg = rfsmpp.gen_dbgcolor('Deployer FSM', { STATE_ENTER=true, STATE_EXIT=true}, false)
      -- redirect rFSM output to rtt log
      fsm.info=function(...) rtt.logl('Info', table.concat({...}, ' ')) end
      fsm.warn=function(...) rtt.logl('Warning', table.concat({...}, ' ')) end
      fsm.err=function(...) rtt.logl('Error', table.concat({...}, ' ')) end
   end

   -- connect event ports to state machine
   fsm.getevents = rfsm_rtt.gen_read_str_events(_deployer_fsm_event_port)
   rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(_deployer_current_state_port))
   return true
end

function updateHook()
   rfsm.run(fsm)
end

function cleanupHook()
   rttlib.tc_cleanup()
end
