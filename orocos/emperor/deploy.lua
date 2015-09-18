require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'
require 'os'

tc = rtt.getTC()

--Property definitions
_peers               = rtt.Property("ints","peers","Index numbers of peer agents")
_velocitycmd_type    = rtt.Property("string","velocitycmd","VelocityCommand component to use (only for intern velocitycmd)")
_velcmd_sample_rate  = rtt.Property("double","velcmd_sample_rate","Frequency to update velocity commander")

tc:addProperty(_peers)
tc:addProperty(_velcmd_sample_rate)
tc:addProperty(_velocitycmd_type)

--Ports which drive/read the FSM
_deployer_fsm_event_port      = rtt.InputPort("string")
_deployer_failure_event_port  = rtt.OutputPort("string")
_deployer_current_state_port  = rtt.OutputPort("string")

tc:addEventPort(_deployer_fsm_event_port, "deployer_fsm_event_port", "Event port for driving the deployer FSM")
tc:addPort(_deployer_failure_event_port, "deployer_failure_event_port", "Port to send indicate a failure in the deployer")
tc:addPort(_deployer_current_state_port, "deployer_current_state_port", "current active state of the deployer FSM")

function configureHook()
   --Create local copies of the property values
   peers                = _peers:get()
   velcmd_sample_rate   = _velcmd_sample_rate:get()
   velocitycmd_type     = _velocitycmd_type:get()

   --Create some variables referering to files
   emperor_file      = 'Emperor/emperor.lua'
   app_file          = 'app.ops'

   -- Initialize the deployment state machine, drop out if it fails
   fsm = rfsm.init(rfsm.load('deploy_fsm.lua'))
   if not fsm then
      rtt.logl("Error","Could not initialize deployement state machine")
      return false
   end

   --Add console printing for the state entry and exit
   fsm.dbg = rfsmpp.gen_dbgcolor("Deployer FSM", { STATE_ENTER=true, STATE_EXIT=true}, false)

   -- redirect rFSM output to rtt log
   fsm.info=function(...) rtt.logl('Info', table.concat({...}, ' ')) end
   fsm.warn=function(...) rtt.logl('Warning', table.concat({...}, ' ')) end
   fsm.err=function(...) rtt.logl('Error', table.concat({...}, ' ')) end

   -- connect event ports to state machine
   fsm.getevents = rfsm_rtt.gen_read_str_events(_deployer_fsm_event_port)
   rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(_deployer_current_state_port))

   return true
end

function startHook()
   --create visualization of fsm
   -- require("rfsm2uml")
   -- rfsm2uml.rfsm2uml(fsm,'svg',"deploy_fsm.svg","Deployement FSM")
   return true
end

function updateHook()
  rfsm.run(fsm)
end

function cleanupHook()
   rttlib.tc_cleanup()
end
