require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'
require 'os'

tc = rtt.getTC()

--Property definitions
_peers               = rtt.Property("ints","peers","Index numbers of peer agents")
_velcmd_sample_rate  = rtt.Property("double","velcmd_sample_rate","Frequency to update velocity commander")
_velocitycmd_type    = rtt.Property("string","velocitycmd","VelocityCommand component to use (only for intern velocitycmd)")

tc:addProperty(_peers)
tc:addProperty(_velcmd_sample_rate)
tc:addProperty(_velocitycmd_type)

--Port to send events
_fsm_event_port   = rtt.InputPort("string")
tc:addEventPort(_fsm_event_port, "fsm_event_port", "rFSM event input port")
--Port to read out the current state.
_current_state_port = rtt.OutputPort("string")
tc:addPort(_current_state_port, "current_state_port", "current active rFSM state")

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

   -- redirect rFSM output to rtt log
   fsm.info=function(...) rtt.logl('Info', table.concat({...}, ' ')) end
   fsm.warn=function(...) rtt.logl('Warning', table.concat({...}, ' ')) end
   fsm.err=function(...) rtt.logl('Error', table.concat({...}, ' ')) end

   --Add console printing for the state entry and exit
   fsm.dbg = rfsmpp.gen_dbgcolor("Deployer FSM", { STATE_ENTER=true, STATE_EXIT=true}, false)

   -- connect event ports to state machine
   fsm.getevents = rfsm_rtt.gen_read_str_events(_fsm_event_port)
   rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(_current_state_port))

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
