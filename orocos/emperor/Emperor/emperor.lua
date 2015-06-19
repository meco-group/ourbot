require "rttlib"
require "rfsm"
require "rfsm_rtt"
require "rfsmpp"

local tc=rtt.getTC();
local fsm
local fqn_out, events_in
local start_time

--Create properties
_peers = rtt.Property("ints","peers","Index numbers of peer agents")

tc:addProperty(_peers)

--Port to send events to coordinator peers
_fsm_event_port   = rtt.OutputPort("string")
tc:addPort(_fsm_event_port, "fsm_event_port", "rFSM event input port")
-- --Port to send events
_emperor_fsm_event_port = rtt.InputPort("string")
tc:addEventPort(_emperor_fsm_event_port, "emperor_fsm_event_port", "rFSM event input port")
--Port from which emperor can send events to its own
_send_events_port = rtt.OutputPort("string")
tc:addPort(_send_events_port,"send_events","Port to send events to the FSM")
_send_events_port:connect(_emperor_fsm_event_port)
--Port to send failure event to deployer
_failure_event_port = rtt.OutputPort("string")
tc:addPort(_failure_event_port,"failure_event_port","Port to send failure event to other components")
--Port where current state of peers come
_current_state_port = rtt.InputPort("string")
tc:addEventPort(_current_state_port, "current_state_port", "current active rFSM state")

function configureHook()
   --create local copies of the property values
   peers         = _peers:get()

   -- load state machine
   fsm = rfsm.init(rfsm.load("Emperor/emperor_fsm.lua"))
   if not fsm then
      rtt.logl("Error","Could not initialize emperor state machine")
      return false
   end

   -- enable state entry and exit dbg output
   fsm.dbg=rfsmpp.gen_dbgcolor('Emperor FSM', { STATE_ENTER=true, STATE_EXIT=true}, false)

   -- connect event ports to state machine
   fsm.getevents = rfsm_rtt.gen_read_str_events(_emperor_fsm_event_port)
   -- rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(_current_state_port))

   -- redirect rFSM output to rtt log
   fsm.info=function(...) rtt.logl('Info', table.concat({...}, ' ')) end
   fsm.warn=function(...) rtt.logl('Warning', table.concat({...}, ' ')) end
   fsm.err=function(...) rtt.logl('Error', table.concat({...}, ' ')) end

   return true
end

function updateHook()
   rfsm.run(fsm)
   -- _current_state_port:read(st)
end

function cleanupHook()
   rttlib.tc_cleanup()
end

--Local function to get the current time in seconds
function get_sec()
   local sec,nsec = rtt.getTime()
   return sec+nsec*1e-9
end
