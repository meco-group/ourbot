require "rttlib"
require "rfsm"
require "rfsm_rtt"
require "rfsmpp"

local tc=rtt.getTC();
local fsm
local fqn_out, events_in
local start_time

--Create properties
_index            = rtt.Property("int","index","Index number of agent")

tc:addProperty(_index)

--Port to send events
_fsm_event_port   = rtt.InputPort("string")
tc:addEventPort(_fsm_event_port, "fsm_event_port", "rFSM event input port")
--Port from which coordinator can send events to its own
_send_events_port = rtt.OutputPort("string")
tc:addPort(_send_events_port,"send_events","Port to send events to the FSM")
_send_events_port:connect(_fsm_event_port)
--Port to send failure event to deployer
_failure_event_port = rtt.OutputPort("string")
tc:addPort(_failure_event_port,"failure_event_port","Port to send failure event to other components")
--Port to read out the current state.
_current_state_port = rtt.OutputPort("string")
tc:addPort(_current_state_port, "current_state_port", "current active rFSM state")

function configureHook()
   --create local copies of the property values
   index             = _index:get()

   --Define component names here
   estimator     = 'estimator'..index
   controller    = 'controller'..index
   pathgenerator = 'pathgenerator'..index
   reference     = 'reference'..index
   velocitycmd   = 'velocitycmd'..index
   sensors       = 'sensors'..index
   coordinator   = 'coordinator'..index
   reporter      = 'reporter'..index

   -- load state machine
   fsm = rfsm.init(rfsm.load("Coordinator/coordinator_fsm.lua"))
   if not fsm then
      rtt.logl("Error","Could not initialize coordinator state machine")
      return false
   end

   -- enable state entry and exit dbg output
   fsm.dbg=rfsmpp.gen_dbgcolor('Coordinator FSM', { STATE_ENTER=true, STATE_EXIT=true}, false)

   -- connect event ports to state machine
   fsm.getevents = rfsm_rtt.gen_read_str_events(_fsm_event_port)
   rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(_current_state_port))

   -- redirect rFSM output to rtt log
   fsm.info=function(...) rtt.logl('Info', table.concat({...}, ' ')) end
   fsm.warn=function(...) rtt.logl('Warning', table.concat({...}, ' ')) end
   fsm.err=function(...) rtt.logl('Error', table.concat({...}, ' ')) end

   return true
end

function updateHook()
   rfsm.run(fsm)
end

function cleanupHook()
   rttlib.tc_cleanup()
end

--Local function to get the current time in seconds
function get_sec()
   local sec,nsec = rtt.getTime()
   return sec+nsec*1e-9
end
