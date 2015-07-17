require "rttlib"
require "rfsm"
require "rfsm_rtt"
require "rfsmpp"

local tc=rtt.getTC();
local fsm
local fqn_out, events_in
local start_time

--Create properties
_index = rtt.Property("int","index","Index number of agent")

tc:addProperty(_index)

--Ports which drive/read the FSM
_coordinator_fsm_event_port      = rtt.InputPort("string")
_coordinator_send_event_port     = rtt.OutputPort("string")
_coordinator_failure_event_port  = rtt.OutputPort("string")
_coordinator_current_state_port  = rtt.OutputPort("string")

tc:addEventPort(_coordinator_fsm_event_port, "coordinator_fsm_event_port", "Event port for driving the coordinator FSM")
tc:addPort(_coordinator_send_event_port, "coordinator_send_event_port", "Port to send events to the coordinator FSM from the coordinator")
tc:addPort(_coordinator_failure_event_port,"coordinator_failure_event_port","Port to send indicate a failure in the coordinator")
tc:addPort(_coordinator_current_state_port, "coordinator_current_state_port", "current active state of the coordinator FSM")

_coordinator_send_event_port:connect(_coordinator_fsm_event_port)

function configureHook()
   -- create local copies of the property values
   index = _index:get()

   -- load state machine
   fsm = rfsm.init(rfsm.load("Coordinator/coordinator_fsm.lua"))
   if not fsm then
      rtt.logl("Error","Could not initialize coordinator state machine")
      return false
   end

   -- connect event ports to state machine
   fsm.getevents = rfsm_rtt.gen_read_str_events(_coordinator_fsm_event_port)
   rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(_coordinator_current_state_port))

   -- enable state entry and exit dbg output
   fsm.dbg=rfsmpp.gen_dbgcolor('Coordinator FSM', { STATE_ENTER=true, STATE_EXIT=true}, false)

   -- redirect FSM output to rtt log
   fsm.info=function(...) rtt.logl('Info', table.concat({...}, ' ')) end
   fsm.warn=function(...) rtt.logl('Warning', table.concat({...}, ' ')) end
   fsm.err=function(...) rtt.logl('Error', table.concat({...}, ' ')) end

   -- create ports with timing info
   _controlloop_duration = rtt.OutputPort("double")
   _controlloop_jitter   = rtt.OutputPort("double")
   tc:addPort(_controlloop_duration,"controlloop_duration","Duration of executing the control loop")
   tc:addPort(_controlloop_jitter,"controlloop_jitter","Jitter of the control loop")

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
