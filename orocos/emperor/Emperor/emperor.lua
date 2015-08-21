require "rttlib"
require "rfsm"
require "rfsm_rtt"
require "rfsmpp"

local tc=rtt.getTC();
local fsm
local fqn_out, events_in
local start_time

--Create properties
_print_level   = rtt.Property("int","print_level","Level of output printing")
_peers         = rtt.Property("ints","peers","Index numbers of peer agents")

tc:addProperty(_print_level)
tc:addProperty(_peers)

--Ports which drive/read the FSM
_emperor_fsm_event_port      = rtt.InputPort("string")
_emperor_send_event_port     = rtt.OutputPort("string")
_emperor_failure_event_port  = rtt.OutputPort("string")
_emperor_current_state_port  = rtt.OutputPort("string")

tc:addEventPort(_emperor_fsm_event_port, "emperor_fsm_event_port", "Event port for driving the emperor FSM")
tc:addPort(_emperor_send_event_port, "emperor_send_event_port", "Port to send events to the emperor FSM from the emperor")
tc:addPort(_emperor_failure_event_port,"emperor_failure_event_port","Port to send indicate a failure in the emperor")
tc:addPort(_emperor_current_state_port, "emperor_current_state_port", "current active state of the emperor FSM")

_emperor_send_event_port:connect(_emperor_fsm_event_port)

function configureHook()
   --create local copies of the property values
   print_level = _print_level:get()
   peers       = _peers:get()

   -- load state machine
   fsm = rfsm.init(rfsm.load("Emperor/emperor_fsm.lua"))
   if not fsm then
      rtt.logl("Error","Could not initialize emperor state machine")
      return false
   end

  -- connect event ports to state machine
   fsm.getevents = rfsm_rtt.gen_read_str_events(_emperor_fsm_event_port)
   rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(_emperor_current_state_port))

   if print_level >= 2 then
      -- enable state entry and exit dbg output
      fsm.dbg=rfsmpp.gen_dbgcolor('Emperor FSM', { STATE_ENTER=true, STATE_EXIT=true}, false)

      -- redirect rFSM output to rtt log
      fsm.info=function(...) rtt.logl('Info', table.concat({...}, ' ')) end
      fsm.warn=function(...) rtt.logl('Warning', table.concat({...}, ' ')) end
      fsm.err=function(...) rtt.logl('Error', table.concat({...}, ' ')) end
   end

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
