require "rttlib"
require "rfsm"
require "rfsm_rtt"
require "rfsmpp"

local tc=rtt.getTC();
local fsm
local fqn_out, events_in

function configureHook()
   -- load state machine
   fsm = rfsm.init(rfsm.load("Coordinator/fsm.lua"))

   -- enable state entry and exit dbg output
   -- fsm.dbg=rfsmpp.gen_dbgcolor("rfsm-rtt-example",
   --                 { STATE_ENTER=true, STATE_EXIT=true},
   --                 false)

   -- redirect rFSM output to rtt log
   fsm.info=function(...) rtt.logl('Info', table.concat({...}, ' ')) end
   fsm.warn=function(...) rtt.logl('Warning', table.concat({...}, ' ')) end
   fsm.err=function(...) rtt.logl('Error', table.concat({...}, ' ')) end

   -- the following creates a string input port, adds it as a event
   -- driven port to the Taskcontext. The third line generates a
   -- getevents function which returns all data on the current port as
   -- events. This function is called by the rFSM core to check for
   -- new events.
   _fsm_event_inport = rtt.InputPort("string")
   tc:addEventPort(_fsm_event_inport, "fsm_event_inport", "rFSM event input port")
   fsm.getevents = rfsm_rtt.gen_read_str_events(_fsm_event_inport)

   -- optional: create a string port to which the currently active
   -- state of the FSM will be written. gen_write_fqn generates a
   -- function suitable to be added to the rFSM step hook to do this.
   _current_state_outport = rtt.OutputPort("string")
   tc:addPort(_current_state_outport, "current_state_outport", "current active rFSM state")
   rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(_current_state_outport))

   return true
end

function updateHook()
   rfsm.run(fsm)
end

function cleanupHook()
   -- cleanup the created ports.
   rttlib.tc_cleanup()
end
