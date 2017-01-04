require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'
require 'os'

tc = rtt.getTC()

-- property definitions
_print_level         = rtt.Property("int","print_level","Level of output printing")
_estimator_type      = rtt.Property("string","estimator","Estimator component to use")
_controller_type     = rtt.Property("string","controller","Controller component to use")
_motionplanning_type = rtt.Property("string","motionplanning","MotionPlanning component to use")
_distributed_mp      = rtt.Property("bool","distributed_mp","Distributed motion planning?")
_control_sample_rate = rtt.Property("double","control_sample_rate","Frequency to update the control loop")
_mp_sample_rate      = rtt.Property("double","mp_sample_rate","Frequency to update the motion planning")
_reporter_sample_rate= rtt.Property("double","reporter_sample_rate", "Frequency to take snapshots for the reporter")
_io_sample_rate      = rtt.Property("double","io_sample_rate","Frequency to update io's")
_obstacle_mode       = rtt.Property("bool","obstacle_mode","Robot is acting as a moving obstacle")
_host                = rtt.Property("string","host","Name of host")

tc:addProperty(_print_level)
tc:addProperty(_estimator_type)
tc:addProperty(_controller_type)
tc:addProperty(_motionplanning_type)
tc:addProperty(_distributed_mp)
tc:addProperty(_control_sample_rate)
tc:addProperty(_mp_sample_rate)
tc:addProperty(_reporter_sample_rate)
tc:addProperty(_io_sample_rate)
tc:addProperty(_obstacle_mode)
tc:addProperty(_host)

-- ports that drive/read the FSM
_deployer_fsm_event_port      = rtt.InputPort("string")
_deployer_failure_event_port  = rtt.OutputPort("string")
_deployer_current_state_port  = rtt.OutputPort("string")

tc:addEventPort(_deployer_fsm_event_port, "deployer_fsm_event_port", "Event port for driving the deployer FSM")
tc:addPort(_deployer_failure_event_port, "deployer_failure_event_port", "Port to indicate a failure in the deployer")
tc:addPort(_deployer_current_state_port, "deployer_current_state_port", "Current active state of the deployer FSM")

function configureHook()
   -- create local copies of the property values
   print_level          = _print_level:get()
   estimator_type       = _estimator_type:get()
   controller_type      = _controller_type:get()
   motionplanning_type  = _motionplanning_type:get()
   distributed_mp       = _distributed_mp:get()
   control_sample_rate  = _control_sample_rate:get()
   mp_sample_rate       = _mp_sample_rate:get()
   reporter_sample_rate = _reporter_sample_rate:get()
   io_sample_rate       = _io_sample_rate:get()
   obstacle_mode        = _obstacle_mode:get()
   host                 = _host:get()

   -- create some variables referering to files
   coordinator_file  = 'Coordinator/coordinator.lua'
   app_file          = 'app.ops'

   -- initialize the deployment state machine, drop out if it fails
   fsm = rfsm.init(rfsm.load('deploy_fsm.lua'))
   if not fsm then
      rtt.logl("Error","Could not initialize deployement state machine")
      return false
   end

   if print_level >= 2 then
      -- add console printing for the state entry and exit
      fsm.dbg = rfsmpp.gen_dbgcolor("Deployer FSM", { STATE_ENTER=true, STATE_EXIT=true}, false)
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
