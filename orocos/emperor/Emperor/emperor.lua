require "rttlib"
require "rfsm"
require "rfsm_rtt"
require "rfsmpp"

local tc=rtt.getTC();
local fsm
local fqn_out, events_in
local start_time
state = ''
main_state = ''
local menu_options         = {'VelocityControl','MotionPlanning', 'TrajectoryFollowing'}
local main_states          = {'velocitycmd', 'motionplanning', 'trajectoryfollowing'}
local sub_states           = {'idle', 'init', 'run', 'stop'}
local menu_option_ind      = 1

--Create properties
_print_level            = rtt.Property("int","print_level","Level of output printing")
_reporter_sample_rate   = rtt.Property("double","reporter_sample_rate", "Frequency to take snapshots for the reporter")

tc:addProperty(_print_level)
tc:addProperty(_reporter_sample_rate)

--Ports which drive/read the FSM
_emperor_fsm_event_port      = rtt.InputPort("string")
_emperor_send_event_port     = rtt.OutputPort("string")
_emperor_failure_event_port  = rtt.OutputPort("string")
_emperor_current_state_port  = rtt.OutputPort("string")

--Ports to connect gamepad
_gamepad_A_port      = rtt.InputPort("bool")
_gamepad_B_port      = rtt.InputPort("bool")
_gamepad_up_port     = rtt.InputPort("bool")
_gamepad_down_port   = rtt.InputPort("bool")

tc:addPort(_emperor_fsm_event_port, "emperor_fsm_event_port", "Event port for driving the emperor FSM")
tc:addPort(_emperor_send_event_port, "emperor_send_event_port", "Port to send events to the emperor FSM from the emperor")
tc:addPort(_emperor_failure_event_port,"emperor_failure_event_port","Port to send indicate a failure in the emperor")
tc:addPort(_emperor_current_state_port, "emperor_current_state_port", "current active state of the emperor FSM")

tc:addPort(_gamepad_A_port, "gamepad_A_port", "A button of gamepad")
tc:addPort(_gamepad_B_port, "gamepad_B_port", "B button of gamepad")
tc:addPort(_gamepad_up_port, "gamepad_up_port", "Up button of gamepad")
tc:addPort(_gamepad_down_port, "gamepad_down_port", "Down button of gamepad")

_emperor_send_event_port:connect(_emperor_fsm_event_port)

function configureHook()
   -- create local copies of the property values
   print_level = _print_level:get()
   reporter_sample_rate = _reporter_sample_rate:get()

   -- variables to use in updateHook
   communicator = tc:getPeer('communicator')
   communicatorUpdate = communicator:getOperation("update")
   communicatorInRunTimeError = communicator:getOperation("inRunTimeError")

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
   -- update communication
   communicatorUpdate()
   if communicatorInRunTimeError() then
      rtt.logl("Error","RunTimeError in communicator")
      rfsm.send_events(fsm,'e_failed')
      return
   end

   rfsm.run(fsm)
   if main_state == 'idle' then
      menuToggle()
   elseif not (main_state == 'failure') then
      switchStates()
   end
end

function cleanupHook()
   rttlib.tc_cleanup()
end

function menuToggle()
   local fs_up, data_up       = _gamepad_up_port:read()
   local fs_down, data_down   = _gamepad_down_port:read()
   local fs_A, data_A         = _gamepad_A_port:read()

   if ((fs_up == 'NewData') and data_up) then
      menu_option_ind = (menu_option_ind)%table.getn(menu_options)+1
      print('Mode selected: '..menu_options[menu_option_ind])
   end
   if ((fs_down == 'NewData') and data_down) then
      menu_option_ind = (menu_option_ind-2)%table.getn(menu_options)+1
      print('Mode selected: '..menu_options[menu_option_ind])
   end
   if ((fs_A == 'NewData') and data_A) then
      print('Entering Mode '..menu_options[menu_option_ind])
      _emperor_send_event_port:write('e_'..main_states[menu_option_ind])
   end
end

function switchStates()
   local fs_A, data_A         = _gamepad_A_port:read()
   local fs_B, data_B         = _gamepad_B_port:read()
   if ((fs_A == 'NewData') and data_A) then
      if sub_state == 'idle' then
         _emperor_send_event_port:write('e_run')
      end
      if sub_state == 'stop' then
         _emperor_send_event_port:write('e_restart')
      end
   elseif ((fs_B == 'NewData') and data_B) then
      if sub_state == 'run' then
         _emperor_send_event_port:write('e_stop')
      end
      if sub_state == 'stop' then
         _emperor_send_event_port:write('e_reset')
      end
   end
end

-- local function to get the current time in seconds
function get_sec()
   local sec,nsec = rtt.getTime()
   return sec+nsec*1e-9
end
