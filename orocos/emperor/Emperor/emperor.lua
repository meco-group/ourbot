require "rttlib"
require "rfsm"
require "rfsm_rtt"
require "rfsmpp"

local tc=rtt.getTC();
local fsm
local fqn_out, events_in
local start_time
state = ''
-- local menu_options = {{'VelocityControl','e_velocitycmdextern'},{'PathFollowing','e_updpathfollowing'}}
local menu_options         = {'VelocityControl','PathFollowing'}
local menu_options_events  = {'e_velocitycmdextern', 'e_updpathfollowing'}
local menu_option_ind = 1

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

--Ports to connect gamepad
_gamepad_A_port      = rtt.InputPort("bool")
_gamepad_up_port     = rtt.InputPort("bool")
_gamepad_down_port   = rtt.InputPort("bool")

tc:addEventPort(_emperor_fsm_event_port, "emperor_fsm_event_port", "Event port for driving the emperor FSM")
tc:addPort(_emperor_send_event_port, "emperor_send_event_port", "Port to send events to the emperor FSM from the emperor")
tc:addPort(_emperor_failure_event_port,"emperor_failure_event_port","Port to send indicate a failure in the emperor")
tc:addPort(_emperor_current_state_port, "emperor_current_state_port", "current active state of the emperor FSM")

tc:addEventPort(_gamepad_A_port, "gamepad_A_port", "A button of gamepad")
tc:addEventPort(_gamepad_up_port, "gamepad_up_port", "Up button of gamepad")
tc:addEventPort(_gamepad_down_port, "gamepad_down_port", "Down button of gamepad")

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
   if state == 'idle' then
      menuToggle()
   elseif state == ''
   if state == ''
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
      _emperor_send_event_port:write(menu_options_events[menu_option_ind])
   end
end

-- function menuChoice()
--    if ((_gamepad_A_port.read(sample) == rtt.NewData) and sample) then
--       _emperor_send_event_port.write(menu_options[menu_option_ind[1]])
--    end
-- end

--Local function to get the current time in seconds
function get_sec()
   local sec,nsec = rtt.getTime()
   return sec+nsec*1e-9
end
