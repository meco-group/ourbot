local fun = require './Coordinator/functions'
local tc = rtt.getTC()
local teensy = tc:getPeer('teensy')

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init', tgt = 'idle'},
  rfsm.trans{src = 'idle', tgt = 'motionplanning', events = {'e_motionplanning'}},
  rfsm.trans{src = 'idle', tgt = 'trajectoryfollowing', events = {'e_trajectoryfollowing'}},
  -- rfsm.trans{src = 'idle',                      tgt = 'velocitycmd',        events = {'e_velocitycmd'}},
  -- rfsm.trans{src = 'idle',                      tgt = 'flexonomy',          events = {'e_flexonomy'}},
  rfsm.trans{src = 'motionplanning', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'trajectoryfollowing', tgt = 'failure', events = {'e_failed'}},
  -- rfsm.trans{src = 'velocitycmd',               tgt = 'failure',            events = {'e_failed'}},
  -- rfsm.trans{src = 'trajectoryfollowing',       tgt = 'failure',            events = {'e_failed'}},
  -- rfsm.trans{src = 'flexonomy',                 tgt = 'failure',            events = {'e_failed'}},
  rfsm.trans{src = 'motionplanning.stop', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'trajectoryfollowing.stop', tgt = 'idle', events = {'e_done'}},
  -- rfsm.trans{src = 'velocitycmd.idle',          tgt = 'idle',               events = {'e_done'}},
  -- rfsm.trans{src = 'trajectoryfollowing.idle',  tgt = 'idle',               events = {'e_done'}},

  initial = rfsm.conn{},

  init = rfsm.state{
    entry = function()
      if not fun:start_sensing_components() then
        rfsm.send_events(fsm, 'e_failed')
      end
    end,
  },

  idle = rfsm.state{
    entry = function()
      teensy:setVelocity(0, 0, 0)
      fun:enable_manualcommand()
      if obstacle_mode then
        print('\nIn obstacle mode (control with gamepad)')
        _coordinator_send_event_port:write('e_velocitycmd')
      else
        print('\nWaiting for command...\nPossibilities: VelocityControl, MotionPlanning, TrajectoryFollowing, Flexonomy')
      end
    end,

    doo = function()
      while true do
        if not fun:control_hook(false) then
          rfsm.send_events(fsm, 'e_failed')
        end
        rfsm.yield(true)
      end
    end,
  },

  failure = rfsm.state{
    entry = function()
      fun:stop_sending_components()
      _coordinator_failure_event_port:write('e_failed')
    end
  },

  motionplanning = rfsm.load('Coordinator/motionplanning_fsm.lua'),
  trajectoryfollowing = rfsm.load('Coordinator/trajectoryfollowing_fsm.lua'),
  -- velocitycmd         = rfsm.load('Coordinator/velocitycmd_fsm.lua'),
  -- flexonomy           = rfsm.load('Coordinator/flexonomy_fsm.lua'),
}
