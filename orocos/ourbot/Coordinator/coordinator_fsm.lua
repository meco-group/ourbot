return rfsm.state {

  rfsm.trans{src = 'initial',                   tgt = 'idle'},
  rfsm.trans{src = 'idle',                      tgt = 'motionplanning',     events = {'e_motionplanning'}},
  rfsm.trans{src = 'idle',                      tgt = 'velocitycmd',        events = {'e_velocitycmd'}},
  rfsm.trans{src = 'idle',                      tgt = 'trajectoryfollowing',events = {'e_trajectoryfollowing'}},
  rfsm.trans{src = 'motionplanning',            tgt = 'failure',            events = {'e_failed'}},
  rfsm.trans{src = 'velocitycmd',               tgt = 'failure',            events = {'e_failed'}},
  rfsm.trans{src = 'trajectoryfollowing',       tgt = 'failure',            events = {'e_failed'}},
  rfsm.trans{src = 'motionplanning.idle',       tgt = 'idle',               events = {'e_done'}},
  rfsm.trans{src = 'velocitycmd.idle',          tgt = 'idle',               events = {'e_done'}},
  rfsm.trans{src = 'trajectoryfollowing.idle',  tgt = 'idle',               events = {'e_done'}},
  rfsm.trans{src = 'failure',                   tgt = 'idle',               events = {'e_recover'}},
    --add more state transitions here

  initial = rfsm.conn{},

  idle = rfsm.state{
    entry=function()
      if obstacle_mode then
        print('\nIn obstacle mode (control with gamepad)')
        _coordinator_send_event_port:write('e_velocitycmd')
      else
        print('\nWaiting for command...\nPossibilities: VelocityControl, MotionPlanning, TrajectoryFollowing')
      end
    end
  },

  failure = rfsm.state{
    entry = function()
      _coordinator_failure_event_port:write('e_failed')
    end
  },

  motionplanning      = rfsm.load("Coordinator/motionplanning_fsm.lua"),
  velocitycmd         = rfsm.load("Coordinator/velocitycmd_fsm.lua"),
  trajectoryfollowing = rfsm.load("Coordinator/trajectoryfollowing_fsm.lua"),
    --add more state descriptions here

}
