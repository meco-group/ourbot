require './Coordinator/subcoordinator'
require './Coordinator/trajectoryfollowing'
require './Coordinator/motionplanning'

local tc = rtt.getTC()
local teensy = tc:getPeer('teensy')
local coordinator_failure_event_port = tc:getPort('coordinator_failure_event_port')

local sub_coordinator = SubCoordinator()
local trajectoryfollowing_coordinator = TrajectoryFollowingCoordinator()
local motionplanning_coordinator = MotionPlanningCoordinator()


return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'idle', tgt = 'trajectoryfollowing', events = {'e_trajectoryfollowing'}},
  rfsm.trans{src = 'idle', tgt = 'motionplanning', events = {'e_motionplanning'}},

  rfsm.trans{src = 'init', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'idle', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'trajectoryfollowing', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'motionplanning', tgt = 'failure', events = {'e_failed'}},

  rfsm.trans{src = 'trajectoryfollowing.stop', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'motionplanning.stop', tgt = 'idle', events = {'e_done'}},

  initial = rfsm.conn{},

  init = rfsm.state{},

  failure = rfsm.state{
    entry = function()
      teensy:setVelocity(0, 0, 0)
      coordinator_failure_event_port:write('e_failed')
    end
  },

  idle = sub_coordinator:get_fsm(),
  trajectoryfollowing = trajectoryfollowing_coordinator:get_fsm(),
  motionplanning = motionplanning_coordinator:get_fsm(),
  -- formation = rfsm.load('Coordinator/formation_fsm.lua'),
}
