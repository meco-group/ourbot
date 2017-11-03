-- create properties
tc:addProperty(rtt.Property('string', 'dynamic_obstacle', 'Name of ourbot that plays dynamic obstacle'))
-- read properties
if not tc:provides('marshalling'):updateProperties('Configuration/system-config.cpf') then
  return false
end
local dynamic_obstacle = tc:getProperty('dynamic_obstacle'):get()
local host_obstacle = (host == dynamic_obstacle)
local ourbot_size = tc:getProperty('ourbot_size')
local ourbot_radius = 1.1*math.sqrt(math.pow(0.5*ourbot_size[0], 2) + math.pow(0.5*ourbot_size[1], 2))

local initialize = function()
  -- add ports
  if host_obstacle then
    if not communicator:addOutgoingConnection('estimator', 'est_pose_port', 'dynamic_obstacle_pose', 'ourbots') then rfsm.send_events(fsm, 'e_failed') return false end
    if not communicator:addOutgoingConnection('estimator', 'est_velocity_port', 'dynamic_obstacle_velocity', 'ourbots') then rfsm.send_events(fsm, 'e_failed') return false end
  else
    dynamic_obstacle_pose_port = rtt.InputPort('array')
    dynamic_obstacle_velocity_port = rtt.InputPort('array')
    tc:addPort(dynamic_obstacle_pose_port, 'dynamic_obstacle_pose_port', 'Pose of dynamic obstacle')
    tc:addPort(dynamic_obstacle_velocity_port, 'dynamic_obstacle_velocity_port', 'Velocity of dynamic obstacle')
    if not communicator:addIncomingConnection('coordinator', 'dynamic_obstacle_pose_port', 'dynamic_obstacle_pose') then rfsm.send_events(fsm, 'e_failed') return false end
    if not communicator:addIncomingConnection('coordinator', 'dynamic_obstacle_velocity_port', 'dynamic_obstacle_velocity') then rfsm.send_events(fsm, 'e_failed') return false end
  end
  return true
end

local release = function()
  if host_obstacle then
    communicator:removeConnection('estimator', 'est_pose_port', 'dynamic_obstacle_pose')
    communicator:removeConnection('estimator', 'est_velocity_port', 'dynamic_obstacle_velocity')
  else
    communicator:removeConnection('coordinator', 'dynamic_obstacle_pose_port', 'dynamic_obstacle_pose')
    communicator:removeConnection('coordinator', 'dynamic_obstacle_velocity_port', 'dynamic_obstacle_velocity')
  end
end

local load_obstacles_fun = function ()
  reset_obstacles()
  -- get dynamic obstacle pose/velocity
  local fs1, dynamic_obstacle_pose = dynamic_obstacle_pose_port:read()
  local fs2, dynamic_obstacle_velocity = dynamic_obstacle_velocity_port:read()
  if fs1 == 'NewData' then
    if fs2 == 'NewData' then
      add_dynamic_circ_obstacle(dynamic_obstacle_pose, dynamic_obstacle_velocity, ourbot_radius)
    else
      rtt.logl('Warning', 'Did not receive velocity info of dynamic obstacle!')
      add_static_circ_obstacle(dynamic_obstacle_pose, ourbot_radius)
    end
  else
    rtt.logl('Warning', 'Did not receive pose and velocity info of dynamic obstacle!')
  end
end

if host_obstacle then
  return rfsm.state {
    rfsm.trans{src = 'initial', tgt = 'init'},
    rfsm.trans{src = 'init', tgt = 'idle', events = {'e_done'}},
    rfsm.trans{src = 'idle', tgt = 'stop', events = {'e_back'}},

    initial = rfsm.conn{},

    init = rfsm.state{
      entry = function(fsm)
        if not initialize() then
          rfsm.send_events(fsm, 'e_failed')
        end
      end
    },

    idle = rfsm.state{
      entry = function(fsm)
        zero_velocity()
        enable_manualcommand()
      end,

      doo = function(fsm)
        while true do
          if not control_hook(false) then
            rfsm.send_events(fsm, 'e_failed')
          end
          rfsm.yield(true)
        end
      end,
    },

    stop = rfsm.state{
      entry = function(fsm)
        release()
      end,
    },

    failure = rfsm.state{
      entry = function(fsm)
        release()
      end,
    },
  }
else
  local mp_fsm = rfsm.load('Coordinator/motionplanning_fsm.lua')

  mp_fsm.init = rfsm.state{
    entry = function(fsm)
      if not deployer:loadComponent('motionplanning', 'MotionPlanningDynamicObstacle') then
        rfsm.send_events(fsm, 'e_failed')
      end
      if not set_motionplanner(deployer:getPeer('motionplanning'), load_obstacles_fun) then
        rfsm.send_events(fsm, 'e_failed')
        return
      end
      initialize()
      if not start_control_components() then
        rfsm.send_events(fsm, 'e_failed')
      end
    end
  }

  mp_fsm.stop.exit = function(fsm)
    release()
  end

  mp_fsm.failure.exit = function(fsm)
    release()
  end

  return mp_fsm
end
