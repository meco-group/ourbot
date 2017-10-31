local load_new_trajectory = reference:getOperation('receiveTrajectory')
local mp_period = math.floor(control_rate/motionplanning_rate)
local ref_cnt = 0
local mp_failure_cnt = 0
local mp_recover_cnt = 0
local predict_shift = 0
local busy = false

function set_motionplanner(mp, load_obstacles_fun)
  motionplanning = mp
  motionplanning:loadService('marshalling')
  if not motionplanning:provides('marshalling'):updateProperties('Configuration/system-config.cpf') then
    return false
  end
  if not motionplanning:provides('marshalling'):updateProperties('Configuration/motionplanning-config.cpf') then
    return false
  end
  mp_trigger_port = rtt.OutputPort('array')
  tc:addPort(mp_trigger_port, 'mp_trigger_port', 'Trigger for motion planning: is composed of current estimated pose and start index of internal reference input vector')
  target_pose_port = rtt.InputPort('array')
  tc:addPort(target_pose_port, 'target_pose_port', 'Target pose')
  if not communicator:addIncomingConnection('coordinator', 'target_pose_port', 'target_pose') then
    rfsm.send_events(fsm, 'e_failed')
  end
  local fs, tgt = target_pose_port:read() -- flush port
  if not deployer:connectPorts('motionplanning', 'estimator') then
    return false
  end
  if not deployer:connectPorts('motionplanning', 'reference') then
    return false
  end
  if not deployer:connectPorts('motionplanning', 'coordinator') then
    return false
  end
  if not deployer:addPeer('communicator', 'motionplanning') then
    return false
  end
  if not motionplanning:configure() then
    return false
  end
  deployer:setActivity('motionplanning', 0, 0, rtt.globals.ORO_SCHED_OTHER)

  motionplanning_error = motionplanning:getOperation('inRunTimeError')
  mp_reset = motionplanning:getOperation('reset')
  mp_busy = motionplanning:getOperation('busy')
  mp_valid = motionplanning:getOperation('valid')
  mp_ready = motionplanning:getOperation('ready')
  get_coeffs = motionplanning:getOperation('getCoefficients')
  get_basis_length = motionplanning:getOperation('getBasisLength')
  reset_obstacles = motionplanning:getOperation('resetObstacles')
  add_static_rect_obstacle = motionplanning:getOperation('addStaticRectObstacle')
  add_static_circ_obstacle = motionplanning:getOperation('addStaticCircObstacle')
  add_dynamic_rect_obstacle = motionplanning:getOperation('addDynamicRectObstacle')
  add_dynamic_circ_obstacle = motionplanning:getOperation('addDynamicCircObstacle')
  add_peer_rect_obstacle = motionplanning:getOperation('addPeerRectObstacle')
  add_peer_circ_obstacle = motionplanning:getOperation('addPeerCircObstacle')
  mp_max_failures = motionplanning:getProperty('max_failures'):get()
  mp_max_recovers = motionplanning:getProperty('max_recovers'):get()
  mp_max_periods = motionplanning:getProperty('max_periods'):get()
  load_obstacles = load_obstacles_fun
  return true
end

function rm_motionplanner()
  motionplanning:stop()
  motionplanning:cleanup()
  deployer:unloadComponent('motionplanning')
end

function trigger_motionplanning(predict_shift)
  load_obstacles()
  local _, pose0 = est_pose_port:read()
  -- print('('..pose0[0]..','..pose0[1]..','..pose0[2]..')')
  local trigger_data = rtt.Variable('array')
  trigger_data:resize(4)
  trigger_data:fromtab{pose0[0], pose0[1], pose0[2], predict_shift}
  mp_trigger_port:write(trigger_data)
end

local load_obstacles_fun = function()
  reset_obstacles()
  -- self.add_static_obstacle()
end

function p2p0_init(fsm)
  disable_manualcommand()
  busy = true
  trigger_motionplanning(0)
  ref_cnt = 0
  mp_failure_cnt = 0
  mp_recover_cnt = 0
  predict_shift = 0
  if not motionplanning:isRunning() then
    motionplanning:start()
  end
end

function p2p0_hook(fsm)
  -- control hook
  if not control_hook(false) or motionplanning_error() then
    rfsm.send_events(fsm, 'e_failed')
    return
  end
  -- check motion planning
  local busy_n = mp_busy()
  if (busy ~= busy_n) and not busy_n then -- decreasing flank detection
    if mp_ready() then
      print 'Target reached.'
      rfsm.send_events(fsm, 'e_idle')
      return
    end
    if mp_valid() then
      load_new_trajectory(0)
      rfsm.send_events(fsm, 'e_p2p')
      return
    else
      rtt.logl('Error', 'Motionplanning could not find a trajectory.')
      rfsm.send_events(fsm, 'e_idle')
      return
    end
  end
  busy = busy_n
end

function p2p_hook(fsm)
  -- trigger motion planning
  if ref_cnt == 0 then
    trigger_motionplanning(predict_shift)
  end
  -- control hook
  if not control_hook(true) or motionplanning_error() then
    rfsm.send_events(fsm, 'e_failed')
    return
  end
  -- check motion planning
  if mp_ready() then
    print 'Target reached.'
    rfsm.send_events(fsm, 'e_idle')
    return
  end
  -- motion planning check is related to next update
  ref_cnt = ref_cnt + 1
  -- e.g. new trajectories are followed, beginning in the nex update
  -- This behavior is similar as putting thte motion planning check in
  -- start of update. But using the current implementation,  the
  -- control_hook update is synced/clocked better.
  if not mp_busy() and ref_cnt >= mp_period then
    if mp_valid() then
      predict_shift = math.max(0, ref_cnt - mp_period)
      load_new_trajectory(predict_shift)
      ref_cnt = 0
    else
      -- try again
      mp_failure_cnt = mp_failure_cnt + 1
      if mp_failure_cnt < mp_max_failures then
        trigger_motionplanning(math.max(0, ref_cnt - mp_period))
      else
        rtt.logl('Error', 'Motionplanning got ' .. mp_max_failures .. ' consecutive invalid solutions. Recover...')
        rfsm.send_events(fsm, 'e_recover')
        return
      end
    end
  end
  if ref_cnt > mp_max_periods*mp_period then
    rtt.logl('Error', 'Motionplanning takes too long. Recover...')
    rfsm.send_events(fsm, 'e_recover')
    return
  end
  if ref_cnt >= mp_period and math.fmod(ref_cnt, mp_period) == 0 then
    rtt.logl('Warning', 'Motionplanning takes longer than '.. math.floor(ref_cnt/mp_period) ..' period(s)!')
  end
end

function recover_init(fsm)
  mp_recover_cnt = mp_recover_cnt + 1
  if mp_recover_cnt >= mp_max_recovers then
    rtt.logl('Error', 'Motionplanning recovered ' .. mp_max_recovers .. ' times. Giving up...')
    rfms.send_events(fsm, 'e_idle')
    return
  end
end

function recover_hook(fsm, time0)
  -- control hook
  if not control_hook(false) or motionplanning_error() then
    rfsm.send_events(fsm, 'e_failed')
    return
  end
  -- wait a little bit and try again
  if (get_sec() - time0) >= 1. then
    rfsm.send_events(fsm, 'e_p2p')
    return
  end
end

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init', tgt = 'home', events = {'e_done'}},
  rfsm.trans{src = 'home', tgt = 'idle', events = {'e_done', 'e_back'}},
  rfsm.trans{src = 'idle', tgt = 'p2p0', events = {'e_p2p'}},
  rfsm.trans{src = 'p2p0', tgt = 'idle', events = {'e_idle', 'e_back'}},
  rfsm.trans{src = 'p2p0', tgt = 'p2p', events = {'e_p2p'}},
  rfsm.trans{src = 'p2p', tgt = 'idle', events = {'e_idle', 'e_back'}},
  rfsm.trans{src = 'p2p', tgt = 'recover', events = {'e_recover'}},
  rfsm.trans{src = 'recover', tgt = 'p2p0', events = {'e_p2p'}},
  rfsm.trans{src = 'recover', tgt = 'idle', events = {'e_idle', 'e_back'}},
  rfsm.trans{src = 'home', tgt = 'stop', events = {'e_back'}},
  rfsm.trans{src = 'idle', tgt = 'stop', events = {'e_back'}},
  rfsm.trans{src = 'home', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'idle', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'p2p0', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'p2p', tgt = 'failure', events = {'e_failed'}},
  rfsm.trans{src = 'recover', tgt = 'failure', events = {'e_failed'}},

  initial = rfsm.conn{},

  init = rfsm.state{
    entry = function(fsm)
      if not deployer:loadComponent('motionplanning', 'MotionPlanning') then
        rfsm.send_events(fsm, 'e_failed')
      end
      if not set_motionplanner(deployer:getPeer('motionplanning'), load_obstacles_fun) then
        rfsm.send_events(fsm, 'e_failed')
        return
      end
      if not start_control_components() then
        rfsm.send_events(fsm, 'e_failed')
      end
    end
  },

  home = rfsm.state{
    -- wait until valid estimate
    doo = function(fsm)
      while true do
        if not control_hook(false) then
          rfsm.send_events(fsm, 'e_failed')
        end
        if estimator_valid() then
          return
        end
        rfsm.yield(true)
      end
    end
  },

  idle = rfsm.state{
    entry = function(fsm)
      enable_manualcommand()
      zero_velocity()

    end,

    doo = function(fsm)
      while true do
        if not control_hook(false) or motionplanning_error() then
          rfsm.send_events(fsm, 'e_failed')
        end
        local fs, target_pose = target_pose_port:read()
        if fs == 'NewData' then
          mp_reset()
          motionplanning:setTargetPose(target_pose)
          rfsm.send_events(fsm, 'e_p2p')
        end
        rfsm.yield(true)
      end
    end,
  },

  p2p0 = rfsm.state{
    doo = function(fsm)
      p2p0_init(fsm)
      while true do
        p2p0_hook(fsm)
        rfsm.yield(true)
      end
    end,
  },

  p2p = rfsm.state{
    doo = function(fsm)
      while true do
        p2p_hook(fsm)
        rfsm.yield(true)
      end
    end,
  },

  recover = rfsm.state{
    doo = function(fsm)
      recover_init(fsm)
      local time0 = get_sec()
      while true do
        recover_hook(fsm, time0)
        rfsm.yield(true)
      end
    end,
  },

  stop = rfsm.state {
    entry = function(fsm)
      rm_motionplanner()
      stop_control_components()
    end,
  },

  failure = rfsm.state {
    entry = function(fsm)
      rm_motionplanner()
      stop_control_components()
      rfsm.send_events(fsm, 'e_failed')
    end,
  },
}
