local tc = rtt.getTC()

local estimator_valid = estimator:getOperation('valid')
local load_new_trajectory = reference:getOperation('receiveTrajectory')

local motionplanning
local motionplanning_error
local mp_reset
local mp_busy
local mp_valid
local mp_ready

-- add ports
local _est_pose_port = rtt.InputPort('array')
local _target_pose_port = rtt.InputPort('array')
local _mp_trigger_port = rtt.OutputPort('array')
tc:addPort(_est_pose_port, 'est_pose_port', 'Estimated pose wrt to initial frame')
tc:addPort(_target_pose_port, 'target_pose_port', 'Target pose')
tc:addPort(_mp_trigger_port, 'mp_trigger_port', 'Trigger for motion planning: is composed of current estimated pose and start index of internal reference input vector')

-- connect ports
_est_pose_port:connect(estimator:getPort('est_pose_port'))
if not communicator:addIncomingConnection('coordinator', 'target_pose_port', 'target_pose') then
  rfsm.send_events(fsm, 'e_failed')
end

-- local variables
local n_robots = communicator:getGroupSize('ourbots')
local ref_cnt = 0
local mp_failure_cnt = 0
local mp_recover_cnt = 0
local mp_max_failures
local mp_max_recovers
local mp_max_periods
local mp_period = math.floor(control_rate/motionplanning_rate)

local pose0

function load_motionplanner()
  if not dp:loadComponent('motionplanning', 'FormationMotionPlanning') then
    return false
  end
  mp = dp:getPeer('motionplanning')
  mp:loadService('marshalling')
  if not mp:provides('marshalling'):updateProperties('Configuration/system-config.cpf') then
    return false
  end
  if not mp:provides('marshalling'):updateProperties('Configuration/motionplanning-config.cpf') then
    return false
  end
  if not mp:provides('marshalling'):updateProperties('Configuration/formation-config.cpf') then
    return false
  end
  if not dp:connectPorts('motionplanning', 'estimator') then
    return false
  end
  if not dp:connectPorts('motionplanning', 'reference') then
    return false
  end
  if not dp:connectPorts('motionplanning', 'coordinator') then
    return false
  end
  if not dp:addPeer('communicator', 'motionplanning') then
    return false
  end
  if not communicator:addIncomingConnection('motionplanning', 'obstacle_port', 'obstacles') then
    return false
  end
  if not mp:configure() then
    return false
  end
  dp:setActivity('motionplanning', 0, 0, rtt.globals.ORO_SCHED_OTHER)
  mp:start()
  return true
end

function unload_motionplanner()
  motionplanning:stop()
  dp:unloadComponent('motionplanning')
end



function build_formation()
  local peers = communicator:getGroupPeers('ourbots')
  local _, pose0 = _est_pose_port:read()
  -- communicate pose
  for k=0, peers.size-1 do
    communicator:writeMail(string.format('%.3f,%.3f,%.3f', pose0[0], pose0[1], pose0[2]), communicator:getPeerUUID(peers[k]))
  end
  -- receive pose
  local pose = rtt.Variable('array')
  pose:resize(3)
  local continue = true
  for k=0, peers.size-1 do
    while(continue) do
      local ret = communicator:readMail(false)
      while (ret.size == 2) do
        -- decode message
        local msg = ret[0]
        local uuid = ret[1]
        if uuid == communicator:getPeerUUID(peers[k]) then
          local a, b, c, d, e, f = string.match(msg, '(%d+).(%d+),(%d+).(%d+),(%d+).(%d+)')
          if a ~= nil and b ~= nil and c ~= nil and d ~= nil and e ~= nil and f ~= nil then
            pose[0] = a + 1000*b
            pose[1] = c + 1000*d
            pose[2] = e + 1000*f
            motionplanning:addNeighbor(peers[k], pose)
            communicator:removeMail()
            continue = false
          end
        end
        ret = communicator:readMail(false)
      end
      rfsm.yield(true)
    end
  end
  return motionplanning:buildFormation()
end

function connect_neighbors()
  local neighbors = motionplanning:getNeighbors()
  for k=0, 1 do
    if (n_robots == 2) then
      communicator:addOutgoingConnection('motionplanning', 'x_var_port_' .. tostring(k), 'x_var_' .. host .. tostring(k), neighbors[k])
      communicator:addOutgoingConnection('motionplanning', 'zl_ij_var_port_' .. tostring(k), 'zl_var_' .. host .. tostring(k), neighbors[k])
      communicator:addIncomingConnection('motionplanning', 'x_j_var_port_' .. tostring(k), 'x_var_' .. neighbors[k] .. tostring(k))
      communicator:addIncomingConnection('motionplanning', 'zl_ji_var_port_' .. tostring(k), 'zl_var_' .. neighbors[k] .. tostring(k))
    else
      communicator:addOutgoingConnection('motionplanning', 'x_var_port_' .. tostring(k), 'x_var_' .. host, neighbors[k])
      communicator:addOutgoingConnection('motionplanning', 'zl_ij_var_port_' .. tostring(k), 'zl_var_' .. host, neighbors[k])
      communicator:addIncomingConnection('motionplanning', 'x_j_var_port_' .. tostring(k), 'x_var_' .. neighbors[k])
      communicator:addIncomingConnection('motionplanning', 'zl_ji_var_port_' .. tostring(k), 'zl_var_' .. neighbors[k])
    end
  end
end

return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init', tgt = 'home', events = {'e_done'}},
  rfsm.trans{src = 'home', tgt = 'p2p0', events = {'e_p2p'}},
  rfsm.trans{src = 'idle', tgt = 'p2p0', events = {'e_p2p'}},
  rfsm.trans{src = 'p2p0', tgt = 'idle', events = {'e_idle'}},
  rfsm.trans{src = 'p2p0', tgt = 'p2p', events = {'e_done'}},
  rfsm.trans{src = 'p2p', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'p2p', tgt = 'recover', events = {'e_recover'}},
  rfsm.trans{src = 'recover', tgt = 'p2p0', events = {'e_p2p'}},
  rfsm.trans{src = 'recover', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'home', tgt = 'stop', events = {'e_back'}},
  rfsm.trans{src = 'idle', tgt = 'stop', events = {'e_back'}},
  rfsm.trans{src = 'p2p0', tgt = 'stop', events = {'e_back'}},
  rfsm.trans{src = 'p2p', tgt = 'stop', events = {'e_back'}},
  rfsm.trans{src = 'recover', tgt = 'stop', events = {'e_back'}},

  initial = rfsm.conn{},

  init = rfsm.state{
    entry = function(fsm)
      if not load_motionplanner() then
        rfsm.send_events(fsm, 'e_failed')
        return
      end
      motionplanning = dp:getPeer('motionplanning')
      motionplanning_error = motionplanning:getOperation('inRunTimeError')
      mp_reset = motionplanning:getOperation('reset')
      mp_busy = motionplanning:getOperation('busy')
      mp_valid = motionplanning:getOperation('valid')
      mp_ready = motionplanning:getOperation('ready')
      mp_max_failures = motionplanning:getProperty('max_failures'):get()
      mp_max_recovers = motionplanning:getProperty('max_recovers'):get()
      mp_max_periods = motionplanning:getProperty('max_periods'):get()
      if not fun:start_control_components() then
        rfsm.send_events(fsm, 'e_failed')
      end
    end,

    doo = function(fsm)
      pose0 = build_formation()
      connect_neighbors()
    end
  },

  home = rfsm.state{
    doo = function(fsm)
      while true do
        if not fun:control_hook(false) then
          rfsm.send_events(fsm, 'e_failed')
        end
        if estimator_valid() then
          motionplanning:setTargetPose(pose0)
          mp_reset()
          rfsm.send_events(fsm, 'e_p2p')
        end
        rfsm.yield(true)
      end
    end
  },

  idle = rfsm.state{
    entry = function(fsm)
      teensy:setVelocity(0, 0, 0)
      fun:enable_manualcommand()
    end,

    doo = function(fsm)
      while true do
        if not fun:control_hook(false) or not motionplanning_error then
          rfsm.send_events(fsm, 'e_failed')
        end
        local fs, target_pose = _target_pose_port:read()
        if fs == 'NewData' then
          motionplanning:setTargetPose(target_pose)
          mp_reset()
          rfsm.send_events(fsm, 'e_p2p')
        end
        rfsm.yield(true)
      end
    end,
  },

  p2p0 = rfsm.state{
    doo = function(fsm)
      fun:disable_manualcommand()
      local busy = mp_busy()
      local busy_n
      trigger_motionplanning(0)
      ref_cnt = 0
      mp_failure_cnt = 0
      mp_recover_cnt = 0
      while true do
        if not fun:control_hook(false) or not motionplanning_error then
          rfsm.send_events(fsm, 'e_failed')
        end
        busy_n = mp_busy()
        if (busy or busy_n and busy ~= busy_n) and not busy_n then -- decreasing flank detection
          if mp_valid() then
            load_new_trajectory(0)
            return
          else
            rtt.logl('Error', 'Motionplanning could not find a trajectory.')
            rfsm.send_events(fsm, 'e_idle')
            return
          end
        end
        busy = busy_n
        rfsm.yield(true)
      end
    end,
  },

  p2p = rfsm.state{
    doo = function(fsm)
      local start_time = fun:get_sec()
      local prev_start_time = start_time
      local end_time = start_time
      local predict_shift = 0
      while true do
        prev_start_time = start_time
        start_time = fun:get_sec()
        if ref_cnt == 0 then
          trigger_motionplanning(predict_shift)
        end
        if not fun:control_hook(true) or not motionplanning_error then
          rfsm.send_events(fsm, 'e_failed')
        end
        -- check motion planning
        if mp_ready() then
          print 'Target reached.'
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
        fun:guard_time(start_time, prev_start_time, end_time)
        end_time = fun:get_sec()
        rfsm.yield(true)
      end
    end,
  },

  recover = rfsm.state{
    doo = function(fsm)
      mp_recover_cnt = mp_recover_cnt + 1
      if mp_recover_cnt >= mp_max_recovers then
        rtt.logl('Error', 'Motionplanning recovered ' .. mp_max_recovers .. ' times. Giving up...')
        return
      end
      local time0 = fun:get_sec()
      local recover_time = 1.
      while true do
        if not fun:control_hook(false) or not motionplanning_error then
          rfsm.send_events(fsm, 'e_failed')
        end
        -- wait a little bit and try again
        if fun:get_sec() - time0 >= recover_time then
          rfsm.send_events(fsm, 'e_p2p')
        end
        rfsm.yield(true)
      end
    end,
  },

  stop = rfsm.state {
    entry = function(fsm)
      unload_motionplanner()
      fun:stop_control_components()
    end,
  },
}
