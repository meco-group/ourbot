-- local variables
local home_pose

function build_formation()
  local peers = communicator:getGroupPeers('ourbots')
  local _, pose0 = est_pose_port:read()
  -- communicate pose
  for k=0, peers.size-1 do
    communicator:writeMail(string.format('%+.3f,%+.3f,%+.3f', pose0[0], pose0[1], pose0[2]), communicator:getPeerUUID(peers[k]))
  end
  -- receive pose
  local pose = rtt.Variable('array')
  pose:resize(3)
  local continue = true
  for k=0, peers.size-1 do
    if peers[k] == host then
      motionplanning:addRobot(peers[k], pose0)
    else
      while(continue) do
        local ret = communicator:readMail(false)
        while (ret.size == 2) do
          -- decode message
          local msg = ret[0]
          local uuid = ret[1]

          if uuid == communicator:getPeerUUID(peers[k]) then
            local a, b, c, d, e, f = string.match(msg, '.-(%d+).(%d+),.-(%d+).(%d+),.-(%d+).(%d+)')
            if a ~= nil and b ~= nil and c ~= nil and d ~= nil and e ~= nil and f ~= nil then
              pose[0] = a + 0.001*b
              pose[1] = c + 0.001*d
              pose[2] = e + 0.001*f
              if string.match(msg, '%-.*,.*,.*') then
                  pose[0] = -pose[0]
              end
              if string.match(msg, '.*,%-.*,.*') then
                  pose[1] = -pose[1]
              end
              if string.match(msg, '.*,.*,%-.*') then
                  pose[2] = -pose[2]
              end
              motionplanning:addRobot(peers[k], pose)
              communicator:removeMail()
              continue = false
              break
            end
          end
          ret = communicator:readMail(false)
        end
        rfsm.yield(true)
      end
    end
  end
  return motionplanning:buildFormation()
end

local initialize = function()
  local n_robots = communicator:getGroupSize('ourbots')
  local neighbors = motionplanning:getNeighbors()

  for k=0, 1 do
    if (n_robots == 2) then
      if not communicator:addOutgoingConnection('motionplanning', 'x_var_port_' .. tostring(k), 'x_var_' .. host .. tostring(k), neighbors[k]) then return false end
      if not communicator:addOutgoingConnection('motionplanning', 'zl_ij_var_port_' .. tostring(k), 'zl_var_' .. host .. tostring(k), neighbors[k]) then return false end
      if not communicator:addIncomingConnection('motionplanning', 'x_j_var_port_' .. tostring(k), 'x_var_' .. neighbors[k] .. tostring(k)) then return false end
      if not communicator:addIncomingConnection('motionplanning', 'zl_ji_var_port_' .. tostring(k), 'zl_var_' .. neighbors[k] .. tostring(k)) then return false end
    else
      if not communicator:addOutgoingConnection('motionplanning', 'x_var_port_' .. tostring(k), 'x_var_' .. host, neighbors[k]) then return false end
      if not communicator:addOutgoingConnection('motionplanning', 'zl_ij_var_port_' .. tostring(k), 'zl_var_' .. host, neighbors[k]) then return false end
      if not communicator:addIncomingConnection('motionplanning', 'x_j_var_port_' .. tostring(k), 'x_var_' .. neighbors[k]) then return false end
      if not communicator:addIncomingConnection('motionplanning', 'zl_ji_var_port_' .. tostring(k), 'zl_var_' .. neighbors[k]) then return false end
    end
  end
  return true
end

local load_obstacles_fun = function()
  reset_obstacles()
  -- rectangular obstacles
  local r_data = get_rectangular_obstacles(n_obstacles())
  local n_rect = r_data.size/4
  for k=0, n_rect-1 do
    local pose = rtt.Variable('array')
    pose:resize(3)
    pose:fromtab{r_data[5*k], r_data[5*k+1], r_data[5*k+2]}
    local size = rtt.Variable('array')
    size:resize(2)
    size:fromtab{r_data[5*k+3], r_data[5*k+4]}
    add_static_rect_obstacle(pose, size)
  end
  -- circular obstacles
  local c_data = get_circular_obstacles(0)
  local n_circ = c_data.size/3
  for k=0, n_circ-1 do
    local pos = rtt.Variable('array')
    pos:resize(2)
    pos:fromtab{c_data[3*k], c_data[3*k+1]}
    add_static_circ_obstacle(pos, c_data[3*k+2])
  end


end

local mp_fsm = rfsm.load('Coordinator/motionplanning_fsm.lua')

mp_fsm.init = rfsm.state{
  entry = function(fsm)
    if not deployer:loadComponent('motionplanning', 'FormationMotionPlanning') then
      rfsm.send_events(fsm, 'e_failed')
    end
    if not set_motionplanner(deployer:getPeer('motionplanning'), load_obstacles_fun) then
      rfsm.send_events(fsm, 'e_failed')
      return
    end
    if not motionplanning:provides('marshalling'):updateProperties('Configuration/formation-config.cpf') then
      rfsm.send_events(fsm, 'e_failed')
      return
    end
    motionplanning:configure()
    if not start_control_components() then
      rfsm.send_events(fsm, 'e_failed')
      return
    end
  end,
}

mp_fsm.home = rfsm.state{
    doo = function(fsm)
      while true do
        if not control_hook(false) then
          rfsm.send_events(fsm, 'e_failed')
        end
        if estimator_valid() then
          home_pose = build_formation()
          if not initialize() then
            rfsm.send_events(fsm, 'e_failed')
          end
          mp_reset()
          motionplanning:setTargetPose(home_pose)
          rfsm.send_events(fsm, 'e_p2p')
        end
        rfsm.yield(true)
      end
    end,
}

-- mp_fsm.stop.exit = function(fsm)
--   release()
-- end

-- mp_fsm.failure.exit = function(fsm)
--   release()
-- end

return mp_fsm
