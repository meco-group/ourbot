local json = require "cjson"

-- local variables
local current_eta = nil
local current_task = nil
local task_queue = {}
local time_tbl = {}
local current_task_canceled = false
local priority = false
local state = 'busy'
local ts_prev = 0

local peer_uuid = communicator:getOperation('getPeerUUID')
local write_mail = communicator:getOperation('writeMail')
local read_mail = communicator:getOperation('readMail')
local remove_mail = communicator:getOperation('removeMail')
local msg_tbl = {
  header = {
    version = header_version,
    type = '',
    model = '',
    uuid = communicator:getUUID(),
    timestamp = ''
  },
  payload = {}
}
local n_tables = 0
local robot_tables = {}

local init_robot_tables = function()
  local robot_table_ids = tc:getProperty('robot_table_ids'):get()
  local robot_table_sizes = tc:getProperty('robot_table_sizes'):get()
  n_tables = math.min(n_obstacles()-1, robot_table_ids.size)
  for k=0, n_tables-1 do
    robot_tables[k] = {id = robot_table_ids[k], length = robot_table_sizes[2*k], width = robot_table_sizes[2*k+1],
                       no_robot_cnt = no_robot_cnt_max}
  end
  print('Added ' .. n_tables .. ' robot table(s).')
end

local initialize = function()
  -- create properties
  tc:addProperty(rtt.Property('string', 'coordinator_name', 'Name of coordinating node'))
  tc:addProperty(rtt.Property('string', 'header_version', 'Version of communication header'))
  tc:addProperty(rtt.Property('double', 'statemsg_rate', 'Rate to send state messages'))
  tc:addProperty(rtt.Property('double', 'nghbcom_rate', 'Rate to communicate trajectories to neighbor'))
  tc:addProperty(rtt.Property('ints', 'robot_table_ids', 'Ids of robot tables to detect'))
  tc:addProperty(rtt.Property('array', 'robot_table_sizes', 'Length and width of robot arm tables'))
  tc:addProperty(rtt.Property('int', 'no_robot_cnt_max', 'Maximum iterations to decide that robot arm is not visible'))
  -- read properties
  if not tc:provides('marshalling'):updateProperties('Configuration/flexonomy-config.cpf') then
    return false
  end
  coordinator_name = tc:getProperty('coordinator_name'):get()
  header_version = tc:getProperty('header_version'):get()
  statemsg_rate = tc:getProperty('statemsg_rate'):get()
  nghbcom_rate = tc:getProperty('nghbcom_rate'):get()
  no_robot_cnt_max = tc:getProperty('no_robot_cnt_max'):get()
  local ourbot_size = tc:getProperty('ourbot_size')
  ourbot_radius = 1.2*math.sqrt(math.pow(0.5*ourbot_size[0]+0.13, 2) + math.pow(0.5*ourbot_size[1], 2))
  statemsg_cnt = 1./(statemsg_rate*period)
  nghbcom_cnt = 1./(nghbcom_rate*period)
  max_vel_position = motionplanning:getProperty('max_vel_position'):get()
  max_vel_orientation = motionplanning:getProperty('max_vel_orientation'):get()
  if host == 'kurt' then
    peer = 'krist'
  else
    peer = 'kurt'
  end

  -- add ports
  motion_time_port = rtt.InputPort('double')
  host_trajectory_port = rtt.OutputPort('array')
  peer_trajectory_port = rtt.InputPort('array')
  host_priority_port = rtt.OutputPort('bool')
  peer_priority_port = rtt.InputPort('bool')
  tc:addPort(motion_time_port, 'motion_time_port', 'Motion time of computed motion trajectory')
  tc:addPort(host_trajectory_port, 'host_trajectory_port', 'Port to send trajectory of host')
  tc:addPort(peer_trajectory_port, 'peer_trajectory_port', 'Port to receive trajectory of peer')
  tc:addPort(host_priority_port, 'host_priority_port', 'Am I claiming priority?')
  tc:addPort(peer_priority_port, 'peer_priority_port', 'Is my peer claiming priority?')
  motion_time_port:connect(motionplanning:getPort('motion_time_port'))
  if not communicator:addOutgoingConnection('coordinator', 'host_trajectory_port', 'trajectory_'..host, peer) then rfsm.send_events(fsm, 'e_failed') return false end
  if not communicator:addIncomingConnection('coordinator', 'peer_trajectory_port', 'trajectory_'..peer) then rfsm.send_events(fsm, 'e_failed') return false end
  if not communicator:addOutgoingConnection('coordinator', 'host_priority_port', 'priority_'..host, peer) then rfsm.send_events(fsm, 'e_failed') return false end
  if not communicator:addIncomingConnection('coordinator', 'peer_priority_port', 'priority_'..peer) then rfsm.send_events(fsm, 'e_failed') return false end
  -- join group
  if not communicator:joinGroup(host .. '_flex') then rfsm.send_events(fsm, 'e_failed') return false end
  -- init robot tables
  init_robot_tables()
  return true
end

local release = function()
  communicator:removeConnection('coordinator', 'host_trajectory_port', 'trajectory_'..host)
  communicator:removeConnection('coordinator', 'peer_trajectory_port', 'trajectory_'..peer)
  communicator:removeConnection('coordinator', 'host_priority_port', 'priority_'..host)
  communicator:removeConnection('coordinator', 'peer_priority_port', 'priority_'..peer)
end

function flex_update(fsm, control)
  -- send state to coordinator
  if control then
    send_state('busy')
  else
    send_state('idle')
  end
  -- send trajectory to neighbor
  communicate_trajectory(control)
  -- check if current task was canceled
  if current_task_canceled then
    rfsm.send_events(fsm, 'e_idle')
    current_task_canceled = false
  end
  -- update eta
  if control then
    local fs, motion_time = motion_time_port:read()
    if fs == 'NewData' then
      local eta = get_sec() + motion_time
      if math.abs(current_eta - eta) >= 1 then
        send_task_status(current_task, 'ETA changed', eta)
      end
      current_eta = eta
    end
  end
  -- check mail
  check_mail()
end

local load_obstacles_fun = function ()
  reset_obstacles()
  -- add robot tables
  for k=0, n_tables - 1 do
    local robot_pose = get_table_pose(k)
    local robot_table_size = rtt.Variable('array')
    robot_table_size:resize(2)
    robot_table_size:fromtab{robot_tables[k].length, robot_tables[k].width}
    add_static_rect_obstacle(robot_pose, robot_table_size)
  -- print('robot pose: (' ..robot_pose[0]..','..robot_pose[1]..','..robot_pose[2]..')')
  end
  -- add peer
  local fs, peer_coeffs = peer_trajectory_port:read()
  local n_coeffs = get_basis_length()
  local coeffs = rtt.Variable('array')
  coeffs:resize(2*n_coeffs)
  if fs ~= 'NewData' then
    print('No trajectory received from peer, using robot arm position!')
    local robot_pose = get_table_pose(0)
    for k=0, n_coeffs-1 do
      coeffs[k] = robot_pose[0]
      coeffs[k+n_coeffs] = robot_pose[1]
    end
  else
    coeffs = peer_coeffs
  end
  add_peer_circ_obstacle(coeffs, ourbot_radius)
end

function next_task()
  if table.getn(task_queue) > 0 then
    current_task = task_queue[1]
    print('executing task ' .. current_task.task_uuid)
  else
    current_task = nil
  end
  return current_task
end

function check_mail()
  local ret = read_mail(false)
  while (ret.size == 2) do
    -- decode message
    local msg = ret[0]
    local peer = ret[1]
    print(peer)
    if peer == peer_uuid(coordinator_name) then
      print(msg)
      local status, msg_tbl = pcall(json.decode, msg)
      if status then
        if msg_tbl.header.version == header_version then
          local msg_type = msg_tbl.header.type
          if msg_type == 'task_request' then task_bid(msg_tbl.payload, peer)
          elseif msg_type == 'execute' then add_task(msg_tbl.payload)
          elseif msg_type == 'allocate' then allocate_task(msg_tbl.payload)
          elseif msg_type == 'cancel' then cancel_task(msg_tbl.payload)
          end
        end
      else
        print('decoding failed')
      end
      remove_mail()
      break -- read only 1 valid message per period
    end
    ret = read_mail(false)
  end
end

function task_bid(task, peer)
  print('bidding for task ' .. task.task_uuid)
  -- compute bid based on eta
  local total_time = 0
  for k, time in pairs(time_tbl) do
      if k == 1 and current_task ~= nil and current_eta ~= nil then
          total_time = total_time + current_eta - get_sec()  -- use latest motion time guess of current task
      else
          total_time = total_time + time
      end
      total_time = total_time + 5.0 -- estimated wait time between tasks
  end
  local nt = table.getn(task_queue)
  local _, pose = est_pose_port:read()
  local start_pose = (nt == 0) and pose or get_target_pose(task_queue[nt])
  local new_target = get_target_pose(task)
  if new_target == nil then
      total_time = 9999999999
  else
      total_time = total_time + motion_time_guess(start_pose, get_target_pose(task))
  end
  -- if os.time()+total_time > decode_time(task.finished_by) then
  --     total_time = 9999999999
  -- end
  -- send bid message
  msg_tbl.header.type = 'bid'
  msg_tbl.header.timestamp = encode_time(get_sec())
  msg_tbl.payload = {task_uuid = task.task_uuid, task_position = {new_target[0], new_target[1], new_target[2]}, bid = total_time}
  print('bid: ' .. tostring(total_time))
  write_mail(json.encode(msg_tbl), peer)
end

function add_task(task)
  print('adding task ' .. task.task_uuid)
  local target_pose = get_target_pose(task)
  if target_pose == nil then
      send_task_status(task, 'rejected', get_sec())
      return
  end
  table.insert(task_queue, task)
  local nt = table.getn(time_tbl)
  local _, pose = est_pose_port:read()
  local start_pose = (nt == 0) and pose or get_target_pose(task_queue[nt])
  local motiontime_guess = motion_time_guess(start_pose, target_pose)
  table.insert(time_tbl, motiontime_guess)
  send_task_status(task, 'received', get_sec() + motiontime_guess)
end

function allocate_task(task)
  print('task allocation not implemented.')
end

function cancel_task(task)
  print('canceling task ' .. task.task_uuid)
  if current_task ~= nil then
    if task.task_uuid == current_task.task_uuid then
      current_task_canceled = true
      send_task_status(task, 'canceled', current_eta)
    else
      send_task_status(task, 'canceled', get_sec())
    end
  end
  remove_task(task)
end

function remove_task(task)
  print('removing task ' .. task.task_uuid)
  if current_task ~= nil and task.task_uuid == current_task.task_uuid then
    current_eta = nil
    current_task = nil
  end
  for k, task in pairs(task_queue) do
    if task.task_uuid == task.task_uuid then
      table.remove(task_queue, k)
      table.remove(time_tbl, k)
      return
    end
  end
  print('task uuid not known!')
end

function send_task_status(task, status, eta)
  msg_tbl.header.type = 'task_status'
  msg_tbl.header.timestamp = encode_time(get_sec())
  msg_tbl.payload = {
    task_uuid = task.task_uuid,
    status = status,
    eta = encode_time(eta)
  }
  local coordinator_uuid = peer_uuid(coordinator_name)
  if coordinator_uuid ~= '' then
    write_mail(json.encode(msg_tbl), coordinator_uuid)
  end
end

function send_state(state)
  if statemsg_cnt >= 1./(statemsg_rate*period) then
    local _, pose = est_pose_port:read()
    msg_tbl.header.type = 'state'
    msg_tbl.header.timestamp = encode_time(get_sec())
    msg_tbl.payload = {
      state = state,
      features = {x = pose[0], y = pose[1], theta = pose[2]}
    }
    local coordinator_uuid = peer_uuid(coordinator_name)
    if coordinator_uuid ~= '' then
      write_mail(json.encode(msg_tbl), coordinator_uuid)
    end
    statemsg_cnt = 1
  else
    statemsg_cnt = statemsg_cnt + 1
  end
end

function communicate_trajectory(control)
  if nghbcom_cnt >= 1./(nghbcom_rate*period) then
    determine_priority(control)
    local coeffs = rtt.Variable('array')
    local n_coeffs = get_basis_length()
    coeffs:resize(2*n_coeffs)
    if not control then -- send current position
      local _, pose = est_pose_port:read()
      for k=0, n_coeffs-1 do
        coeffs[k] = pose[0]
        coeffs[k+n_coeffs] = pose[1]
      end
    else
      if priority then -- send trajectory
        coeffs = get_coeffs()
      else -- send robot position (i.e. send nothing)
        for k=0, n_coeffs-1 do
        local robot_pose = get_table_pose(0)
          coeffs[k] = robot_pose[0]
          coeffs[k+n_coeffs] = robot_pose[1]
        end
      end
    end
    host_trajectory_port:write(coeffs)
    nghbcom_cnt = 1
  else
    nghbcom_cnt = nghbcom_cnt + 1
  end
end

function determine_priority(control)
  local priority_old = priority
  local fs, peer_priority = peer_priority_port:read()
  -- resolve priority collisions
  if priority and peer_priority then
    rtt.logl('Warning', 'Priority collision occured!')
    if host == 'kurt' then
      priority = false
    end
  end
  if control then
    priority = not peer_priority
  else
    priority = false
  end
  if priority ~= priority_old then -- event-based sending
    host_priority_port:write(priority)
    if priority then
      rtt.logl('Warning', 'I take priority')
    else
      rtt.logl('Warning', 'I release priority')
    end
  end
end

function get_target_pose(task)
  local pose = rtt.Variable('array')
  pose:resize(3)
  if task.task_parameter_key == 'null' then
    for k=0,2 do
      pose[k] = task.task_parameters[k+1]
    end
    return pose
  else
    local key = task.task_parameter_key
    if key == 'robot0' then
      local rp = get_table_pose(0)
      local dx = -0.85
      local dy = 0.
      local dt = math.pi/2
      pose[0] = rp[0] + dx*math.cos(rp[2]) - dy*math.sin(rp[2])
      pose[1] = rp[1] + dx*math.sin(rp[2]) + dy*math.cos(rp[2])
      pose[2] = rp[2] + dt
      return pose
    elseif key == 'robot1' then
      local rp = get_table_pose(1)
      local dx = -1.0
      local dy = 0.
      local dt = -math.pi/2
      pose[0] = rp[0] + dx*math.cos(rp[2]) - dy*math.sin(rp[2])
      pose[1] = rp[1] + dx*math.sin(rp[2]) + dy*math.cos(rp[2])
      pose[2] = rp[2] + dt
      return pose
    end
  end
  rtt.logl('Warning', 'Target not recognized!')
  return nil
end

function get_table_pose(nr)
  local robot_pose = get_robot_pose(robot_tables[nr].id)
  local ts = get_robot_timestamp(robot_tables[nr].id)
  -- robot table still visible?
  if ts == ts_prev or ts < 0 then
    robot_tables[nr].no_robot_cnt = robot_tables[nr].no_robot_cnt + 1
    if (robot_tables[nr].no_robot_cnt >= no_robot_cnt_max) then
      robot_tables[nr].no_robot_cnt = no_robot_cnt_max
      rtt.logl('Warning', 'Robot table not detected! Putting it far away!')
      robot_pose:fromtab{-100, -100, 0}
      return robot_pose
    end
  else
    robot_tables[nr].no_robot_cnt = 0
  end
  return robot_pose
end

function motion_time_guess(start, target)
  local dist = math.sqrt(math.pow(target[0]-start[0], 2) + math.pow(target[1]-start[1], 2))
  local position_time = dist/(0.87*max_vel_position) -- 0.87 when 3th deg spline with 10 knot intervals
  local target_or = target[2] + math.floor(start[2]/(2*math.pi))*2*math.pi
  if (target_or - start[2]) > math.pi then
    target_or = target_or - 2*math.pi
  end
  if (target_or - start[2]) < -math.pi then
    target_or = target_or + 2*math.pi
  end
  local orientation_time = math.abs(target_or-start[2])/max_vel_orientation
  return position_time
  -- return math.max(position_time, orientation_time)
end

function encode_time(time)
  if time == nil then
      time = get_sec()
  end
  local str = string.format("%.3f", time)
  local time_string = os.date("!%FT%H:%M:%S", time) .. "." .. str:sub(#str-2,#str) .. "Z"
  return time_string
end

function decode_time(time_string)
  local time_format = "(%d+)-(%d+)-(%d+)T(%d+):(%d+):(%d+).(%d+)Z"
  local year, month, day, hour, min, sec, msec = time_string:match(time_format)
  return os.time({year=year, month=month, day=day, hour=hour, min=min, sec=sec+msec*1e-3})
end

-- finite state machine
local flex_fsm = rfsm.load('Coordinator/motionplanning_fsm.lua')

flex_fsm.init = rfsm.state{
  entry = function(fsm)
    if not deployer:loadComponent('motionplanning', 'FlexonomyMotionPlanning') then
      rfsm.send_events(fsm, 'e_failed')
    end
    if not set_motionplanner(deployer:getPeer('motionplanning'), load_obstacles_fun) then
      rfsm.send_events(fsm, 'e_failed')
      return
    end
    if not initialize() then
      rfsm.send_events(fsm, 'e_failed')
    end
    if not start_control_components() then
      rfsm.send_events(fsm, 'e_failed')
    end
  end
}

flex_fsm.idle = rfsm.state{
  entry = function(fsm)
    zero_velocity()
    enable_manualcommand()

    if current_task ~= nil and not mp_ready() then
      send_task_status(current_task, 'failed', current_eta)
      remove_task(current_task)
    end

    state = 'ready'
  end,

  doo = function(fsm)
    while true do
      flex_update(fsm, false)
      if not control_hook(false) or motionplanning_error() then
        rfsm.send_events(fsm, 'e_failed')
      end
      -- check for new task
      local task = next_task()
      if task ~= nil then
        mp_reset()
        local target_pose = get_target_pose(task)
        motionplanning:setTargetPose(target_pose)
        rfsm.send_events(fsm, 'e_p2p')
      end
      rfsm.yield(true)
    end
  end,

  exit = function(fsm)
    state = 'busy'
  end,
}

flex_fsm.p2p0 = rfsm.state{
  doo = function(fsm)
    p2p0_init(fsm)
    while true do
      flex_update(fsm, false)
      p2p0_hook(fsm)
      rfsm.yield(true)
    end
  end,

  exit = function(fsm)
    if not mp_valid() then
      send_task_status(current_task, 'rejected', get_sec())
      remove_task(current_task)
      return
    elseif not current_task.started then
      local fs, motion_time = motion_time_port:read()
      current_eta = get_sec() + motion_time
      send_task_status(current_task, 'started', current_eta)
      current_task.started = true
    end
    if mp_ready() then
      send_task_status(current_task, 'finished', get_sec())
      remove_task(current_task)
    end
  end,
}

flex_fsm.p2p = rfsm.state{
  doo = function(fsm)
    while true do
      flex_update(fsm, true)
      p2p_hook(fsm)
      rfsm.yield(true)
    end
  end,

  exit = function(fsm)
    if mp_ready() then
      send_task_status(current_task, 'finished', get_sec())
      remove_task(current_task)
      return
    end
  end,
}

flex_fsm.recover = rfsm.state{
  doo = function (fsm)
    recover_init(fsm)
    local time0 = get_sec()
    while true do
      flex_update(fsm, false)
      recover_hook(fsm, time0)
      rfsm.yield(true)
    end
  end,
}

flex_fsm.stop.exit = function(fsm)
  release()
end

flex_fsm.failure.exit = function(fsm)
  release()
end

return flex_fsm
