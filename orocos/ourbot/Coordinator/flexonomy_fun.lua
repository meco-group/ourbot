-- local json = require "./Coordinator/json" -- this is buggy (I think)
local json = require "cjson"
local tc = rtt.getTC()

local motionplanning      = tc:getPeer('motionplanning')
local controller          = tc:getPeer('controller')
local estimator           = tc:getPeer('estimator')
local reference           = tc:getPeer('reference')
local reporter            = tc:getPeer('reporter')
local io                  = tc:getPeer('io')
local teensy              = tc:getPeer('teensy')
local communicator        = tc:getPeer('communicator')

-- operations
local estimatorUpdate = estimator:getOperation("update")
local referenceUpdate = reference:getOperation("update")
local controllerUpdate = controller:getOperation("update")
local validEstimation = estimator:getOperation("validEstimation")
local validReference = reference:getOperation("ready")
local estimatorInRunTimeError = estimator:getOperation("inRunTimeError")
local controllerInRunTimeError = controller:getOperation("inRunTimeError")
local referenceInRunTimeError = reference:getOperation("inRunTimeError")
local snapshot = reporter:getOperation("snapshot")
local getPeerUUID = communicator:getOperation('getPeerUUID')
local readMail = communicator:getOperation('readMail')
local removeMail = communicator:getOperation('removeMail')
local writeMail = communicator:getOperation('writeMail')
local getPose = estimator:getOperation('getEstimatedPose')
local mpBusy = motionplanning:getOperation('gotTarget')
local targetReached = motionplanning:getOperation('targetReached')
local hostObstTraj = motionplanning:getOperation("writeHostObstTraj")

-- properties
local header_version = '1.0.0'
local statemsg_sample_rate = 1
local nghbcom_sample_rate = 4
local coordinator_name = 'SH1'
local vmax = 0.8

-- ports
local _motion_time_port = rtt.InputPort("double")
tc:addPort(_motion_time_port, "motion_time_port", "Motion time of computed motion trajectory")
_motion_time_port:connect(motionplanning:getPort('motion_time_port'))
local _cmd_velocity_port = rtt.OutputPort("array")
tc:addPort(_cmd_velocity_port, "cmd_velocity_port", "Input port for low level velocity controller. Vector contains [vx,vy,w]")
_cmd_velocity_port:connect(teensy:getPort('cmd_velocity_port'))

-- wireless ports
local addIncoming = communicator:getOperation('addIncomingConnection')
local addOutgoing = communicator:getOperation('addOutgoingConnection')
if not addIncoming('motionplanning', 'obstacle_trajectory_port', 'obstacle_trajectory') then rfsm.send_events(fsm, 'e_failed') return end
if host == 'dave' then
    if not addOutgoing('motionplanning', 'host_obstacle_trajectory_port', 'obstacle_trajectory', 'kurt') then rfsm.send_events(fsm, 'e_failed') return end
else
    if not addOutgoing('motionplanning', 'host_obstacle_trajectory_port', 'obstacle_trajectory', 'dave') then rfsm.send_events(fsm, 'e_failed') return end
end

-- global vars
current_task = nil
current_eta = nil
current_task_canceled = false

local task_time = {}
local task_queue = {}
local inf = 999999999
local period = tc:getPeriod()
local max_snap_cnt = 1/(reporter_sample_rate*period)
local snap_cnt = max_snap_cnt
local max_statemsg_cnt = 1/(statemsg_sample_rate*period)
local statemsg_cnt = max_statemsg_cnt
local max_nghbcom_cnt = 1/(nghbcom_sample_rate*period)
local nghbcom_cnt = max_nghbcom_cnt
local failure_cnt = 0
local max_failure_cnt = 5
local zero_cmd = rtt.Variable('array')
zero_cmd:resize(3)
zero_cmd:fromtab{0, 0, 0}

moving = false

local msg_tbl = {
    header = {
        version     = header_version,
        type        = '',
        model       = '',
        uuid        = communicator:getUUID(),
        timestamp   = ''
    },
    payload = {}
}

function init(fsm)
    reference:setMotionPlanner("motionplanning")
    if not io:start() then
        rtt.logl("Error","Could not start io component")
        rfsm.send_events(fsm,'e_failed')
        return
    end
    if not motionplanning:start() then
        rtt.logl("Error","Could not start motionplanning component")
        rfsm.send_events(fsm,'e_failed')
        return
    end
    if not reporter:start() then
        rtt.logl("Error","Could not start reporter component")
        rfsm.send_events(fsm,'e_failed')
        return
    end
    if not reference:start() or not estimator:start() or not controller:start() then
        rtt.logl("Error","Could not start reference/estimator/controller component")
        rfsm.send_events(fsm,'e_failed')
        return
    end
    teensy:strongVelocityControl()
    -- init counters
    snap_cnt = max_snap_cnt
    statemsg_cnt = max_statemsg_cnt
    nghbcom_cnt = max_nghbcom_cnt
end

function stop()
    mp:stop()
    estimator:stop()
    reference:stop()
    controller:stop()
    reporter:stop()
    io:stop()
end

function waitForValidEstimate(fsm)
    while true do
        estimatorUpdate()
        if validEstimation() then
            return
        else
            print "Estimate not valid!"
        end
        if estimatorInRunTimeError() then
            rtt.logl("Error","RunTimeError in estimator component")
            rfsm.send_events(fsm,'e_failed')
            return
        end
        rfsm.yield(true)
    end
end

function home(fsm)
    -- local pose0 = motionplanning:setConfiguration()
    local pose0 = getPose()
    motionplanning:setTargetPose(pose0[0], pose0[1], pose0[2])
    while true do
        -- check if homing is finished
        if not mpBusy() then
            if targetReached() then -- succesful
                controlLoop(fsm, false)
                return
            else -- unsuccesful
                rtt.log("Error", "Could not home!")
                rfsm.send_events(fsm, 'e_failed')
                controlLoop(fsm, false)
                return
            end
        end
        if not controlLoop(fsm, true) then
            return
        end
        rfsm.yield(true) -- sleep until next sample
    end
end

function compute_distance(start,target)
    local dist = math.sqrt(math.pow(target[1]-start[1], 2) + math.pow(target[2]-start[2], 2))
    return dist
end

function update(fsm, state, control)
    -- control action
    if not controlLoop(fsm, control) then
        return false
    end
    -- take snapshot for logger
    if snap_cnt >= max_snap_cnt then
        snapshot:send()
        snap_cnt = 1
    else
        snap_cnt = snap_cnt + 1
    end
    -- send state messages
    if statemsg_cnt >= max_statemsg_cnt then
        sendState(state)
        statemsg_cnt = 1
    else
        statemsg_cnt = statemsg_cnt + 1
    end
    -- send host obstacle trajectory
    if nghbcom_cnt >= max_nghbcom_cnt then
        nghbcom_cnt = 1
        if not moving then
            hostObstTraj(3)
        else
            if host == 'dave' then
               hostObstTraj(1)
            else
               hostObstTraj(2)
            end
        end
    else
        nghbcom_cnt = nghbcom_cnt + 1
    end
    -- check for new mail
    checkMail()
    return true
end

function controlLoop(fsm, control)
    -- update reference/estimator/controller
    estimatorUpdate()
    if control then
        if validEstimation() then
            referenceUpdate()
            if validReference() then
                controllerUpdate()
            else
                _cmd_velocity_port:write(zero_cmd)
            end
        else
            print "Estimate not valid!"
            _cmd_velocity_port:write(zero_cmd)
        end
    else
        _cmd_velocity_port:write(zero_cmd)
    end
    if controllerInRunTimeError() or estimatorInRunTimeError() or referenceInRunTimeError() then
        rtt.logl("Error","RunTimeError in controller/estimator/reference component")
        rfsm.send_events(fsm,'e_failed')
        return false
    end
    if motionplanning:inRunTimeError() then
        rtt.logl("Error","RunTimeError in motionplanning")
        rfsm.send_events(fsm,'e_failed')
        return false
    end
    return true
end

function checkMail()
    local ret = readMail(false)
    --print "Reading mail..."
    while (ret.size == 2) do
        -- decode message
        local msg = ret[0]
        local peer = ret[1]
        print(peer)
        if peer == getPeerUUID(coordinator_name) then
            print(msg)
            local status, msg_tbl = pcall(json.decode, msg)
            if status then
                if msg_tbl.header.version == header_version then
                    local msg_type = msg_tbl.header.type
                    if msg_type == 'task_request' then bidForTask(msg_tbl.payload, peer)
                    elseif msg_type == 'execute' then addTask(msg_tbl.payload)
                    elseif msg_type == 'allocate' then allocateTask()
                    elseif msg_type == 'cancel' then cancelTask(msg_tbl.payload)
                    end
                end
            else
                print('decoding failed')
            end
            removeMail()
        else
            print('Not my coordinator!')
        end
        ret = readMail(false)
    end
end

function checkMotionPlanning(fsm)
    -- check if task was canceled
    if current_task_canceled then
        rfsm.send_events(fsm, 'e_stop')
        current_task_canceled = false
    end
    -- check if p2p task is finished
    if not mpBusy() then
        if targetReached() then -- succesful
            failure_cnt = 0
            if not moving then -- we were already at our target
                sendTaskStatus(current_task, 'finished', get_sec())
            else
                sendTaskStatus(current_task, 'finished', current_eta)
            end
            print 'motion planning succeeded.'
            removeTask(current_task)
            current_eta = nil
            rfsm.send_events(fsm, 'e_idle')
            return false
        else  -- unsuccesful
            if failure_cnt == 0 and not moving then -- first time
                failure_cnt = 0
                sendTaskStatus(current_task, 'rejected', get_sec())
                removeTask(current_task)
                rfsm.send_events(fsm, 'e_idle')
            elseif failure_cnt <= max_failure_cnt then
                print 'try auto recovering motion planning.'
                failure_cnt = failure_cnt + 1
                rfsm.send_events(fsm, 'e_auto_recover')
            else
                print 'motion planning failed.'
                failure_cnt = 0
                sendTaskStatus(current_task, 'failed', current_eta)
                removeTask(current_task)
                rfsm.send_events(fsm, 'e_idle')
            end
            return false
        end
    end
    -- check if task has started/update ETA
    local fs, motion_time = _motion_time_port:read()
    if fs == 'NewData' then -- new opt problem was solved
        local eta = get_sec() + motion_time
        if not moving then
            current_eta = eta
            sendTaskStatus(current_task, 'started', current_eta)
            moving = true
        elseif math.abs(current_eta-eta) >= 1 then
            current_eta = eta
            sendTaskStatus(current_task, 'ETA changed', current_eta)
        end
    end
    return true
end

function sendState(state)
    local pose = getPose()
    msg_tbl.header.type = 'state'
    msg_tbl.header.timestamp = encodeTime(get_sec())
    msg_tbl.payload = {
        state = state,
        features = {x = pose[0], y = pose[1], theta = pose[2]}
    }
    local coordinator_uuid = getPeerUUID(coordinator_name)
    if coordinator_uuid ~= '' then
        writeMail(json.encode(msg_tbl), coordinator_uuid)
    end
end

function sendTaskStatus(task, status, ETA)
    msg_tbl.header.type = 'task_status'
    msg_tbl.header.timestamp = encodeTime(get_sec())
    msg_tbl.payload = {
        task_uuid = task.task_uuid,
        status = status,
        ETA = encodeTime(ETA)
    }
    local coordinator_uuid = getPeerUUID(coordinator_name)
    if coordinator_uuid ~= '' then
        writeMail(json.encode(msg_tbl), coordinator_uuid)
    end
end

function addTask(task)
    print('adding task '..task.task_uuid)
    local target_pose = getTargetPose(task)
    if target_pose == nil then
        sendTaskStatus(task, 'rejected', get_sec())
        return
    end
    table.insert(task_queue, task)
    local nt = table.getn(task_time)
    local start_pose = (nt == 0) and getPose() or getTargetPose(task_queue[nt])
    local motiontime_guess = getExpectedMotionTime(start_pose, target_pose)
    table.insert(task_time, motiontime_guess)
    sendTaskStatus(task, 'received', get_sec() + motiontime_guess)
end

function allocateTask()
    print('task allocation not implemented.')
end

function removeTask(task)
    print('removing task '..task.task_uuid)
    for k, task in pairs(task_queue) do
        if task.task_uuid == task.task_uuid then
            table.remove(task_queue, k)
            table.remove(task_time, k)
            return
        end
    end
    print ('task uuid not known')
end

function cancelTask(task)
    print('cancelling task '..task.task_uuid)
    removeTask(task)
    if current_task == nil then
        return
    end
    if task.task_uuid == current_task.task_uuid then
        current_task_canceled = true
        sendTaskStatus(task, 'cancelled', current_eta)
    else
        sendTaskStatus(task, 'cancelled', get_sec())
    end
end

function bidForTask(task, peer)
    print('bidding for task '..task.task_uuid)
    -- compute bid based on eta
    local total_time = 0
    for k, time in pairs(task_time) do
        if k == 1 and current_task ~= nil then
            total_time = total_time + current_eta - get_sec()  -- use latest motion time guess of current task
        else
            total_time = total_time + time
        end
    end
    nt = table.getn(task_queue)
    local start_pose = (nt == 0) and getPose() or getTargetPose(task_queue[nt])
    total_time = total_time + getExpectedMotionTime(start_pose, getTargetPose(task))
    if os.time()+total_time > decodeTime(task.finished_by) then
        total_time = inf
    end
    -- send bid message
    msg_tbl.header.type = 'bid'
    msg_tbl.header.timestamp = encodeTime(get_sec())
    msg_tbl.payload = {task_uuid = task.task_uuid, bid = total_time}
    writeMail(json.encode(msg_tbl), peer)
end

function executeNextTask()
    if table.getn(task_queue) > 0 then
        current_task = task_queue[1]
        print('executing task '..current_task.task_uuid)
    else
        current_task = nil
    end
    return current_task
end

function inTaskQueue(task)
    for k, t in pairs(task_queue) do
        if t.task_uuid == task_queue then
            return true
        end
    end
    return false
end

function getExpectedMotionTime(start, target)
    local v_mean = 0.87*vmax -- 0.87 when 3th deg spline with 10 knot intervals
    local dist = math.sqrt(math.pow(target[1]-start[1], 2) + math.pow(target[2]-start[2], 2))
    return dist/v_mean
end

function decodeTime(time_string)
    local time_format = "(%d+)-(%d+)-(%d+)T(%d+):(%d+):(%d+).(%d+)Z"
    local year, month, day, hour, min, sec, msec = time_string:match(time_format)
    return os.time({year=year, month=month, day=day, hour=hour, min=min, sec=sec+msec*1e-3})
end

function encodeTime(time)
    if time == nil then
        time = get_sec()
    end
    local str = string.format("%.3f", time)
    local time_str = os.date("!%FT%H:%M:%S", time) .. "." .. str:sub(#str-2,#str) .. "Z"
    return time_str
end

function getTargetPose(task)
    local zone = task.task_parameters
    if zone == 'A' then
        -- return {2.7, 2.2, 3.141}
        return {3.2, 2, -1.5702}
    elseif zone == 'B' then
        return {4.2, 1.2, 3.1415}
        -- return {3.8, 1.1, -1.5702}
        --return {3.8, 1.1, 0.0}
    elseif zone == 'C' then
        return {3.2, 0.5, 1.5702}
        -- return {1.5, 0.5, 0.0}
    elseif zone == 'D' then
        return {1, 1.2, 0}
        -- return {1.0, 2.0, 0.0}
    else
        print('target zone not recognized')
        return nil
    end
end
