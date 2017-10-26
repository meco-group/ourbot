require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'

-- components to load
local components_to_load = {
  coordinator = 'OCL::LuaTLSFComponent',
  communicator = 'Communicator',
  estimator = 'Kalman',
  controller = 'PoseController',
  reference = 'Reference',
  reporter = 'OCL::NetcdfReporting',
  io = 'Container',
  teensy = 'TeensyBridge'
}

-- containers to fill
local containers_to_fill = {
  io = {'teensy'}
}

-- ports to report
local ports_to_report = {
  -- estimator = {'est_pose_port'}
  reference = {'ref_pose_port', 'ref_velocity_port'}
  -- teensy = {'cmd_velocity_passthrough_port'}
}

-- packages to import
local packages_to_import = {'Communicator', 'Estimator', 'Controller',
'Reference', 'MotionPlanning', 'Container','Serial'}


-- configuration files to load
local system_config_file = 'Configuration/system-config.cpf'
local component_config_files = {
  estimator = 'Configuration/estimator-config.cpf',
  communicator = 'Configuration/communicator-config.cpf',
  controller = 'Configuration/controller-config.cpf',
  reference = 'Configuration/reference-config.cpf',
  reporter = 'Configuration/reporter-config.cpf',
  teensy = 'Configuration/teensy-config.cpf',
  lidar = 'Configuration/lidar-config.cpf',
  scanmatcher = 'Configuration/scanmatcher-config.cpf'
}

local components = {}
local dp = rtt.getTC():getPeer('Deployer')

return rfsm.state {

  rfsm.transition {src = 'initial', tgt = 'deploy'},
  rfsm.transition {src = 'deploy', tgt = 'failure',  events = {'e_failed'}},
  rfsm.transition {src = 'deployed',  tgt = 'failure',  events = {'e_failed'}},
  rfsm.transition {src = '.deploy.load_coordinator', tgt = 'deployed', events = {'e_done'}},
  rfsm.transition {src = 'failure', tgt = 'deploy', events = {'e_reset'}},

  initial = rfsm.conn{},
  deployed = rfsm.conn{},

  failure = rfsm.state {
    entry = function()
      _deployer_failure_event_port:write('e_failed')
      components.communicator:update()
    end,

    doo = function()
      local cnt = 0
      while (cnt <= 3) do
        cnt = cnt+1
      end
      dp:stopComponents()
    end,
   },

  deploy = rfsm.state {
    rfsm.transition {src = 'initial', tgt = 'load_components'},
    rfsm.transition {src = 'load_components', tgt = 'configure_components', events = {'e_done'}},
    rfsm.transition {src = 'configure_components', tgt = 'connect_components', events = {'e_done'}},
    rfsm.transition {src = 'connect_components', tgt = 'connect_remote_components', events = {'e_done'}},
    rfsm.transition {src = 'connect_remote_components', tgt = 'set_activities', events = {'e_done'}},
    rfsm.transition {src = 'set_activities', tgt = 'prepare_reporter', events = {'e_done'}},
    rfsm.transition {src = 'prepare_reporter', tgt = 'load_coordinator', events = {'e_done'}},

    initial = rfsm.conn{},

    load_components = rfsm.state {
      entry = function(fsm)
        components = {}
        -- import necessary packages
        for _, package in ipairs(packages_to_import) do
          if not rtt.provides("ros"):import(package) then rfsm.send_events(fsm,'e_failed') return end
        end
        -- go through the table of components and load them
        for name, type in pairs(components_to_load) do
          if not dp:loadComponent(name, type) then rfsm.send_events(fsm,'e_failed') return end
          components[name] = dp:getPeer(name)
        end
        -- fill containers with correct components
        for container, comps in pairs(containers_to_fill) do
          local addToContainer = components[container]:getOperation('addComponent')
          for i, name in pairs(comps) do
            dp:addPeer(container, name)
            if not (name == imul or name == imur) then
              addToContainer(name, '')
            elseif name == imul then
              addToContainer(name, 'imul_')
            elseif name == imur then
              addToContainer(name, 'imur_')
            end
          end
        end
        -- load execution file in coordinator component
        if not components.coordinator:exec_file(coordinator_file) then rfsm.send_events(fsm,'e_failed') return end
      end
    },

    configure_components = rfsm.state {
      entry = function(fsm)
        for name, comp in pairs(components) do
          if (name ~= 'reporter' and name ~= 'coordinator') then -- reporter and coordinator configured in later stage
            comp:loadService('marshalling')
            -- every component loads system configurations
            if not comp:provides('marshalling'):updateProperties(system_config_file) then rfsm.send_events(fsm,'e_failed') return end
            -- if available, a component loads its specific configurations
            if component_config_files[name] then
              if not comp:provides('marshalling'):updateProperties(component_config_files[name]) then rfsm.send_events(fsm,'e_failed') return end
            end
            -- configure the component
            if not comp:configure() then rfsm.send_events(fsm,'e_failed') return end
          end
        end
      end
    },

    connect_components = rfsm.state {
      entry = function(fsm)
        -- connect teensy to lidar
        if components_to_load['teensy'] and components_to_load[lidar] then
          if not dp:connectPorts('teensy', 'lidar') then rfsm.send_events(fsm,'e_failed') return end
        end
        -- connect scanmatcher
        if components_to_load['scanmatcher'] then
          if not dp:connectPorts('estimator', 'scanmatcher') then rfsm.send_events(fsm,'e_failed') return end
          if not dp:connectPorts('scanmatcher', 'io') then rfsm.send_events(fsm,'e_failed') return end
        end
        -- connect other components
        if not dp:connectPorts('estimator', 'io') then rfsm.send_events(fsm,'e_failed') return end
        if not dp:connectPorts('estimator', 'controller') then rfsm.send_events(fsm,'e_failed') return end
        if not dp:connectPorts('reference', 'controller') then rfsm.send_events(fsm,'e_failed') return end
        if not dp:connectPorts('estimator', 'reference') then rfsm.send_events(fsm,'e_failed') return end
        if not dp:connectPorts('io', 'controller') then rfsm.send_events(fsm,'e_failed') return end
          --add more connections here

        -- connect the deployer to coordinator (for communicating failure events)
        if not _deployer_fsm_event_port:connect(components.coordinator:getPort('coordinator_failure_event_port')) then rfsm.send_events(fsm,'e_failed') return end
        if not _deployer_failure_event_port:connect(components.coordinator:getPort('coordinator_fsm_event_port')) then rfsm.send_events(fsm,'e_failed') return end
        -- add every component as peer of coordinator
        for name, comp in pairs(components) do
          if (name ~= 'coordinator') then
            if not dp:addPeer('coordinator', name) then rfsm.send_events(fsm, 'e_failed') return end
          end
        end
        if not dp:addPeer('coordinator', 'Deployer') then rfsm.send_events(fsm, 'e_failed') return end
      end
    },

    connect_remote_components = rfsm.state{
      entry = function(fsm)
        -- add every component as peer of communicator
        for name, comp in pairs(components) do
          if (name ~= 'communicator') then
            if not dp:addPeer('communicator', name) then rfsm.send_events(fsm, 'e_failed') return end
          end
        end
        if not dp:addPeer('communicator', 'lua') then rfsm.send_events(fsm,'e_failed') return end
        -- load operations
        local addIncoming = components.communicator:getOperation('addIncomingConnection')
        local addBufferedIncoming = components.communicator:getOperation('addBufferedIncomingConnection')
        local addOutgoing = components.communicator:getOperation('addOutgoingConnection')
        local setRate = components.communicator:getOperation('setConnectionRate')
        -- coordinator
        if not addOutgoing('coordinator', 'coordinator_send_event_port', 'fsm_event', 'emperor') then rfsm.send_events(fsm, 'e_failed') return end
        if not addIncoming('coordinator', 'coordinator_fsm_event_port', 'fsm_event') then rfsm.send_events(fsm, 'e_failed') return end
        -- estimator
        if not addIncoming('estimator', 'markers_port', 'markers_'..host) then rfsm.send_events(fsm, 'e_failed') return end
        -- io
        if not addIncoming('io', 'cmd_velocity_port', 'cmd_velocity') then rfsm.send_events(fsm, 'e_failed') return end
        -- estimator
        -- if not addOutgoing('estimator', 'est_pose_port', 'est_pose_'..host, 'emperor') then rfsm.send_events(fsm, 'e_failed') return end
        -- if not setRate('estimator', 'est_pose_port', 'est_pose_'..host, 20, control_rate) then rfsm.send_events(fsm, 'e_failed') return end
        -- reference
        if not addOutgoing('reference', 'ref_position_trajectory_x_port', 'ref_x_'..host, 'emperor') then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('reference', 'ref_position_trajectory_y_port', 'ref_y_'..host, 'emperor') then rfsm.send_events(fsm, 'e_failed') return end
        -- motion planning
        -- if not obstacle_mode then
        --   if not addIncoming('motionplanning', 'robobs_pose_port', 'robobs_pose') then rfsm.send_events(fsm, 'e_failed') return end
        --   if not addIncoming('motionplanning', 'robobs_velocity_port', 'robobs_velocity') then rfsm.send_events(fsm, 'e_failed') return end
        -- end
        -- if not addIncoming('distrmotionplanning', 'obstacle_port', 'obstacles') then rfsm.send_events(fsm, 'e_failed') return end
        -- if not addIncoming('distrmotionplanning', 'target_pose_port', 'target_pose') then rfsm.send_events(fsm, 'e_failed') return end
        -- if not obstacle_mode then
        --   if not addIncoming('distrmotionplanning', 'robobs_pose_port', 'robobs_pose') then rfsm.send_events(fsm, 'e_failed') return end
        --   if not addIncoming('distrmotionplanning', 'robobs_velocity_port', 'robobs_velocity') then rfsm.send_events(fsm, 'e_failed') return end
        -- end
        -- if not addOutgoing('distrmotionplanning', 'negotiate_out_port', 'negotiate', 'ourbots') then rfsm.send_events(fsm, 'e_failed') return end
        -- if not addBufferedIncoming('distrmotionplanning', 'negotiate_in_port', 'negotiate', 10) then rfsm.send_events(fsm, 'e_failed') return end
        -- if not addIncoming('distrmotionplanning', 'negotiate_in_port', 'negotiate') then rfsm.send_events(fsm, 'e_failed') return end
        -- end
        -- deployer
        if not addOutgoing('lua', 'deployer_failure_event_port', 'deployer_event', 'all') then rfsm.send_events(fsm,'e_failed') return end
        if not addIncoming('lua', 'deployer_fsm_event_port', 'deployer_event') then rfsm.send_events(fsm, 'e_failed') return end
      end
    },

    set_activities = rfsm.state {
      entry = function(fsm)
        dp:setActivity('coordinator', 1./control_rate, 0, rtt.globals.ORO_SCHED_OTHER)
        if components_to_load['scanmatcher'] then
          dp:setActivity('scanmatcher', 0, 0, rtt.globals.ORO_SCHED_OTHER)
        end
          --add here extra activities

        -- the following components are triggered by the coordinator and executed in the same thread.
        dp:setMasterSlaveActivity('coordinator', 'io')
        dp:setMasterSlaveActivity('coordinator', 'estimator')
        dp:setMasterSlaveActivity('coordinator', 'controller')
        dp:setMasterSlaveActivity('coordinator', 'reference')
        dp:setMasterSlaveActivity('coordinator', 'communicator')
        -- all elements of a container are triggered by the container
        for container, comps in pairs(containers_to_fill) do
          for i,name in pairs(comps) do
            dp:setMasterSlaveActivity(container, name)
          end
        end
      end
    },

    prepare_reporter = rfsm.state {
      entry = function(fsm)
        -- load the mashalling service and the configuration file
        components.reporter:loadService('marshalling')
        components.reporter:provides('marshalling'):loadProperties(component_config_files['reporter'])
        -- add components to report
        for comp, portlist in pairs(ports_to_report) do
          for i, port in pairs(portlist) do
            if not dp:addPeer('reporter', comp) then rfsm.send_events(fsm,'e_failed') end
            if not components.reporter:reportPort(comp, port) then rfsm.send_events(fsm,'e_failed') end
          end
        end
        -- configure the reporter
        if not components.reporter:configure() then rfsm.send_events(fsm, 'e_failed') return end
      end
    },

    load_coordinator = rfsm.state {
      entry = function(fsm)
        -- coordinator loads system configurations
        components.coordinator:loadService('marshalling')
        if not components.coordinator:provides('marshalling'):updateProperties(system_config_file) then rfsm.send_events(fsm,'e_failed') return end
        -- configure the coordinator
        if not components.coordinator:configure() then rfsm.send_events(fsm,'e_failed') return end
        -- load the local application script
        components.coordinator:loadService("scripting")
        if not components.coordinator:provides("scripting"):loadPrograms(app_file) then rfsm.send_events(fsm,'e_failed') return end
        -- start the coordinator and communicator
        if not components.coordinator:start() then rfsm.send_events(fsm,'e_failed') return end
        if not components.communicator:start() then rfsm.send_events(fsm,'e_failed') return end
      end
    },
  }
}

