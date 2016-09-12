require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'

-- components to load
local components_to_load = {
  coordinator     = 'OCL::LuaTLSFComponent',
  communicator    = 'Communicator',
  estimator       = estimator_type,
  controller      = controller_type,
  reference       = 'Reference',
  motionplanning  = motionplanning_type,
  reporter        = 'OCL::NetcdfReporting',
  io              = 'Container'
  -- teensy          = 'TeensyBridge'
  -- lidar           = 'RPLidar',
  -- scanmatcher     = 'Scanmatcher'
    --add here componentname = 'componenttype'
}

-- containers to fill
local containers_to_fill = {
  -- io  = {'teensy', 'lidar'}
  -- io = {'teensy'}
  io = {}
}

-- ports to report
local ports_to_report = {
  -- controller      = {'cmd_velocity_port'},
  estimator       = {'est_pose_port'}
  -- reference       = {'ref_velocity_port'}
  -- coordinator     = {'controlloop_duration', 'controlloop_jitter'},
  -- io              = {--'cal_lidar_node_port',
  --                     -- 'cal_imul_transacc_port',
  --                     -- 'cal_imul_orientation_3d_port',
  --                     -- 'cal_imul_orientation_port',
  --                     -- 'cal_imul_dorientation_3d_port',
  --                     -- 'cal_imul_dorientation_port',
  --                     -- 'cal_imur_transacc_port',
  --                     -- 'cal_imur_orientation_3d_port',
  --                     -- 'cal_imur_orientation_port',
  --                     -- 'cal_imur_dorientation_3d_port',
  --                     -- 'cal_imur_dorientation_port'
  --                     -- 'cal_lidar_x_port',
  --                     -- 'cal_lidar_y_port',
  --                     -- 'cal_enc_pose_port'
  --                     -- 'raw_imul_mag_port',
  --                     -- 'raw_imur_mag_port',
  --                     -- 'cal_lidar_global_node_port',
  --                     -- 'cal_motor_current_port',
  --                     -- 'cal_motor_voltage_port',
                      -- 'cal_velocity_port'
                      -- }
    --add here componentname = 'portnames'
}

-- packages to import
local packages_to_import = {
  communicator    = 'Communicator',
  estimator       = 'EstimatorInterface',
  controller      = 'ControllerInterface',
  reference       = 'Reference',
  motionplanning  = 'MotionPlanning',
  io              = 'Container',
  teensy          = 'SerialInterface'
  -- lidar           = 'SerialInterface',
  -- scanmatcher     = 'Scanmatcher'
    --add here componentname = 'parentcomponenttype'
}

-- configuration files to load
local system_config_file      = 'Configuration/system-config.cpf'
local component_config_files  = {
  estimator       = 'Configuration/estimator-config.cpf',
  reporter        = 'Configuration/reporter-config.cpf',
  teensy          = 'Configuration/teensy-config.cpf',
  lidar           = 'Configuration/lidar-config.cpf',
  motionplanning  = 'Configuration/motionplanning-config.cpf',
  scanmatcher     = 'Configuration/scanmatcher-config.cpf'
    --add here componentname = 'Configuration/component-config.cpf'
}

local components = {}
local dp = rtt.getTC():getPeer('Deployer')

return rfsm.state {

  rfsm.transition {src = 'initial',                   tgt = 'deploy'},
  rfsm.transition {src = 'deploy',                    tgt = 'failure',  events = {'e_failed'}},
  rfsm.transition {src = 'deployed',                  tgt = 'failure',  events = {'e_failed'}},
  rfsm.transition {src = '.deploy.load_coordinator',  tgt = 'deployed', events = {'e_done'}},
  rfsm.transition {src = 'failure',                   tgt = 'deploy',   events = {'e_reset'}},

  initial = rfsm.conn{},

  deployed = rfsm.conn{},

  failure = rfsm.state {
    entry = function()
      _deployer_failure_event_port:write('e_failed')
      components.communicator:update()
      for name, comp in pairs(components) do
        comp:stop()
        comp:cleanup()
      end
    end,
    exit = function()
      for name, type in pairs(components_to_load) do
        dp:unloadComponent(name)
      end
    end,
   },

  deploy = rfsm.state {
    rfsm.transition {src = 'initial',                   tgt = 'load_components'},
    rfsm.transition {src = 'load_components',           tgt = 'connect_components',         events = {'e_done'}},
    rfsm.transition {src = 'connect_components',        tgt = 'connect_remote_components',  events = {'e_done'}},
    rfsm.transition {src = 'connect_remote_components', tgt = 'configure_components',       events = {'e_done'}},
    rfsm.transition {src = 'configure_components',      tgt = 'set_activities',             events = {'e_done'}},
    rfsm.transition {src = 'set_activities',            tgt = 'prepare_reporter',           events = {'e_done'}},
    rfsm.transition {src = 'prepare_reporter',          tgt = 'load_coordinator',           events = {'e_done'}},

    load_components = rfsm.state {
      entry = function(fsm)
        components = {}
        -- import necessary packages
        for name, type in pairs(packages_to_import) do
          if not rtt.provides("ros"):import(type) then rfsm.send_events(fsm,'e_failed') return end
        end
        -- go through the table of components to load them
        for name, type in pairs(components_to_load) do
          if not dp:loadComponent(name, type) then rfsm.send_events(fsm,'e_failed') return end
          components[name] = dp:getPeer(name)
        end
        -- fill containers with correct components
        for container, comps in pairs(containers_to_fill) do
          local addToContainer = components[container]:getOperation("addComponent")
          for i, name in pairs(comps) do
            dp:addPeer(container, name)
            if not (name == imul or name == imur) then
              addToContainer(name,"")
            elseif name == imul then
              addToContainer(name,"imul_")
            elseif name == imur then
              addToContainer(name,"imur_")
            end
          end
        end
        -- load execution file in coordinator component
        if not components.coordinator:exec_file(coordinator_file) then rfsm.send_events(fsm,'e_failed') return end
      end
    },


    connect_components = rfsm.state {
      entry = function(fsm)
        -- connect teensy to lidar
        if components_to_load['teensy'] and components_to_load[lidar] then
          if not dp:connectPorts('teensy', 'lidar')            then rfsm.send_events(fsm,'e_failed') return end
        end
        -- connect scanmatcher
        if components_to_load['scanmatcher'] then
          if not dp:connectPorts('estimator', 'scanmatcher')     then rfsm.send_events(fsm,'e_failed') return end
          if not dp:connectPorts('scanmatcher', 'io')            then rfsm.send_events(fsm,'e_failed') return end
        end
        -- connect other components
        if not dp:connectPorts('estimator', 'io')              then rfsm.send_events(fsm,'e_failed') return end
        if not dp:connectPorts('estimator', 'controller')      then rfsm.send_events(fsm,'e_failed') return end
        if not dp:connectPorts('reference', 'controller')      then rfsm.send_events(fsm,'e_failed') return end
        if not dp:connectPorts('io', 'controller')             then rfsm.send_events(fsm,'e_failed') return end
        if not dp:connectPorts('reference', 'motionplanning')  then rfsm.send_events(fsm,'e_failed') return end
        if not dp:connectPorts('estimator', 'motionplanning')  then rfsm.send_events(fsm,'e_failed') return end
          --add more connections here

        -- connect the deployer to coordinator (for communicating failure events)
        if not _deployer_fsm_event_port:connect(components.coordinator:getPort('coordinator_failure_event_port')) then rfsm.send_events(fsm,'e_failed') return end
        if not _deployer_failure_event_port:connect(components.coordinator:getPort('coordinator_fsm_event_port')) then rfsm.send_events(fsm,'e_failed') return end
        -- add every component as peer of coordinator
        for name, comp in pairs(components) do
          if (name ~= 'coordinator') then
            if not dp:addPeer('coordinator', name) then rfsm.send_events(fsm,'e_failed') return end
          end
        end
      end
    },

    connect_remote_components = rfsm.state{
      entry = function(fsm)
        local addOutgoing = components.communicator:getOperation("addOutgoing")
        local addIncoming = components.communicator:getOperation("addIncoming")
        -- coordinator
        dp:addPeer('communicator', 'coordinator')
        if not addIncoming('coordinator', 'coordinator_fsm_event_port', 4000) then rfsm.send_events(fsm, 'e_failed') return end
        -- io
        if components_to_load['teensy'] then
          dp:addPeer('communicator', 'io')
          if not addIncoming('io', 'cmd_velocity_port', 4002) then rfsm.send_events(fsm, 'e_failed') return end
        end
        -- distributed motion planning
        if distributed_mp then
          dp:addPeer('communicator', 'motionplanning')
          if not addOutgoing('motionplanning', 'x_var_port', 5000 + 10*index, neighbors) then rfsm.send_events(fsm, 'e_failed') return end
          for i=0, nghb_index.size-1 do
            if not addOutgoing('motionplanning', 'zl_ij_var_port_'..tostring(i), 5100 + 10*index + i, neighbor[i]) then rfsm.send_events(fsm, 'e_failed') return end
            if not addIncoming('motionplanning', 'zl_ji_var_port_'..tostring(i), 5100 + 10*nghb_index[i] + (2+i-1)%2) then rfsm.send_events(fsm, 'e_failed') return end
            if not addIncoming('motionplanning', 'x_j_var_port_'..tostring(i), 5000 + 10*nghb_index[i]) then rfsm.send_events(fsm, 'e_failed') return end
          end
        end
        -- deployer (added as last: highest priority)
        dp:addPeer('communicator', 'lua')
        if not addIncoming('lua', 'deployer_fsm_event_port', 4001) then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('lua', 'deployer_failure_event_port', 4001, broadcast) then rfsm.send_events(fsm,'e_failed') return end
      end
    },

    configure_components = rfsm.state {
      entry = function(fsm)
        for name, comp in pairs(components) do
          if (name ~= 'reporter') then -- reporter configured in later stage
            comp:loadService('marshalling')
            -- every component loads system configurations
            if not comp:provides('marshalling'):updateProperties(system_config_file) then rfsm.send_events(fsm,'e_failed') return end
            -- if available, a component loads its specific configurations
            if(component_config_files[name]) then
              if not comp:provides('marshalling'):updateProperties(component_config_files[name]) then rfsm.send_events(fsm,'e_failed') return end
            end
            -- configure the component
            if not comp:configure() then rfsm.send_events(fsm,'e_failed') return end
          end
        end
      end
    },

    set_activities = rfsm.state {
      entry = function(fsm)
        dp:setActivity('motionplanning', 0, 0, rtt.globals.ORO_SCHED_OTHER)
        dp:setActivity('coordinator', 1./control_sample_rate, 0, rtt.globals.ORO_SCHED_OTHER)
        dp:setActivity('io', 1./io_sample_rate, 0,rtt.globals.ORO_SCHED_OTHER)
        if components_to_load['scanmatcher'] then
          dp:setActivity('scanmatcher', 0, 0, rtt.globals.ORO_SCHED_OTHER)
        end
          --add here extra activities

        -- the estimator, controller and reference component are triggered by the coordinator and executed in the same thread.
        dp:setMasterSlaveActivity('coordinator', 'estimator')
        dp:setMasterSlaveActivity('coordinator', 'controller')
        dp:setMasterSlaveActivity('coordinator', 'reference')
        -- all elements of a container are triggered by the container
        for container, comps in pairs(containers_to_fill) do
          for i,name in pairs(comps) do
            dp:setMasterSlaveActivity(container, name)
          end
        end
        -- communicator should run in the fastest thread (= io thread)
        dp:setMasterSlaveActivity('coordinator', 'communicator')

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

