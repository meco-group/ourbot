require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'

--Some flags
local test = false -- odroid is not in a robot (no teensy, lidar, ...)

--Define component names here
local estimator     = 'estimator'..index
local controller    = 'controller'..index
local motionplanning= 'motionplanning'..index
local reference     = 'reference'..index
local coordinator   = 'coordinator'..index
local reporter      = 'reporter'..index
local io            = 'io'..index
local teensy        = 'teensy'..index
local lidar         = 'lidar'..index
local scanmatcher   = 'scanmatcher'..index
  --add here extra components

--Components to load
local components_to_load = {
  [estimator]       = estimator_type,
  [controller]      = controller_type,
  [motionplanning]  = motionplanning_type,
  [reference]       = 'Reference',
  [coordinator]     = 'OCL::LuaTLSFComponent',
  [reporter]        = 'OCL::NetcdfReporting',
  [io]              = 'Container',
  [teensy]          = 'TeensyBridge',
  [lidar]           = 'RPLidar',
  [scanmatcher]     = 'Scanmatcher'
   -- add here componentname = 'componenttype'
}

--Containers to fill
local containers_to_fill = {
  [io]  = {teensy, lidar}
}
if test then
  containers_to_fill = {
  [io]  = {}
  }
end

--Ports to report
local ports_to_report = {

  -- [controller]      = {'cmd_velocity_port'},
  [estimator]       =  {'est_pose_port'},--, 'scanstart_pose_port'},
  -- [reference]       = {'ref_velocity_port'}
  -- [coordinator]     = {'controlloop_duration', 'controlloop_jitter'},
  [io]              = {-- 'cal_lidar_node_port',
                       -- 'cal_imul_transacc_port',
                       -- 'cal_imul_orientation_3d_port',
                       -- 'cal_imul_orientation_port',
                       -- 'cal_imul_dorientation_3d_port',
                       -- 'cal_imul_dorientation_port',
                       -- 'cal_imur_transacc_port',
                       -- 'cal_imur_orientation_3d_port',
                       -- 'cal_imur_orientation_port',
                       -- 'cal_imur_dorientation_3d_port',
                       -- 'cal_imur_dorientation_port'
                       -- 'cal_lidar_x_port',
                       -- 'cal_lidar_y_port',
                       -- 'cal_enc_pose_port'
                       -- 'raw_imul_mag_port',
                       -- 'raw_imur_mag_port',
                       -- 'cal_lidar_global_node_port',
                       -- 'cal_motor_current_port',
                       -- 'cal_motor_voltage_port',
                       -- 'cal_velocity_port',
                      'cor_lidar_distance_port',
                      'cor_lidar_angle_port'
                        
                    },
  [scanmatcher] = {   -- 'scanmatch_pose_port', 
                      'artificial_lidar_distances_port',
                      'artificial_lidar_angles_port'
                    }
  --add here componentname = 'portnames'
}

--Packages to import
local packages_to_import = {
  [estimator]       = 'EstimatorInterface',
  [controller]      = 'ControllerInterface',
  [motionplanning]  = 'MotionPlanning',
  [reference]       = 'Reference',
  [io]              = 'Container',
  [teensy]          = 'SerialInterface',
  [lidar]           = 'SerialInterface',
  [scanmatcher]     = 'Scanmatcher'
  --add here componentname = 'parentcomponenttype'
}

if test then
  packages_to_import = {
    [estimator]       = 'EstimatorInterface',
    [controller]      = 'ControllerInterface',
    [motionplanning]  = 'MotionPlanning',
    [reference]       = 'Reference',
    [io]              = 'Container'
  }
end

--Configuration files to load
local system_config_file      = 'Configuration/system-config.cpf'
local reporter_config_file    = 'Configuration/reporter-config.cpf'
local component_config_files  = {
  [teensy]          = 'Configuration/teensy-config.cpf',
  [lidar]           = 'Configuration/lidar-config.cpf',
  [motionplanning]  = 'Configuration/motionplanning-config.cpf',
  [scanmatcher]     = 'Configuration/scanmatcher-config.cpf'
  --add here componentname = 'Configuration/component-config.cpf'
}

--Accessible components over CORBA
local server_components = {coordinator, io}
-- for comp,ports in pairs(ports_to_report) do
--   table.insert(server_components,comp)
-- end
if distributed_mp then
  table.insert(server_components, motionplanning)
end

local components = {}
local remote_components = {}
local dp = rtt.getTC():getPeer('Deployer')

return rfsm.state {

  rfsm.transition {src = 'initial',                   tgt = 'deploy'},
  rfsm.transition {src = 'deploy',                    tgt = 'failure',  events = {'e_failed'}},
  rfsm.transition {src = '.deploy.prepare_reporter',  tgt = 'deployed', events = {'e_done'}},
  rfsm.transition {src = 'failure',                   tgt = 'deploy',   events = {'e_reset'}},

  deployed = rfsm.conn{},

  failure = rfsm.state {
    entry = function()
      _deployer_failure_event_port:write('e_failed')
      for name,comp in pairs(components) do
        comp:stop()
        comp:cleanup()
      end
    end,
    exit = function()
      for name,type in pairs(components_to_load) do
        dp:unloadComponent(name)
      end
    end,
   },

  deploy = rfsm.state {
    rfsm.transition {src = 'initial',                   tgt = 'load_components'},
    rfsm.transition {src = 'load_components',           tgt = 'configure_components',       events = {'e_done'}},
    rfsm.transition {src = 'configure_components',      tgt = 'connect_components',         events = {'e_done'}},
    rfsm.transition {src = 'connect_components',        tgt = 'connect_distr_components',   events = {'e_done'}},
    rfsm.transition {src = 'connect_distr_components',  tgt = 'set_activities',             events = {'e_done'}},
    rfsm.transition {src = 'set_activities',            tgt = 'load_app_coordination',      events = {'e_done'}},
    rfsm.transition {src = 'load_app_coordination',     tgt = 'prepare_reporter',           events = {'e_done'}},

    load_components = rfsm.state {
      entry = function(fsm)
        components = {}
        --Import necessary packages
        for name,type in pairs(packages_to_import) do
          if (not rtt.provides("ros"):import(type) ) then rfsm.send_events(fsm,'e_failed') return end
        end
        --Go through the table of components to load them
        for name,type in pairs(components_to_load) do
          if (not dp:loadComponent(name,type)) then rfsm.send_events(fsm,'e_failed') return end
          components[name] = dp:getPeer(name)
        end
        --Fill containers with correct components
        for container,comps in pairs(containers_to_fill) do
          addToContainer = components[container]:getOperation("addComponent")
          for i,name in pairs(comps) do
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
        -- Go through the table of components to make server
        for i,name in pairs(server_components) do
          if (not dp:server(name,true)) then rfsm.send_events(fsm,'e_failed') return end
        end
      end
    },

    configure_components = rfsm.state {
      entry = function(fsm)
        for name,comp in pairs(components) do
          if (name ~= coordinator and name ~= reporter) then --coordinator/reporter is configured in a later stadium
            comp:loadService('marshalling')
            --Every component loads system configurations
            if (not comp:provides('marshalling'):updateProperties(system_config_file)) then rfsm.send_events(fsm,'e_failed') return end
            --If available, a component loads its specific configurations
            if(component_config_files[name]) then
              if (not comp:provides('marshalling'):updateProperties(component_config_files[name])) then rfsm.send_events(fsm,'e_failed') return end
            end
            --Configure the components
            if (not comp:configure()) then rfsm.send_events(fsm,'e_failed') return end
          end
        end
      end,
    },

    connect_components = rfsm.state {
      entry = function(fsm)
        --Connect teensy to lidar
        if components_to_load[teensy] and components_to_load[lidar] then
          if (not dp:connectPorts(teensy,lidar))            then rfsm.send_events(fsm,'e_failed') return end
        end

        --Connect scanmatcher
        if components_to_load[scanmatcher] then
          if (not dp:connectPorts(estimator,scanmatcher))     then rfsm.send_events(fsm,'e_failed') return end
          if (not dp:connectPorts(scanmatcher,io))            then rfsm.send_events(fsm,'e_failed') return end
        end

        --Connect all components
        if (not dp:connectPorts(estimator,io))              then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:connectPorts(estimator,controller))      then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:connectPorts(reference,controller))      then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:connectPorts(io,controller))             then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:connectPorts(reference,motionplanning))  then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:connectPorts(estimator,motionplanning))  then rfsm.send_events(fsm,'e_failed') return end
          --add more connections here

        --Add every component as peer of coordinator
        if (not dp:addPeer(coordinator,controller))         then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:addPeer(coordinator,estimator))          then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:addPeer(coordinator,motionplanning))     then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:addPeer(coordinator,reference))          then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:addPeer(coordinator,reporter))           then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:addPeer(coordinator,io))                 then rfsm.send_events(fsm,'e_failed') return end
        if components_to_load[scanmatcher] then
          if (not dp:addPeer(coordinator,scanmatcher))        then rfsm.send_events(fsm,'e_failed') return end
        end
          --add more peers here
      end,
    },

    connect_distr_components = rfsm.state{
      entry = function(fsm)
        if distributed_mp then
          cp=rtt.Variable("ConnPolicy")
          --Sleeping ...
          local ntime = os.time() + 2
          repeat until os.time() > ntime
          --Load components over the network & connect ports
          for i=0,neighbors.size-1 do
            local nghb_index = neighbors[i]
            local remote_cmp = 'motionplanning'..tostring(nghb_index)
            if (not remote_components[remote_cmp] ) then
              if (not dp:loadComponent(remote_cmp,'CORBA')) then rfsm.send_events(fsm,'e_failed') return end
              remote_components[remote_cmp] = dp:getPeer(remote_cmp)
            end
            --Connect distributed motionplanning ports: THIS HOLDS ONLY FOR CIRCULAR INTERCONNECTION!!
            if (not dp:connect(remote_cmp..'.x_var_port', 'motionplanning'..tostring(index)..'.x_j_var_port_'..tostring(i), cp)) then rfsm.send_events(fsm,'e_failed') return end
            if (not dp:connect(remote_cmp..'.zl_ij_var_port_'..tostring(1-i), 'motionplanning'..tostring(index)..'.zl_ji_var_port_'..tostring(i), cp)) then rfsm.send_events(fsm,'e_failed') return end
          end
        end
      end
    },

    set_activities = rfsm.state {
      entry = function(fsm)
        --dp:setActivity(motionplanning,0, 10,rtt.globals.ORO_SCHED_RT)
        dp:setActivity(coordinator,1./control_sample_rate, 7,rtt.globals.ORO_SCHED_RT)
        dp:setActivity(reporter,0,1,rtt.globals.ORO_SCHED_RT)
        dp:setActivity(io,1./io_sample_rate, 7,rtt.globals.ORO_SCHED_RT)
        if components_to_load[scanmatcher] then
          dp:setActivity(scanmatcher, 0, 5,rtt.globals.ORO_SCHED_RT)
        end
          --add here extra activities

        --The estimator, controller and reference component are triggered by the coordinator and executed in the same thread.
        dp:setMasterSlaveActivity(coordinator,estimator)
        dp:setMasterSlaveActivity(coordinator,controller)
        dp:setMasterSlaveActivity(coordinator,reference)

        for container,comps in pairs(containers_to_fill) do
          for i,name in pairs(comps) do
            dp:setMasterSlaveActivity(container,name)
          end
        end
      end,
    },

    load_app_coordination = rfsm.state {
      entry = function(fsm)
        --Load execution file in coordinator component
        if not components[coordinator]:exec_file(coordinator_file) then rfsm.send_events(fsm,'e_failed') return end
        --Copy the values of the application properties into the coordinator component
        components[coordinator]:getProperty('index'):set(index)
        components[coordinator]:getProperty('print_level'):set(print_level)
        components[coordinator]:getProperty('reporter_sample_rate'):set(reporter_sample_rate)
        --Configure the coordinator
        if not components[coordinator]:configure() then rfsm.send_events(fsm,'e_failed') return end
        --Connect the deployer to coordinator (for receiving each others failure events)
        if not _deployer_fsm_event_port:connect(components[coordinator]:getPort('coordinator_failure_event_port')) then rfsm.send_events(fsm,'e_failed') return end
        if not _deployer_failure_event_port:connect(components[coordinator]:getPort('coordinator_fsm_event_port')) then rfsm.send_events(fsm,'e_failed') return end
        --Load the local application script
        components[coordinator]:loadService("scripting")
        if not components[coordinator]:provides("scripting"):loadPrograms(app_file) then rfsm.send_events(fsm,'e_failed') return end
        --Start the coordinator
        if not components[coordinator]:start() then rfsm.send_events(fsm,'e_failed') return end
      end,
    },

    prepare_reporter = rfsm.state{
      entry = function(fsm)
        --Load the marshalling service and the previously defined configuration file
        components[reporter]:loadService('marshalling')
        components[reporter]:provides('marshalling'):loadProperties(reporter_config_file)
        --Add components to report
        for comp,portlist in pairs(ports_to_report) do
          for i,port in pairs(portlist) do
            if (not dp:addPeer(reporter,comp)) then rfsm.send_events(fsm,'e_failed') end
            if (not components[reporter]:reportPort(comp,port)) then rfsm.send_events(fsm,'e_failed') end
          end
        end
        --Configure the reporter
        if (not components[reporter]:configure()) then rfsm.send_events(fsm,'e_failed') return end
      end,
    },
  }
}

