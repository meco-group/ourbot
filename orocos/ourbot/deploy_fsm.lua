require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'

--Define component names here
local estimator     = 'estimator'..index
local controller    = 'controller'..index
local pathgenerator = 'pathgenerator'..index
local reference     = 'reference'..index
local velocitycmd   = 'velocitycmd'..index
local coordinator   = 'coordinator'..index
local reporter      = 'reporter'..index
local io            = 'io'..index
local teensy        = 'teensy'..index
local lidar         = 'lidar'..index
local imul          = 'imul'..index
-- local imur          = 'imur'..index
local spimaster     = 'spimaster'..index
  --add here extra components

--Components to load
local components_to_load = {
  [estimator]       = estimator_type,
  [controller]      = controller_type,
  [pathgenerator]   = pathgenerator_type,
  [reference]       = reference_type,
  [velocitycmd]     = velocitycmd_type,
  [coordinator]     = 'OCL::LuaTLSFComponent',
  [reporter]        = 'OCL::NetcdfReporting',
  [io]              = 'Container',
  [teensy]          = 'TeensyBridge',
  [lidar]           = 'RPLidar',
  [spimaster]       = 'SPIMaster',
  -- [imul]            = 'IMU'
  -- [imur]            = 'IMU',
   --add here componentname = 'componenttype'
}

--Containers to fill
local containers_to_fill = {
  [io]              = {teensy, lidar, spimaster, imul} --, imur}
}

--SPI components
local spi_components = {
  [spimaster]       = {imul} -- ,imur}
}

--Ports to report
local ports_to_report = {
  [controller]      = {'cmd_velocity_port'},
  [estimator]       = {'est_pose_port', 'est_velocity_port', 'est_acceleration_port', 'est_global_offset_port'},
  [reference]       = {'ref_pose_port', 'ref_ffw_port'},
  [velocitycmd]     = {'cmd_velocity_port'},
  [io]              = {'cal_enc_pose_port', 'cal_velocity_port', 'cal_lidar_node_port'},
  [coordinator]     = {'controlloop_duration', 'controlloop_jitter'}
  --add here componentname = 'portnames'
}

--Packages to import
local packages_to_import = {
  [estimator]       = 'EstimatorInterface',
  [controller]      = 'ControllerInterface',
  [pathgenerator]   = 'PathGeneratorInterface',
  [reference]       = 'Reference',
  [velocitycmd]     = 'VelocityCommandInterface',
  [io]              = 'Container',
  [teensy]          = 'SerialInterface',
  [lidar]           = 'SerialInterface'
  -- [spimaster]       = 'SPIMaster'
  --add here componentname = 'parentcomponenttype'
}

--Configuration files to load
local system_config_file      = 'Configuration/system-config.cpf'
local reporter_config_file    = 'Configuration/reporter-config.cpf'
local component_config_files  = {
  [teensy]          = 'Configuration/teensy-config.cpf',
  [lidar]           = 'Configuration/lidar-config.cpf'
  -- [spimaster]       = 'Configuration/spimaster-config.cpf',
  -- [imul]            = 'Configuration/imul-config.cpf'
  -- [imur]            = 'Configuration/imur-config.cpf'
  --add here componentname = 'Configuration/component-config.cpf'
}

--Accessible components over CORBA
local server_components = {coordinator}

for comp,ports in pairs(ports_to_report) do
  table.insert(server_components,comp)
end
if distributed then
  table.insert(server_components,estimator)
  table.insert(server_components,controller)
  table.insert(server_components,pathgenerator)
end

local components = {}
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
            addToContainer(name)
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
        --Connect SPI devices to SPI master
        -- for master,devices in pairs(spi_components) do
        --   for i,dev in pairs(devices) do
        --     if (not dp:connectPorts(master,dev)) then rfsm.send_events(fsm,'e_failed') return end
        --   end
        -- end

        --Connect all components
        if (not dp:connectPorts(estimator,io))              then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:connectPorts(estimator,controller))      then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:connectPorts(reference,controller))      then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:connectPorts(reference,pathgenerator))   then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:connectPorts(estimator,pathgenerator))   then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:connectPorts(velocitycmd,io))            then rfsm.send_events(fsm,'e_failed') return end
          --add more connections here

        --Add every component as peer of coordinator
        if (not dp:addPeer(coordinator,controller))         then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:addPeer(coordinator,estimator))          then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:addPeer(coordinator,pathgenerator))      then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:addPeer(coordinator,reference))          then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:addPeer(coordinator,velocitycmd))        then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:addPeer(coordinator,reporter))           then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:addPeer(coordinator,io))                 then rfsm.send_events(fsm,'e_failed') return end
          --add more peers here
      end,
    },

    connect_distr_components = rfsm.state{
      entry = function(fsm)
        if distributed then
          dir_tbl = {[1]='L', [2]='R'}
          nghb_cnt = 1
          --Load components over the network & connect ports
          for i=0,neighbours.size-1 do
            --Estimator
            if (not dp:loadComponent('estimator'..tostring(neighbours[i]),'CORBA')) then rfsm.send_events(fsm,'e_failed') return end
            if (not dp:connect('estimator'..tostring(index)..'.com_in'..dir_tbl[nghb_cnt]..'_port', 'estimator'..tostring(neighbours[i])..'.est_pose_port', cp)) then rfsm.send_events(fsm,'e_failed') return end
            distrcomponents['estimator'..tostring(neighbours[i])] = dp:getPeer('estimator'..tostring(neighbours[i]))
            writeSample = distrcomponents['estimator'..tostring(neighbours[i])]:getOperation("writeSample")
            writeSample()
            --Controller
            if (not dp:loadComponent('controller'..tostring(neighbours[i]),'CORBA')) then rfsm.send_events(fsm,'e_failed') return end
            if (not dp:connect('controller'..tostring(index)..'.com_in'..dir_tbl[nghb_cnt]..'_port', 'controller'..tostring(neighbours[i])..'.est_pose_port', cp)) then rfsm.send_events(fsm,'e_failed') return end
            distrcomponents['controller'..tostring(neighbours[i])] = dp:getPeer('controller'..tostring(neighbours[i]))
            writeSample = distrcomponents['controller'..tostring(neighbours[i])]:getOperation("writeSample")
            writeSample()
            --Pathgenerator
            if (not dp:loadComponent('pathgenerator'..tostring(neighbours[i]),'CORBA')) then rfsm.send_events(fsm,'e_failed') return end
            if (not dp:connect('pathgenerator'..tostring(index)..'.ref_in'..dir_tbl[nghb_cnt]..'_path_x_port', 'controller'..tostring(neighbours[i])..'.ref_pose_path_x_port', cp)) then rfsm.send_events(fsm,'e_failed') return end
            if (not dp:connect('pathgenerator'..tostring(index)..'.ref_in'..dir_tbl[nghb_cnt]..'_path_y_port', 'controller'..tostring(neighbours[i])..'.ref_pose_path_y_port', cp)) then rfsm.send_events(fsm,'e_failed') return end
            if (not dp:connect('pathgenerator'..tostring(index)..'.ref_in'..dir_tbl[nghb_cnt]..'_path_t_port', 'controller'..tostring(neighbours[i])..'.ref_pose_path_t_port', cp)) then rfsm.send_events(fsm,'e_failed') return end
            distrcomponents['pathgenerator'..tostring(neighbours[i])] = dp:getPeer('pathgenerator'..tostring(neighbours[i]))
            writeSample = distrcomponents['pathgenerator'..tostring(neighbours[i])]:getOperation("writeSample")
            writeSample()
            nghb_cnt = nghb_cnt + 1
          end
        end
      end
    },

    set_activities = rfsm.state {
      entry = function(fsm)
        dp:setActivity(coordinator,1./control_sample_rate,10,rtt.globals.ORO_SCHED_RT)
        dp:setActivity(pathgenerator,1./pathupd_sample_rate,10,rtt.globals.ORO_SCHED_RT)
        dp:setActivity(velocitycmd,1./velcmd_sample_rate,10,rtt.globals.ORO_SCHED_RT)
        dp:setActivity(reporter,0,2,rtt.globals.ORO_SCHED_RT)
        dp:setActivity(io,1./200,10,rtt.globals.ORO_SCHED_RT)
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

