require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'

--Components to load
local components_to_load = {
  velocitycmd     = velocitycmd_type,
  emperor         = 'OCL::LuaTLSFComponent',
  reporter        = 'OCL::NetcdfReporting'
   --add here componentname = 'componenttype'
}

--Ports to report
local ports_to_report = {
  velocitycmd       = {'cmd_velocity_port'}
  --add here componentname = 'portnames'
}

--Distributed ports to report
local distrports_to_report = {
  controller        = {'cmd_velocity_port'},
  estimator         = {'est_pose_port', 'est_velocity_port', 'est_acceleration_port', 'est_global_offset_port'},
  reference         = {'ref_pose_port', 'ref_ffw_port'},
  velocitycmd       = {'cmd_velocity_port'},
  coordinator       = {'controlloop_duration', 'controlloop_jitter'}
  --add here componentname = 'portnames'
}

--Packages to import
local packages_to_import = {
  velocitycmd = 'VelocityCommandInterface'
  --add here componentname = 'parentcomponenttype'
}

--Configuration files to load
local system_config_file      = 'Configuration/emperor-config.cpf'
local reporter_config_file    = 'Configuration/reporter-config.cpf'
local component_config_files  = {
  --add here componentname = 'Configuration/component-config.cpf'
}

--Distributed components to load
local distr_components_to_load = {}

for i=0,peers.size-1 do
  table.insert(distr_components_to_load,'coordinator'..peers[i])
  table.insert(distr_components_to_load,'io'..peers[i])
  table.insert(distr_components_to_load,'controller'..peers[i])
  table.insert(distr_components_to_load,'estimator'..peers[i])
  table.insert(distr_components_to_load,'reference'..peers[i])
  table.insert(distr_components_to_load,'velocitycmd'..peers[i])
end

local components      = {}
local distrcomponents = {}
local dp = rtt.getTC():getPeer('Deployer')

return rfsm.state {

  rfsm.transition {src = 'initial',                  tgt = 'deploy'},
  rfsm.transition {src = 'deploy',                   tgt = 'failure',  events = {'e_failed'}},
  rfsm.transition {src = '.deploy.prepare_reporter', tgt = 'deployed', events = {'e_done'}},
  rfsm.transition { src = 'failure',                 tgt = 'deploy',   events = {'e_reset'}},

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
    rfsm.transition {src = 'initial',               tgt = 'load_components'},
    rfsm.transition {src = 'load_components',       tgt = 'load_peers',             events = {'e_done'}},
    rfsm.transition {src = 'load_peers',            tgt = 'configure_components',   events = {'e_done'}},
    rfsm.transition {src = 'configure_components',  tgt = 'connect_components',     events = {'e_done'}},
    rfsm.transition {src = 'connect_components',    tgt = 'set_activities',         events = {'e_done'}},
    rfsm.transition {src = 'set_activities',        tgt = 'load_app_coordination',  events = {'e_done'}},
    rfsm.transition {src = 'load_app_coordination', tgt = 'prepare_reporter',       events = {'e_done'}},


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
      end
    },

    load_peers = rfsm.state {
      entry = function(fsm)
        for i,name in pairs(distr_components_to_load) do
          if (not dp:loadComponent(name,'CORBA')) then rfsm.send_events(fsm,'e_failed') return end
          distrcomponents[name] = dp:getPeer(name)
        end
      end
    },

    configure_components = rfsm.state {
      entry = function(fsm)
        for name,comp in pairs(components) do
          if (name ~= 'emperor' and name ~= 'reporter') then --emperor/reporter is configured in a later stadium
            comp:loadService('marshalling')
            --Every component loads system configurations
            if (not comp:provides('marshalling'):updateProperties(system_config_file)) then rfsm.send_events(fsm,'e_failed') return end
            --If available, a component loads its specific configurations
            if(component_config_files[name]) then
              if (not comp:provides('marshalling'):updateProperties(component_config[name])) then rfsm.send_events(fsm,'e_failed') return end
            end
            --Configure the components
            if (not comp:configure()) then rfsm.send_events(fsm,'e_failed') return end
          end
        end
      end,
    },

    connect_components = rfsm.state {
      entry = function(fsm)
        --Connect all components
        for i=0,peers.size-1 do
          if (not dp:connectPorts('velocitycmd','io'..peers[i])) then rfsm.send_events(fsm,'e_failed') return end
        end
          --add more connections here

        --Add every component as peer of emperor
        for i=0,peers.size-1 do
          if (not dp:addPeer('emperor','coordinator'..peers[i])) then rfsm.send_events(fsm,'e_failed') return end
        end
        if (not dp:addPeer('emperor','reporter')) then rfsm.send_events(fsm,'e_failed') return end
        if (not dp:addPeer('emperor','velocitycmd')) then rfsm.send_events(fsm,'e_failed') return end
          --add more peers here
      end,
    },

    set_activities = rfsm.state {
      entry = function(fsm)
        dp:setActivity('velocitycmd',1./velcmd_sample_rate,10,rtt.globals.ORO_SCHED_RT)
        dp:setActivity('reporter',0,2,rtt.globals.ORO_SCHED_RT)
          --add here extra activities
      end,
    },


    load_app_coordination = rfsm.state {
      entry = function(fsm)
        --Load execution file in emperor component
        if not components.emperor:exec_file(emperor_file) then rfsm.send_events(fsm,'e_failed') return end
        --Copy the values of the application properties into the coordinator emperor
        components.emperor:getProperty('peers'):set(peers)
        --Configure the emperor
        if not components.emperor:configure() then rfsm.send_events(fsm,'e_failed') return end
        --Connect emperor to each coordinator
        for i=0,peers.size-1 do
          port1 = components.emperor:getPort('emperor_send_event_port')
          port2 = distrcomponents['coordinator'..tostring(peers[i])]:getPort('coordinator_fsm_event_port')
          if (not port1:connect(port2)) then rfsm.send_events(fsm,'e_failed') return end
        end
        --Connect failure event ports from coordinators to emperor and visa versa
        for i=0,peers.size-1 do
          port1 = components.emperor:getPort('emperor_fsm_event_port')
          port2 = distrcomponents['coordinator'..tostring(peers[i])]:getPort('coordinator_failure_event_port')
          if (not port1:connect(port2)) then rfsm.send_events(fsm,'e_failed') return end
          port1 = distrcomponents['coordinator'..tostring(peers[i])]:getPort('coordinator_fsm_event_port')
          port2 = components.emperor:getPort('emperor_failure_event_port')
          if (not port1:connect(port2)) then rfsm.send_events(fsm,'e_failed') return end
        end
        --Connect the deployer to emperor (for receiving each others failure events)
        if not _deployer_fsm_event_port:connect(components.emperor:getPort('emperor_failure_event_port')) then rfsm.send_events(fsm,'e_failed') return end
        if not _deployer_failure_event_port:connect(components.emperor:getPort('emperor_fsm_event_port')) then rfsm.send_events(fsm,'e_failed') return end
        --Load the local application script
        components.emperor:loadService("scripting")
        if not components.emperor:provides("scripting"):loadPrograms(app_file) then rfsm.send_events(fsm,'e_failed') return end
        --Start the emperor
        if not components.emperor:start() then rfsm.send_events(fsm,'e_failed') return end
      end,
    },

    prepare_reporter = rfsm.state{
      entry = function(fsm)
        --Load the marshalling service and the previously defined configuration file
        components.reporter:loadService('marshalling')
        components.reporter:provides('marshalling'):loadProperties(reporter_config_file)
        --Add components to report
        for comp,portlist in pairs(ports_to_report) do
          for i,port in pairs(portlist) do
            if (not dp:addPeer('reporter',comp)) then rfsm.send_events(fsm,'e_failed') end
            if (not components.reporter:reportPort(comp,port)) then rfsm.send_events(fsm,'e_failed') end
          end
        end
        --Add distributed components to report
        for comp,portlist in pairs(distrports_to_report) do
          for i,port in pairs(portlist) do
            for i=0,peers.size-1 do
              if (not dp:addPeer('reporter',comp..peers[i])) then rfsm.send_events(fsm,'e_failed') end
              if (not components.reporter:reportPort(comp..peers[i],port)) then rfsm.send_events(fsm,'e_failed') end
            end
          end
        end
        --Configure the reporter
        if (not components.reporter:configure()) then rfsm.send_events(fsm,'e_failed') end
      end
    },
},

}
