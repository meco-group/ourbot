require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'

--Components to load
local components_to_load = {
  velocitycmd     = velocitycmd_type,
  emperor         = 'OCL::LuaTLSFComponent'
   -- reporter='OCL::NetcdfReporting'
   --add here componentname = 'componenttype'
}

--Packages to import
local packages_to_import = {
  velocitycmd = 'VelocityCommandInterface'
  --add here componentname = 'parentcomponenttype'
}

--Configuration files to load
local system_config_file      = 'Configuration/emperor.cpf'
local component_config_files  = {
  --add here componentname = 'Configuration/component-config.cpf'
}

--Peer components to load
local distr_components_to_load = {}

for i=0,peers.size-1 do
  table.insert(distr_components_to_load,'coordinator'..peers[i])
  table.insert(distr_components_to_load,'sensors'..peers[i])
end

local components = {}
local distrcomponents = {}
local dp = rtt.getTC():getPeer('Deployer')

return rfsm.state {

   rfsm.transition {src = 'initial', tgt = 'deploy'},

   rfsm.transition {src = 'deploy', tgt = 'failure', events = {'e_failed'}},

   deploy = rfsm.state {
      rfsm.transition {src = 'initial', tgt = 'load_components'},

      load_components = rfsm.state {
        entry = function(fsm)
          components = {}
          --Import necessary packages
          for name,type in pairs(packages_to_import) do
            if (not rtt.provides("ros"):import(type) ) then rfsm.send_events(fsm,'e_failed') return end
          end

          --Go through the table of components to load them, send 'e_fail' if loading fails
          for name,type in pairs(components_to_load) do
            if (not dp:loadComponent(name,type)) then rfsm.send_events(fsm,'e_failed') return end
            --store the component in the components table
            components[name] = dp:getPeer(name)
          end
        end
      },

      rfsm.transition {src = 'load_components', tgt = 'load_peers', events = {'e_done'}},

      load_peers = rfsm.state {
        entry = function(fsm)
          for i,name in pairs(distr_components_to_load) do
            if (not dp:loadComponent(name,'CORBA')) then rfsm.send_events(fsm,'e_failed') return end
            --store the component in the distributed components table
            distrcomponents[name] = dp:getPeer(name)
          end
        end
      },

      rfsm.transition {src = 'load_peers', tgt = 'configure_components', events = {'e_done'}},

      configure_components = rfsm.state {
        entry = function(fsm)
          for name,comp in pairs(components) do
            if (name ~= 'emperor') then --emperor is configured in a later stadium
              comp:loadService('marshalling')
              --Every component loads system configurations
              if (not comp:provides('marshalling'):loadProperties(system_config_file)) then rfsm.send_events(fsm,'e_failed') return end
              --If available, a component loads its specific configurations
              if(component_config_files[name]) then
                if (not comp:provides('marshalling'):loadProperties(component_config[name])) then rfsm.send_events(fsm,'e_failed') return end
              end
              --Configure the components and send 'e_failed' if configuring fails
              if (not comp:configure()) then rfsm.send_events(fsm,'e_failed') return end
            end
          end
        end,
      },

      rfsm.transition {src = 'configure_components', tgt = 'connect_components', events = {'e_done'}},

      connect_components = rfsm.state {
        entry = function(fsm)
          --Connect all components
          for i=0,peers.size-1 do
            if (not dp:connectPorts('velocitycmd','sensors'..peers[i])) then rfsm.send_events(fsm,'e_failed') return end
          end
            --add more connections here

          --Add every component as peer of emperor
          for i=0,peers.size-1 do
            if (not dp:addPeer('emperor','coordinator'..peers[i])) then rfsm.send_events(fsm,'e_failed') return end
          end
          if (not dp:addPeer('emperor','velocitycmd')) then rfsm.send_events(fsm,'e_failed') return end
            --add more peers here
        end,
      },

      rfsm.transition {src = 'connect_components', tgt = 'set_activities', events={'e_done'}},

      set_activities = rfsm.state {
        entry = function(fsm)
          dp:setActivity('velocitycmd',1./velcmd_sample_rate,7,rtt.globals.ORO_SCHED_RT)
            --add here extra activities
          -- dp:setActivity('reporter',0,2,rtt.globals.ORO_SCHED_RT)
        end,
      },

      rfsm.transition {src = 'set_activities', tgt = 'load_app_coordination', events = {'e_done'}},

      load_app_coordination = rfsm.state {
        entry = function(fsm)
          --Try to load the emperor component, drop out if it fails
          if not components.emperor:exec_file(emperor_file) then rfsm.send_events(fsm,'e_failed') return end

          --Copy the values of the application properties into the emperor component
          components.emperor:getProperty('peers'):set(peers)

          --Try to configure the emperor, drop out if it fails
          if not components.emperor:configure() then rfsm.send_events(fsm,'e_failed') return end

          --Connect emperor to each coordinator
          for i=0,peers.size-1 do
            if not dp:connectPorts('emperor','coordinator'..peers[i]) then rfsm.send_events(fsm,'e_failed') return end
          end

          --Connect failure event ports from coordinators to emperor and visa versa
          for i=0,peers.size-1 do
            port1 = components.emperor:getPort('emperor_fsm_event_port')
            port2 = distrcomponents['coordinator'..peers[0]]:getPort('failure_event_port')
            if (not port1:connect(port2)) then rfsm.send_events(fsm,'e_failed') return end
            port1 = distrcomponents['coordinator'..peers[0]]:getPort('fsm_event_port')
            port2 = components.emperor:getPort('failure_event_port')
            if (not port1:connect(port2)) then rfsm.send_events(fsm,'e_failed') return end
          end

          --Try to connect deployer to emperor
          if not _fsm_event_port:connect(components.emperor:getPort('failure_event_port')) then rfsm.send_events(fsm,'e_failed') return end

          --Try to load the local application script
          components.emperor:loadService("scripting")
          if not components.emperor:provides("scripting"):loadPrograms(app_file) then rfsm.send_events(fsm,'e_failed') return end

          --Try to start the emperor
          if not components.emperor:start() then rfsm.send_events(fsm,'e_failed') return end
        end,
      },

      rfsm.transition {src = 'load_app_coordination', tgt = 'prepare_reporter', events = {'e_done'}},

      prepare_reporter = rfsm.state{
        -- entry = function(fsm)
        --   --Create necessary connections
        --   dp:connectPeers('reporter','reference')
        --   dp:connectPeers('reporter','controller')
        --   dp:connectPeers('reporter','plant')
        --   dp:connectPeers('reporter','emperor')
        --   --Load the marshalling service and the previously defined configuration file
        --   components.reporter:loadService('marshalling')
        --   components.reporter:provides('marshalling'):loadProperties(reporter_configuration_file)
        --   --Try to configure the reporter, drop out if it fails
        --   if (not components.reporter:configure()) then
        --      rfsm.send_events(fsm,'e_failed')
        --   end
        -- end
      },
  },

  --Deployment is finished.
  rfsm.transition {src = '.deploy.prepare_reporter', tgt='deployed', events = {'e_done'}},

  deployed = rfsm.conn{},

  --Currently there is no logic to failed from a failed deployement
  failure = rfsm.state {
    exit = function()
      for name,type in pairs(components_to_load) do
          dp:unloadComponent(name)
      end
    end,
  },

  rfsm.transition { src = 'failure', tgt = 'deploy', events = {'e_reset'}},
}

