require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'

-- components to load
local components_to_load = {
  coordinator     = 'OCL::LuaTLSFComponent',
  communicator    = 'Communicator',
  hawkeye         = 'Hawkeye'
}

-- ports to report
local ports_to_report = {
    --add here componentname = 'portnames'
}

-- packages to import
local packages_to_import = {
  communicator    = 'Communicator',
  hawkeye         = 'Hawkeye'
}

-- configuration files to load
local system_config_file      = 'Configuration/system-config.cpf'
local component_config_files  = {
  Hawkeye         = 'Configuration/hawkeye-config.cpf'
  reporter        = 'Configuration/reporter-config.cpf',
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
        -- load execution file in coordinator component
        if not components.coordinator:exec_file(coordinator_file) then rfsm.send_events(fsm,'e_failed') return end
      end
    },


    connect_components = rfsm.state {
      entry = function(fsm)
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
        -- hawkeye
        dp:addPeer('communicator', 'hawkeye')
        if not addOutgoing('hawkeye', 'dave_port', 6000, dave) then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('hawkeye', 'dave_port', 6000, emperor) then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('hawkeye', 'kurt_port', 6001, kurt) then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('hawkeye', 'kurt_port', 6001, emperor) then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('hawkeye', 'krist_port', 6002, krist) then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('hawkeye', 'krist_port', 6002, emperor) then rfsm.send_events(fsm, 'e_failed') return end
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
        dp:setActivity('coordinator', 1./cam_sample_rate, 0, rtt.globals.ORO_SCHED_OTHER)
        -- the hawkeye is triggered by the coordinator and executed in the same thread.
        dp:setMasterSlaveActivity('coordinator', 'hawkeye')
        -- communicator should run in the fastest thread
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

