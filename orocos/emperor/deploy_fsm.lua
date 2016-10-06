require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'

-- components to load
local components_to_load = {
  gamepad         = 'GamePad',
  communicator    = 'Communicator',
  hawkeye         = 'HawkEye',
  emperor         = 'OCL::LuaTLSFComponent',
  reporter        = 'OCL::NetcdfReporting'
    --add here componentname = 'componenttype'
}

-- ports to report
local ports_to_report = {
  gamepad         = {'cmd_velocity_port'}
    --add here componentname = 'portnames'
}

-- packages to import
local packages_to_import = {
  gamepad = 'Serial',
  hawkeye = 'HawkEye',
  communicator = 'Communicator'
    --add here componentname = 'parentcomponenttype'
}

-- configuration files to load
local system_config_file      = 'Configuration/system-config.cpf'
local component_config_files  = {
  reporter  = 'Configuration/reporter-config.cpf',
  hawkeye   = 'Configuration/hawkeye-config.cpf',
  gamepad   = 'Configuration/gamepad-config.cpf'
    --add here componentname = 'Configuration/component-config.cpf'
}

local components      = {}
local remote_components = {}
local dp = rtt.getTC():getPeer('Deployer')

return rfsm.state {

  rfsm.transition {src = 'initial',                  tgt = 'deploy'},
  rfsm.transition {src = 'deploy',                   tgt = 'failure',  events = {'e_failed'}},
  rfsm.transition {src = 'deployed',                 tgt = 'failure',  events = {'e_failed'}},
  rfsm.transition {src = '.deploy.load_emperor',     tgt = 'deployed', events = {'e_done'}},
  rfsm.transition {src = 'failure',                  tgt = 'deploy',   events = {'e_reset'}},

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
    rfsm.transition {src = 'load_components',           tgt = 'configure_components',       events = {'e_done'}},
    rfsm.transition {src = 'configure_components',      tgt = 'connect_components',         events = {'e_done'}},
    rfsm.transition {src = 'connect_components',        tgt = 'connect_remote_components',  events = {'e_done'}},
    rfsm.transition {src = 'connect_remote_components', tgt = 'set_activities',             events = {'e_done'}},
    rfsm.transition {src = 'set_activities',            tgt = 'prepare_reporter',           events = {'e_done'}},
    rfsm.transition {src = 'prepare_reporter',          tgt = 'load_emperor',               events = {'e_done'}},

    initial = rfsm.conn{},

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
        -- add all components as peer of emperor
        for name, comp in pairs(components) do
          if (name ~= 'emperor') then
            if not dp:addPeer('emperor', name) then rfsm.send_events(fsm,'e_failed') return end
          end
        end
        -- load execution file in emperor component
        if not components.emperor:exec_file(emperor_file) then rfsm.send_events(fsm,'e_failed') return end
      end
    },


    configure_components = rfsm.state {
      entry = function(fsm)
        for name, comp in pairs(components) do
          if (name ~= 'reporter' and name ~= 'emperor') then -- reporter configured in later stage
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
      end,
    },

    connect_components = rfsm.state {
      entry = function(fsm)
        -- connect the deployer to emperor (for communicating failure events)
        if not _deployer_fsm_event_port:connect(components.emperor:getPort('emperor_failure_event_port')) then rfsm.send_events(fsm,'e_failed') return end
        if not _deployer_failure_event_port:connect(components.emperor:getPort('emperor_fsm_event_port')) then rfsm.send_events(fsm,'e_failed') return end

        if not dp:connectPorts('emperor', 'gamepad') then rfsm.send_events(fsm,'e_failed') return end
      end,
    },

    connect_remote_components = rfsm.state {
      entry = function(fsm)
        local addOutgoing = components.communicator:getOperation("addOutgoing")
        local addIncoming = components.communicator:getOperation("addIncoming")
        -- emperor
        dp:addPeer('communicator', 'emperor')
        if not addOutgoing('emperor', 'emperor_send_event_port', 4000, robots) then rfsm.send_events(fsm,'e_failed') return end
        -- gamepad
        dp:addPeer('communicator', 'gamepad')
        if not addOutgoing('gamepad', 'cmd_velocity_port', 4002, robots) then rfsm.send_events(fsm,'e_failed') return end
        -- hawkeye
        dp:addPeer('communicator', 'hawkeye')
        if not addOutgoing('hawkeye', 'kurt_pose_port', 6050, kurt) then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('hawkeye', 'krist_pose_port', 6051, kurt) then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('hawkeye', 'dave_pose_port', 6052, kurt) then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('hawkeye', 'obstacle_port', 6070, robots) then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('hawkeye', 'target_pose_port', 6071, robots) then rfsm.send_events(fsm, 'e_failed') return end
        -- if not addIncoming('hawkeye', 'kurt_est_pose_port', 6000) then rfsm.send_events(fsm,'e_failed') return end
        -- if not addIncoming('hawkeye', 'krist_est_pose_port', 6001) then rfsm.send_events(fsm,'e_failed') return end
        -- if not addIncoming('hawkeye', 'dave_est_pose_port', 6002) then rfsm.send_events(fsm,'e_failed') return end

        -- if not addIncoming('hawkeye', 'ref_x_kurt_port', 6010) then rfsm.send_events(fsm,'e_failed') return end
        -- if not addIncoming('hawkeye', 'ref_x_krist_port', 6011) then rfsm.send_events(fsm,'e_failed') return end
        -- if not addIncoming('hawkeye', 'ref_x_dave_port', 6012) then rfsm.send_events(fsm,'e_failed') return end
        -- if not addIncoming('hawkeye', 'ref_y_kurt_port', 6020) then rfsm.send_events(fsm,'e_failed') return end
        -- if not addIncoming('hawkeye', 'ref_y_krist_port', 6021) then rfsm.send_events(fsm,'e_failed') return end
        -- if not addIncoming('hawkeye', 'ref_y_dave_port', 6022) then rfsm.send_events(fsm,'e_failed') return end

        -- deployer (added as last: highest priority)
        dp:addPeer('communicator', 'lua')
        if not addIncoming('lua', 'deployer_fsm_event_port', 4001) then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('lua', 'deployer_failure_event_port', 4001, broadcast) then rfsm.send_events(fsm,'e_failed') return end
      end,
    },

    set_activities = rfsm.state {
      entry = function(fsm)
        dp:setActivity('emperor', 1./emperor_sample_rate, 0, rtt.globals.ORO_SCHED_OTHER)
        dp:setActivity('gamepad', 1./velcmd_sample_rate, 0, rtt.globals.ORO_SCHED_OTHER)
        dp:setActivity('hawkeye', 1./hawkeye_sample_rate, 0, rtt.globals.ORO_SCHED_OTHER)
        dp:setActivity('reporter', 0, 0, rtt.globals.ORO_SCHED_OTHER)
        dp:setMasterSlaveActivity('emperor', 'communicator')
          --add here extra activities
      end,
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

    load_emperor = rfsm.state {
      entry = function(fsm)
        -- emperor loads system configurations
        components.emperor:loadService('marshalling')
        if not components.emperor:provides('marshalling'):updateProperties(system_config_file) then rfsm.send_events(fsm,'e_failed') return end
        -- configure the emperor
        if not components.emperor:configure() then rfsm.send_events(fsm,'e_failed') return end
        -- load the local application script
        components.emperor:loadService("scripting")
        if not components.emperor:provides("scripting"):loadPrograms(app_file) then rfsm.send_events(fsm,'e_failed') return end
        -- start the emperor and gamepad
        if not components.emperor:start() then rfsm.send_events(fsm,'e_failed') return end
        if not components.communicator:start() then rfsm.send_events(fsm, 'e_failed') return end
        components.gamepad:start()
      end,
    },
  },
}
