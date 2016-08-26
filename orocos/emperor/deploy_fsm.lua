require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'

-- components to load
local components_to_load = {
  gamepad         = 'GamePad',
  communicator    = 'Communicator',
  emperor         = 'OCL::LuaTLSFComponent',
  reporter        = 'OCL::NetcdfReporting'
    --add here componentname = 'componenttype'
}

-- ports to report
local ports_to_report = {
  gamepad         = {'cmd_velocity_port'}
    --add here componentname = 'portnames'
}

-- remote ports to report
local remote_ports_to_report = {
  -- controller        = {'cmd_velocity_port'},
  -- estimator         = {'est_pose_port'}
  -- reference         = {'ref_pose_port', 'ref_velocity_port'}
  -- coordinator       = {'controlloop_duration', 'controlloop_jitter'},
  -- io                = {--'cal_lidar_node_port',
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
  --                     -- 'cal_velocity_port'
  --                     }
  --add here componentname = 'portnames'
}

-- packages to import
local packages_to_import = {
  gamepad = 'SerialInterfaceEmperor',
  communicator = 'Communicator'
    --add here componentname = 'parentcomponenttype'
}

-- configuration files to load
local system_config_file      = 'Configuration/system-config.cpf'
local reporter_config_file    = 'Configuration/reporter-config.cpf'
local component_config_files  = {
  reporter  = 'Configuration/reporter-config.cpf',
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
    rfsm.transition {src = 'load_components',           tgt = 'connect_components',         events = {'e_done'}},
    rfsm.transition {src = 'connect_components',        tgt = 'connect_remote_components',  events = {'e_done'}},
    rfsm.transition {src = 'connect_remote_components', tgt = 'configure_components',       events = {'e_done'}},
    rfsm.transition {src = 'configure_components',      tgt = 'set_activities',             events = {'e_done'}},
    rfsm.transition {src = 'set_activities',            tgt = 'load_emperor',               events = {'e_done'}},

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

    connect_components = rfsm.state {
      entry = function(fsm)
        -- connect the deployer to emperor (for communicating failure events)
        if not _deployer_fsm_event_port:connect(components.emperor:getPort('emperor_failure_event_port')) then rfsm.send_events(fsm,'e_failed') return end
        if not _deployer_failure_event_port:connect(components.emperor:getPort('emperor_fsm_event_port')) then rfsm.send_events(fsm,'e_failed') return end

        if not dp:connectPorts('emperor', 'gamepad') then rfsm.send_events(fsm,'e_failed') return end

        -- connect components-to-report
        for comp, portlist in pairs(ports_to_report) do
          for i, port in pairs(portlist) do
            if not dp:addPeer('reporter', comp) then rfsm.send_events(fsm,'e_failed') end
            if not components.reporter:reportPort(comp, port) then rfsm.send_events(fsm,'e_failed') end
          end
        end
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
        -- deployer (added as last: highest priority)
        dp:addPeer('communicator', 'lua')
        if not addIncoming('lua', 'deployer_fsm_event_port', 4001) then rfsm.send_events(fsm, 'e_failed') return end
        if not addOutgoing('lua', 'deployer_failure_event_port', 4001, broadcast) then rfsm.send_events(fsm,'e_failed') return end
      end,
    },

    configure_components = rfsm.state {
      entry = function(fsm)
        for name, comp in pairs(components) do
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
      end,
    },

    set_activities = rfsm.state {
      entry = function(fsm)
        dp:setActivity('emperor', 1./emperor_sample_rate, 10, rtt.globals.ORO_SCHED_RT)
        dp:setActivity('gamepad', 1./velcmd_sample_rate, 10, rtt.globals.ORO_SCHED_RT)
        dp:setActivity('reporter', 0, 2, rtt.globals.ORO_SCHED_RT)
        dp:setMasterSlaveActivity('emperor', 'communicator')
          --add here extra activities
      end,
    },

    load_emperor = rfsm.state {
      entry = function(fsm)
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
