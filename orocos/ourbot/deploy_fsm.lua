require 'rttlib'
require 'rfsm_rtt'
require 'rfsmpp'

--Define component names here
local estimator     = 'estimator'..index
local controller    = 'controller'..index
local pathgenerator = 'pathgenerator'..index
local reference     = 'reference'..index
local velocitycmd   = 'velocitycmd'..index
local sensors       = 'sensors'..index
local coordinator   = 'coordinator'..index
local reporter      = 'reporter'..index

--Components to load
local components_to_load = {
  [estimator]       = estimator_type,
  [controller]      = controller_type,
  [pathgenerator]   = pathgenerator_type,
  [reference]       = reference_type,
  [velocitycmd]     = velocitycmd_type,
  [sensors]         = 'Sensors',
  [coordinator]     = 'OCL::LuaTLSFComponent',
  [reporter]        = 'OCL::NetcdfReporting'
   --add here componentname = 'componenttype'
}

--Ports to report
local ports_to_report = {
  cmd_velocity_port     = controller,
  est_pose_port         = estimator,
  est_velocity_port     = estimator,
  est_acceleration_port = estimator,
  est_global_offset_port= estimator,
  ref_pose_port         = reference,
  ref_ffw_port          = reference,
  cmd_velocity_port     = velocitycmd
  --add here portname = componentname
}

--Packages to import
local packages_to_import = {
  [estimator]       = 'EstimatorInterface',
  [controller]      = 'ControllerInterface',
  [pathgenerator]   = 'PathGeneratorInterface',
  [reference]       = 'Reference',
  [velocitycmd]     = 'VelocityCommandInterface',
  [sensors]         = 'Sensors'
  --add here componentname = 'parentcomponenttype'
}

--Configuration files to load
local system_config_file      = 'Configuration/system.cpf'
local component_config_files  = {
  --add here componentname = 'Configuration/component-config.cpf'
}

local components = {}
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

            -- if coordinator or Teensy, make it server
            if (name == coordinator) or (name == sensors) then
              if (not dp:server(name,true)) then rfsm.send_events(fsm,'e_failed') return end
            end

            --If distributed working, make controller/estimator/pathgenerator server
            if (distributed==true) then
              if (name == controller or name == estimator or name == pathgenerator) then
                if (not dp:server(name,true)) then rfsm.send_events(fsm,'e_failed') return end
              end
            end

          end
        end
      },

      rfsm.transition {src = 'load_components', tgt = 'configure_components', events = {'e_done'}},

      configure_components = rfsm.state {
        entry = function(fsm)
          for name,comp in pairs(components) do
            if (name ~= coordinator and name ~= reporter) then --coordinator/reporter is configured in a later stadium
              comp:loadService('marshalling')
              --Every component loads system configurations
              if (not comp:provides('marshalling'):loadProperties(system_config_file)) then rfsm.send_events(fsm,'e_failed') return end
              --If available, a component loads its specific configurations
              if(component_config_files[name]) then
                if (not comp:provides('marshalling'):loadProperties(component_config_files[name])) then rfsm.send_events(fsm,'e_failed') return end
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
          if (not dp:connectPorts(sensors,estimator))        then rfsm.send_events(fsm,'e_failed') return end
          if (not dp:connectPorts(estimator,controller))     then rfsm.send_events(fsm,'e_failed') return end
          if (not dp:connectPorts(reference,controller))     then rfsm.send_events(fsm,'e_failed') return end
          if (not dp:connectPorts(reference,pathgenerator))  then rfsm.send_events(fsm,'e_failed') return end
          if (not dp:connectPorts(estimator,pathgenerator))  then rfsm.send_events(fsm,'e_failed') return end
          if (not dp:connectPorts(velocitycmd,sensors))      then rfsm.send_events(fsm,'e_failed') return end
            --add more connections here

          --Add every component as peer of coordinator
          if (not dp:addPeer(coordinator,controller))   then rfsm.send_events(fsm,'e_failed') return end
          if (not dp:addPeer(coordinator,estimator))    then rfsm.send_events(fsm,'e_failed') return end
          if (not dp:addPeer(coordinator,pathgenerator))then rfsm.send_events(fsm,'e_failed') return end
          if (not dp:addPeer(coordinator,reference))    then rfsm.send_events(fsm,'e_failed') return end
          if (not dp:addPeer(coordinator,sensors))      then rfsm.send_events(fsm,'e_failed') return end
          if (not dp:addPeer(coordinator,velocitycmd))  then rfsm.send_events(fsm,'e_failed') return end
          if (not dp:addPeer(coordinator,reporter))     then rfsm.send_events(fsm,'e_failed') return end
            --add more peers here
        end,
      },

      rfsm.transition {src = 'connect_components', tgt = 'connect_distr_components', events={'e_done'}},

      connect_distr_components = rfsm.state{
        entry = function(fsm)
          if distributed then
            dir_tbl = {[1]='L', [2]='R'}
            nghb_cnt = 1
            --Load components over the network & connect ports
            for i=0,neighbours.size-1 do
              --Estimator
              if (not dp:loadComponent('estimator'..neighbours[i],'CORBA')) then rfsm.send_events(fsm,'e_failed') return end
              if (not dp:connect('estimator'..index..'.com_in'..dir_tbl[nghb_cnt]..'_port', 'estimator'..neighbours[i]..'.est_pose_port', cp)) then rfsm.send_events(fsm,'e_failed') return end

              --Controller
              if (not dp:loadComponent('controller'..neighbours[i],'CORBA')) then rfsm.send_events(fsm,'e_failed') return end
              if (not dp:connect('controller'..index..'.com_in'..dir_tbl[nghb_cnt]..'_port', 'controller'..neighbours[i]..'.est_pose_port', cp)) then rfsm.send_events(fsm,'e_failed') return end

              --Pathgenerator
              if (not dp:loadComponent('pathgenerator'..neighbours[i],'CORBA')) then rfsm.send_events(fsm,'e_failed') return end
              if (not dp:connect('pathgenerator'..index..'.ref_in'..dir_tbl[nghb_cnt]..'_path_x_port', 'controller'..neighbours[i]..'.ref_pose_path_x_port', cp)) then rfsm.send_events(fsm,'e_failed') return end
              if (not dp:connect('pathgenerator'..index..'.ref_in'..dir_tbl[nghb_cnt]..'_path_y_port', 'controller'..neighbours[i]..'.ref_pose_path_y_port', cp)) then rfsm.send_events(fsm,'e_failed') return end
              if (not dp:connect('pathgenerator'..index..'.ref_in'..dir_tbl[nghb_cnt]..'_path_t_port', 'controller'..neighbours[i]..'.ref_pose_path_t_port', cp)) then rfsm.send_events(fsm,'e_failed') return end

              nghb_cnt = nghb_cnt + 1
            end
          end
        end
      },

      rfsm.transition {src = 'connect_distr_components', tgt = 'set_activities', events = {'e_done'}},

      set_activities = rfsm.state {
        entry = function(fsm)
          dp:setActivity(sensors,1./control_sample_rate,10,rtt.globals.ORO_SCHED_RT)
          dp:setActivity(coordinator,1./control_sample_rate,10,rtt.globals.ORO_SCHED_RT)
          dp:setActivity(pathgenerator,1./pathupd_sample_rate,10,rtt.globals.ORO_SCHED_RT)
          dp:setActivity(velocitycmd,1./velcmd_sample_rate,10,rtt.globals.ORO_SCHED_RT)
          dp:setActivity(reporter,0,2,rtt.globals.ORO_SCHED_RT)
            --add here extra activities
          --The estimator, controller and reference component are triggered by the coordinator and executed in the same thread.
          dp:setMasterSlaveActivity(coordinator,estimator)
          dp:setMasterSlaveActivity(coordinator,controller)
          dp:setMasterSlaveActivity(coordinator,reference)
        end,
      },

      rfsm.transition {src = 'set_activities', tgt = 'load_app_coordination', events = {'e_done'}},

      load_app_coordination = rfsm.state {
        entry = function(fsm)
          --Try to load the coordinator component, drop out if it fails
          if not components[coordinator]:exec_file(coordinator_file) then rfsm.send_events(fsm,'e_failed') return end

          --Copy the values of the application properties into the coordinator component
          components[coordinator]:getProperty('index'):set(index)

          --Try to configure the coordinator, drop out if it fails
          if not components[coordinator]:configure() then rfsm.send_events(fsm,'e_failed') return end

          --Try to connect deployer to coordinator
          if not _fsm_event_port:connect(components[coordinator]:getPort('failure_event_port')) then rfsm.send_events(fsm,'e_failed') return end

          --Try to load the local application script
          components[coordinator]:loadService("scripting")
          if not components[coordinator]:provides("scripting"):loadPrograms(app_file) then rfsm.send_events(fsm,'e_failed') return end

          --Try to start the coordinator
          if not components[coordinator]:start() then rfsm.send_events(fsm,'e_failed') return end
        end,
      },

      rfsm.transition {src = 'load_app_coordination', tgt = 'prepare_reporter', events = {'e_done'}},

      prepare_reporter = rfsm.state{
        entry = function(fsm)
          --Load the marshalling service and the previously defined configuration file
          components[reporter]:loadService('marshalling')
          components[reporter]:provides('marshalling'):loadProperties('Configuration/reporter-config.cpf')

          --Add components to report
          for port,comp in pairs(ports_to_report) do
            if (not dp:addPeer(reporter,comp)) then rfsm.send_events(fsm,'e_failed') end
            if (not components[reporter]:reportPort(comp,port)) then rfsm.send_events(fsm,'e_failed') end
          end

          -- Try to configure the reporter, drop out if it fails
          if (not components[reporter]:configure()) then rfsm.send_events(fsm,'e_failed') return end

        end,
      },
   },

  --Deployment is finished.
  rfsm.transition {src = '.deploy.prepare_reporter', tgt='deployed'}, --, events = {'e_done'}},

  deployed = rfsm.conn{},

  --Currently there is no logic to failure from a failed deployement
  failure = rfsm.state {
    exit = function()
      for name,type in pairs(components_to_load) do
          dp:unloadComponent(name)
      end
    end,
   },

   rfsm.transition { src = 'failure', tgt = 'deploy', events = {'e_reset'}},
}

