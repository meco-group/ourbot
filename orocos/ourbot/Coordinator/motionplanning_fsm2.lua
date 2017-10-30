return rfsm.state {
  rfsm.trans{src = 'initial', tgt = 'init'},
  rfsm.trans{src = 'init', tgt = 'home', events = {'e_done'}},
  rfsm.trans{src = 'home', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'idle', tgt = 'p2p0', events = {'e_p2p'}},
  rfsm.trans{src = 'p2p0', tgt = 'idle', events = {'e_idle'}},
  rfsm.trans{src = 'p2p0', tgt = 'p2p', events = {'e_done'}},
  rfsm.trans{src = 'p2p', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'p2p', tgt = 'recover', events = {'e_recover'}},
  rfsm.trans{src = 'recover', tgt = 'p2p0', events = {'e_p2p'}},
  rfsm.trans{src = 'recover', tgt = 'idle', events = {'e_done'}},
  rfsm.trans{src = 'home', tgt = 'stop', events = {'e_back'}},
  rfsm.trans{src = 'idle', tgt = 'stop', events = {'e_back'}},
  rfsm.trans{src = 'p2p0', tgt = 'idle', events = {'e_back'}},
  rfsm.trans{src = 'p2p', tgt = 'idle', events = {'e_back'}},
  rfsm.trans{src = 'recover', tgt = 'idle', events = {'e_back'}},

  initial = rfsm.conn{},

  init = rfsm.state{
    entry = function(fsm)
      if not motionplanning_coor.deployer:loadComponent('motionplanning', 'MotionPlanning') then
        rfsm.send_events(fsm, 'e_failed')
      end
      if not motionplanning_coor:set_motionplanner(motionplanning_coor.deployer:getPeer('motionplanning')) then
        rfsm.send_events(fsm, 'e_failed')
        return
      end
      motionplanning_coor.motionplanning:start()
      if not motionplanning_coor.control_coor:start() then
        rfsm.send_events(fsm, 'e_failed')
      end
    end
  },

  home = rfsm.state{
    -- wait until valid estimate
    doo = function(fsm)
      while true do
        if not motionplanning_coor.control_coor:control_hook(false) then
          rfsm.send_events(fsm, 'e_failed')
        end
        if true or motionplanning_coor.control_coor.estimator_valid() then
          return
        end
        rfsm.yield(true)
      end
    end
  },

  idle = rfsm.state{
    entry = function(fsm)
      motionplanning_coor.control_coor:zero_velocity()
      motionplanning_coor.control_coor:enable_manualcommand()
    end,

    doo = function(fsm)
      while true do
        if not motionplanning_coor.control_coor:control_hook(false) or motionplanning_coor.motionplanning_error() then
          rfsm.send_events(fsm, 'e_failed')
        end
        local fs, target_pose = motionplanning_coor.target_pose_port:read()
        if fs == 'NewData' then
          motionplanning_coor.mp_reset()
          motionplanning_coor.motionplanning:setTargetPose(target_pose)
          rfsm.send_events(fsm, 'e_p2p')
        end
        rfsm.yield(true)
      end
    end,
  },

  p2p0 = rfsm.state{

    doo = function(fsm)
      motionplanning_coor.control_coor:disable_manualcommand()
      local busy = motionplanning_coor.mp_busy()
      local busy_n
      motionplanning_coor:trigger_motionplanning(0)
      motionplanning_coor.ref_cnt = 0
      motionplanning_coor.mp_failure_cnt = 0
      motionplanning_coor.mp_recover_cnt = 0
      while true do
        if not motionplanning_coor.control_coor:control_hook(false) or motionplanning_coor.motionplanning_error() then
          rfsm.send_events(fsm, 'e_failed')
        end
        busy_n = motionplanning_coor.mp_busy()
        if (busy or busy_n and busy ~= busy_n) and not busy_n then -- decreasing flank detection
          if motionplanning_coor.mp_ready() then
            print 'Target reached.'
            rfsm.send_events(fsm, 'e_idle')
            return
          end
          if motionplanning_coor.mp_valid() then
            motionplanning_coor.load_new_trajectory(0)
            return
          else
            rtt.logl('Error', 'Motionplanning could not find a trajectory.')
            rfsm.send_events(fsm, 'e_idle')
            return
          end
        end
        busy = busy_n
        rfsm.yield(true)
      end
    end,
  },

  p2p = rfsm.state{
    doo = function(fsm)
      local predict_shift = 0
      while true do

        if motionplanning_coor.ref_cnt == 0 then
          motionplanning_coor:trigger_motionplanning(predict_shift)
        end
        if not motionplanning_coor.control_coor:control_hook(true) or motionplanning_coor.motionplanning_error() then
          rfsm.send_events(fsm, 'e_failed')
        end
        -- check motion planning
        if motionplanning_coor.mp_ready() then
          print 'Target reached.'
          return
        end
        -- motion planning check is related to next update
        motionplanning_coor.ref_cnt = motionplanning_coor.ref_cnt + 1
        -- e.g. new trajectories are followed, beginning in the nex update
        -- This behavior is similar as putting thte motion planning check in
        -- start of update. But using the current implementation,  the
        -- control_hook update is synced/clocked better.
        if not motionplanning_coor.mp_busy() and motionplanning_coor.ref_cnt >= motionplanning_coor.mp_period then
          if motionplanning_coor.mp_valid() then
            predict_shift = math.max(0, motionplanning_coor.ref_cnt - motionplanning_coor.mp_period)
            motionplanning_coor.load_new_trajectory(predict_shift)
            motionplanning_coor.ref_cnt = 0
          else
            -- try again
            motionplanning_coor.mp_failure_cnt = motionplanning_coor.mp_failure_cnt + 1
            if motionplanning_coor.mp_failure_cnt < motionplanning_coor.mp_max_failures then
              motionplanning_coor:trigger_motionplanning(math.max(0, motionplanning_coor.ref_cnt - motionplanning_coor.mp_period))
            else
              rtt.logl('Error', 'Motionplanning got ' .. motionplanning_coor.mp_max_failures .. ' consecutive invalid solutions. Recover...')
              rfsm.send_events(fsm, 'e_recover')
              return
            end
          end
        end
        if motionplanning_coor.ref_cnt > motionplanning_coor.mp_max_periods*motionplanning_coor.mp_period then
          rtt.logl('Error', 'Motionplanning takes too long. Recover...')
          rfsm.send_events(fsm, 'e_recover')
          return
        end
        if motionplanning_coor.ref_cnt >= motionplanning_coor.mp_period and math.fmod(motionplanning_coor.ref_cnt, motionplanning_coor.mp_period) == 0 then
          rtt.logl('Warning', 'Motionplanning takes longer than '.. math.floor(motionplanning_coor.ref_cnt/motionplanning_coor.mp_period) ..' period(s)!')
        end
        rfsm.yield(true)
      end
    end,
  },

  recover = rfsm.state{
    doo = function(fsm)
      motionplanning_coor.mp_recover_cnt = motionplanning_coor.mp_recover_cnt + 1
      if motionplanning_coor.mp_recover_cnt >= motionplanning_coor.mp_max_recovers then
        rtt.logl('Error', 'Motionplanning recovered ' .. motionplanning_coor.mp_max_recovers .. ' times. Giving up...')
        return
      end
      local time0 = get_sec()
      local recover_time = 1.
      while true do
        if not motionplanning_coor.control_coor:control_hook(false) or motionplanning_coor.motionplanning_error() then
          rfsm.send_events(fsm, 'e_failed')
        end
        -- wait a little bit and try again
        if get_sec() - time0 >= recover_time then
          rfsm.send_events(fsm, 'e_p2p')
        end
        rfsm.yield(true)
      end
    end,
  },

  stop = rfsm.state {
    entry = function(fsm)
      motionplanning_coor:rm_motionplanner()
      motionplanning_coor.control_coor:stop()
    end,
  },
}
