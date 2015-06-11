
function enterIdle()
  print "Entered idle state"
end

function enterRun()
  print "Entered run state"
  local tc=rtt.getTC();

  local test = tc:getPeer("controller")
  op = test:getOperation("start")
  op()
end

function enterIdentify()

end

function exitIdentify()

end

return rfsm.state {
  idle = rfsm.state {
    entry = enterIdle,
  },

  run = rfsm.state {
      entry = enterRun,
  },

  identify = rfsm.state {
      entry = enterIdentify,
      exit = exitIdentify,
  },

  rfsm.trans {src="initial", tgt="idle" },
  rfsm.trans {src="idle", tgt="run", events={"e_run"}},
  rfsm.trans {src="run", tgt="idle", events={"e_idle"}},
  rfsm.trans {src="idle", tgt="identify", events={"e_identify"}},
  rfsm.trans {src="identify", tgt="idle", events={"e_idle"}},
}


