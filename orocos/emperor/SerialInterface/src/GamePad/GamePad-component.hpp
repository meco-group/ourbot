#ifndef GAMEPAD_H
#define GAMEPAD_H

#define GAMEPAD_TESTFLAG
#define GAMEPAD_DEBUGFLAG

#ifdef GAMEPAD_DEBUGFLAG
  #define GAMEPAD_DEBUG_PRINT(x)  std::cout << x << std::endl;
#else
  #define GAMEPAD_DEBUG_PRINT(x)  //std::cout << x << std::endl;
#endif

#include "../USBInterface/USBInterface-component.hpp"
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <vector>

#include "GamePad_protocol.h"
#include "GamePad_cmd.h"

class GamePad : public USBInterface
{
  private:
    RTT::OutputPort<std::vector<double> > _gamepad_laxis_port;
    RTT::OutputPort<std::vector<double> > _gamepad_raxis_port;
    RTT::OutputPort<bool > _gamepad_A_port;
    RTT::OutputPort<bool > _gamepad_B_port;
    RTT::OutputPort<bool > _gamepad_X_port;
    RTT::OutputPort<bool > _gamepad_Y_port;
    RTT::OutputPort<bool > _gamepad_back_port;
    RTT::OutputPort<bool > _gamepad_start_port;
    RTT::OutputPort<bool > _gamepad_logitech_port;
    RTT::OutputPort<bool > _gamepad_laxisbutton_port;
    RTT::OutputPort<bool > _gamepad_raxisbutton_port;
    RTT::OutputPort<bool > _gamepad_up_port;
    RTT::OutputPort<bool > _gamepad_down_port;
    RTT::OutputPort<bool > _gamepad_left_port;
    RTT::OutputPort<bool > _gamepad_right_port;
    RTT::OutputPort<bool > _gamepad_lb_port;
    RTT::OutputPort<bool > _gamepad_rb_port;
    RTT::OutputPort<double > _gamepad_lt_port;
    RTT::OutputPort<double > _gamepad_rt_port;

    gamepad_event_t _event;
    std::vector<double> _laxis;
    std::vector<double> _raxis;
    void decodeButtons();
    void decodeAxes();
    double transformData(int);

  public:
    GamePad(std::string const& name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
};

#endif //GAMEPAD_H
