#ifndef GAMEPAD_H
#define GAMEPAD_H

// #define GAMEPAD_TESTFLAG
// #define GAMEPAD_DEBUGFLAG

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

using namespace RTT;

class GamePad : public USBInterface
{
  private:
    OutputPort<std::vector<double> > _gamepad_laxis_port;
    OutputPort<std::vector<double> > _gamepad_raxis_port;
    OutputPort<bool > _gamepad_A_port;
    OutputPort<bool > _gamepad_B_port;
    OutputPort<bool > _gamepad_X_port;
    OutputPort<bool > _gamepad_Y_port;
    OutputPort<bool > _gamepad_back_port;
    OutputPort<bool > _gamepad_start_port;
    OutputPort<bool > _gamepad_logitech_port;
    OutputPort<bool > _gamepad_laxisbutton_port;
    OutputPort<bool > _gamepad_raxisbutton_port;
    OutputPort<bool > _gamepad_up_port;
    OutputPort<bool > _gamepad_down_port;
    OutputPort<bool > _gamepad_left_port;
    OutputPort<bool > _gamepad_right_port;
    OutputPort<bool > _gamepad_lb_port;
    OutputPort<bool > _gamepad_rb_port;
    OutputPort<double > _gamepad_lt_port;
    OutputPort<double > _gamepad_rt_port;

    std::vector<double> _gamepad_laxis;
    std::vector<double> _gamepad_raxis;
    bool _gamepad_A;
    bool _gamepad_B;
    bool _gamepad_X;
    bool _gamepad_Y;
    bool _gamepad_back;
    bool _gamepad_start;
    bool _gamepad_logitech;
    bool _gamepad_laxisbutton;
    bool _gamepad_raxisbutton;
    bool _gamepad_up;
    bool _gamepad_down;
    bool _gamepad_left;
    bool _gamepad_right;
    bool _gamepad_lb;
    bool _gamepad_rb;
    double _gamepad_lt;
    double _gamepad_rt;

    bool _enable_velcmd;

    gamepad_event_t _event;
    void writePorts();
    void decodeButtons();
    void decodeAxes();
    double transformData(int);

    // velocity command related attributes
    OutputPort<std::vector<double> > _cmd_velocity_port;
    std::vector<double> _cmd_velocity;
    std::vector<double> _filter_state;
    double _filter_bandwidth;
    double _max_velocity;
    double _max_omega;
    double _velcmd_sample_rate;
    bool _enable_LPF;
    void VelocityCommand();
    std::vector<double> lowPassFilter(std::vector<double>);
    double treshold(double);

  public:
    GamePad(std::string const& name);
    void setVelocity(double, double, double);
    void enableVelocityCmd();
    void disableVelocityCmd();
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
};

#endif //GAMEPAD_H
