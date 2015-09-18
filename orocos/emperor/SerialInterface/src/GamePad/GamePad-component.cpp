#include "GamePad-component.hpp"
#include <iostream>
#include <math.h>

GamePad::GamePad(std::string const& name):USBInterface(name)
{
  this->ports()->addPort("gamepad_laxis_port", _gamepad_laxis_port).doc("X and Y value for left axis");
  this->ports()->addPort("gamepad_raxis_port", _gamepad_laxis_port).doc("X and Y value for right axis");
  this->ports()->addPort("gamepad_A_port", _gamepad_A_port).doc("Bool for A button");
  this->ports()->addPort("gamepad_B_port", _gamepad_B_port).doc("Bool for B button");
  this->ports()->addPort("gamepad_X_port", _gamepad_X_port).doc("Bool for X button");
  this->ports()->addPort("gamepad_Y_port", _gamepad_Y_port).doc("Bool for Y button");
  this->ports()->addPort("gamepad_back_port", _gamepad_back_port).doc("Bool for back button");
  this->ports()->addPort("gamepad_start_port", _gamepad_start_port).doc("Bool for start button");
  this->ports()->addPort("gamepad_logitech_port", _gamepad_logitech_port).doc("Bool for logitech button");
  this->ports()->addPort("gamepad_laxisbutton_port", _gamepad_laxisbutton_port).doc("Bool for button of left axis");
  this->ports()->addPort("gamepad_raxisbutton_port", _gamepad_raxisbutton_port).doc("Bool for button of right axis");
  this->ports()->addPort("gamepad_up_port", _gamepad_up_port).doc("Bool for up button");
  this->ports()->addPort("gamepad_down_port", _gamepad_down_port).doc("Bool for down button");
  this->ports()->addPort("gamepad_left_port", _gamepad_left_port).doc("Bool for left button");
  this->ports()->addPort("gamepad_right_port", _gamepad_right_port).doc("Bool for right button");
  this->ports()->addPort("gamepad_lb_port", _gamepad_lb_port).doc("Bool for lb button");
  this->ports()->addPort("gamepad_rb_port", _gamepad_rb_port).doc("Bool for rb button");
  this->ports()->addPort("gamepad_lt_port", _gamepad_lt_port).doc("Double for lt button");
  this->ports()->addPort("gamepad_rt_port", _gamepad_rt_port).doc("Double for rt button");


  // addOperation("deviceInfo", &GAMEPAD::deviceInfo, this).doc("Send request to the lidar to send its device info.");
  // addProperty("lidar_angle_offset", _angle_offset).doc("Angular offset of the lidar in radians.");
}

// bool GamePad::sendCommand(uint8_t cmd, const void *payload, uint32_t payloadsize)
// {
//   uint8_t pkt_header[10];
//   GAMEPAD_cmd_packet_t * header = reinterpret_cast<GAMEPAD_cmd_packet_t * >(pkt_header);
//   uint8_t checksum = 0;
//   uint8_t header_sent;

//   if (!isConnectedSerial()){ RTT::log(RTT::Warning) << "Cannot send command to the lidar." << RTT::endlog(); return false; }

//   if (payloadsize && payload) {
//       cmd |= GAMEPAD_CMDFLAG_HAS_PAYLOAD;
//   }

//   header->syncByte = GAMEPAD_CMD_SYNC_BYTE;
//   header->cmd_flag = cmd;

//   // send header first
//   header_sent = writeBytes(pkt_header, 2) ;

//   if (cmd & GAMEPAD_CMDFLAG_HAS_PAYLOAD) {
//       checksum ^= GAMEPAD_CMD_SYNC_BYTE;
//       checksum ^= cmd;
//       checksum ^= (payloadsize & 0xFF);

//       // calc checksum
//       for (size_t pos = 0; pos < payloadsize; ++pos) {
//           checksum ^= ((uint8_t *)payload)[pos];
//       }

//       // send size
//       uint8_t sizebyte = payloadsize;
//       writeByte(sizebyte);

//       // send payload
//       writeBytes((uint8_t*)payload, sizebyte);

//       // send checksum
//       writeByte(checksum);
//   }

//   if(cmd == GAMEPAD_CMD_RESET){ _state = RESETTING; }
//   else if(cmd == GAMEPAD_CMD_STOP) { _state = STOP_SCANNING; }
//   else { _state = PENDING; }
//   return (header_sent>0);
// }

bool GamePad::configureHook()
{
  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(2, 0.0);
  _gamepad_laxis_port.setDataSample(example);
  _gamepad_raxis_port.setDataSample(example);

#ifndef GAMEPAD_TESTFLAG
  return true;
#else
  _usb_port_name = "/dev/input/js0";
  return setPeriod(0.01);
#endif //GAMEPAD_TESTFLAG
}

bool GamePad::startHook()
{
  return USBInterface::startHook();
}

void GamePad::updateHook()
{
  //do nothing
  int numbytes = readBytes((uint8_t*)&_event, sizeof(struct gamepad_event_t));

  if(numbytes>0){
    GAMEPAD_DEBUG_PRINT("Bytes received: " << numbytes)
  }

  //   //decode the buffer while bytes are available
  //   numbytes += _buffer_offset;
  //   uint32_t decoding_offset = 0;
  //   uint32_t bytes_decoded = 0;
  //   do{
  //     switch(_state){
  //       case IDLE:
  //         GAMEPAD_DEBUG_PRINT("GAMEPAD internal error: received bytes which were not expected.")
  //         break;
  //       case PENDING:
  //         bytes_decoded = handleRequest(_buffer + decoding_offset, numbytes);
  //         break;
  //       case SCANNING:
  //         bytes_decoded = handleScan(_buffer + decoding_offset, numbytes);
  //         break;
  //       case STOP_SCANNING:
  //         bytes_decoded = handleScan(_buffer + decoding_offset, numbytes);
  //         break;
  //       case RESETTING:
  //         bytes_decoded = numbytes;
  //         _request_status = RESET;
  //         _state = IDLE;
  //         break;
  //     }
  //     decoding_offset += bytes_decoded;
  //     numbytes -= bytes_decoded;

  //     GAMEPAD_DEBUG_PRINT("Decoding offset " << decoding_offset)
  //     GAMEPAD_DEBUG_PRINT("Numbytes: " << numbytes)
  //   }while((bytes_decoded>0)&&(numbytes>0));

  //   //copy the remainder to the beginning of the buffer
  //   if(numbytes>0){
  //     memcpy(_buffer, _buffer + decoding_offset, numbytes);
  //   }
  //   _buffer_offset = numbytes;
  //   GAMEPAD_DEBUG_PRINT("Remaining bytes in the buffer: " << _buffer_offset)
  // } else if( _state == STOP_SCANNING ){
  //   //Scanning has stopped: go to idle
  //   _state = IDLE;
  // }
}

void GamePad::stopHook()
{
  USBInterface::stopHook();
}


ORO_LIST_COMPONENT_TYPE(GamePad)
