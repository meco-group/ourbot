#include "RPLidar-component.hpp"
#include <iostream>
#include <math.h>

RPLidar::RPLidar(std::string const& name) :
	USBInterface(name), _angle_offset(0.05), _state(IDLE), _buffer_offset(0), _lidar_data_length(RPLIDAR_NODE_BUFFER_SIZE), _node_buffer_fill(0), _primary_node_buffer(_node_buffer1), _secondary_node_buffer(_node_buffer2),
	_node_buffer1{std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE)},
	_node_buffer2{std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE),std::vector<double>(RPLIDAR_NODE_BUFFER_SIZE)}
{
  this->ports()->addPort( "cal_enc_pose_port", _cal_enc_pose_port ).doc("Encoder pose input port. This is used to give encoder stamps to the data and to transform local node information to the global node information.");

	this->ports()->addPort( "cal_lidar_angle_port", _cal_lidar_angle_port ).doc( "Output port for calibrated lidar angle. Holds a vector of RPLIDAR_NODE_BUFFER_SIZE angle values [rad]." );
	this->ports()->addPort( "cal_lidar_distance_port", _cal_lidar_distance_port ).doc( "Output port for calibrated lidar node distances. Holds a vector of RPLIDAR_NODE_BUFFER_SIZE distances [m]." );
	this->ports()->addPort( "cal_lidar_x_port", _cal_lidar_x_port ).doc( "Output port for calibrated lidar node positions. Holds a vector of RPLIDAR_NODE_BUFFER_SIZE x-coordinates [m]." );
	this->ports()->addPort( "cal_lidar_y_port", _cal_lidar_y_port ).doc( "Output port for calibrated lidar node positions. Holds a vector of RPLIDAR_NODE_BUFFER_SIZE y-coordinates [m]." );
	this->ports()->addPort( "cal_lidar_quality_port", _cal_lidar_quality_port ).doc( "Output port for node quality. value between 0 and 1" );
	this->ports()->addPort( "cal_lidar_encoder_stamp_x_port", _cal_lidar_encoder_stamp_x_port ).doc( "Output port for lidar calibrated encoder stamps. Holds a vector of RPLIDAR_NODE_BUFFER_SIZE x-coordinates [m] wrt to the local robot frame." );
	this->ports()->addPort( "cal_lidar_encoder_stamp_y_port", _cal_lidar_encoder_stamp_y_port ).doc( "Output port for lidar calibrated encoder stamps. Holds a vector of RPLIDAR_NODE_BUFFER_SIZE y-coordinates [m] wrt to the local robot frame." );
	this->ports()->addPort( "cal_lidar_encoder_stamp_angle_port", _cal_lidar_encoder_stamp_angle_port ).doc( "Output port for lidar calibrated encoder stamps. Holds a vector of RPLIDAR_NODE_BUFFER_SIZE angles [rad] wrt to the initial frame." );
	this->ports()->addPort( "cor_lidar_distance_port",_cor_lidar_distance_port).doc("output port for corrected lidar node positions. HOlds a vector of rplidar_node_buffer_size x-coordinates [m]");
    this->ports()->addPort( "cor_lidar_angle_port",_cor_lidar_angle_port).doc("output port for corrected lidar node positions. HOlds a vector of rplidar_node_buffer_size y-coordinates [m]");
	this->ports()->addPort( "cal_lidar_local_node_port", _cal_lidar_local_node_port ).doc( "Output port for the calibrated local lidar nodes [m,m,-]" );
	this->ports()->addPort( "cal_lidar_global_node_port", _cal_lidar_global_node_port ).doc( "Output port for the calibrated global lidar nodes [m,m,-]" );

	addOperation("deviceInfo", &RPLidar::deviceInfo, this).doc("Send request to the lidar to send its device info.");
	addOperation("deviceHealth", &RPLidar::deviceHealth, this).doc("Send request to the lidar to send its health.");
	addOperation("deviceReset", &RPLidar::deviceReset, this).doc("Send request to the lidar to reset.");
	addOperation("startScan", &RPLidar::startScan, this).doc("Starts the lidar. Returns true if the command has been carried out successfully");
	addOperation("stopScan", &RPLidar::stopScan, this).doc("Stops the lidar. Returns true if the command has been carried out successfully");
	addOperation("showState", &RPLidar::showState, this).doc("Return the state in which the lidar currently operates.");

	addProperty("lidar_angle_offset", _angle_offset).doc("Angular offset of the lidar in radians.");
	addProperty("lidar_data_length", _lidar_data_length).doc("Length of the lidar data.");
}

bool RPLidar::sendCommand(uint8_t cmd, const void *payload, uint32_t payloadsize)
{
	uint8_t pkt_header[10];
  rplidar_cmd_packet_t * header = reinterpret_cast<rplidar_cmd_packet_t * >(pkt_header);
  uint8_t checksum = 0;
  uint8_t header_sent;

  if (!isConnectedSerial()){ RTT::log(RTT::Warning) << "Cannot send command to the lidar." << RTT::endlog(); return false; }

  if (payloadsize && payload) {
      cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
  }

  header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
  header->cmd_flag = cmd;

  // send header first
  header_sent = writeBytes(pkt_header, 2) ;

  if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
      checksum ^= RPLIDAR_CMD_SYNC_BYTE;
      checksum ^= cmd;
      checksum ^= (payloadsize & 0xFF);

      // calc checksum
      for (size_t pos = 0; pos < payloadsize; ++pos) {
          checksum ^= ((uint8_t *)payload)[pos];
      }

      // send size
      uint8_t sizebyte = payloadsize;
      writeByte(sizebyte);

      // send payload
      writeBytes((uint8_t*)payload, sizebyte);

      // send checksum
      writeByte(checksum);
  }

  if(cmd == RPLIDAR_CMD_RESET){ _state = RESETTING; }
  else if(cmd == RPLIDAR_CMD_STOP) { _state = STOP_SCANNING; }
  else { _state = PENDING; }
  return (header_sent>0);
}

int RPLidar::handleRequest(uint8_t* buffer, uint32_t numbytes)
{
	_request_status = DECODING;
	uint32_t bytes_decoded = 0;
	uint32_t offset = 0;
	while(((numbytes-offset)>=sizeof(rplidar_ans_header_t)) && (_request_status == DECODING)){ //iterate all bits to check for start bits
		if((buffer[offset] == RPLIDAR_ANS_SYNC_BYTE1)&&(buffer[offset+1] == RPLIDAR_ANS_SYNC_BYTE2)){
			//answer found. Let's handle the message now
			RPLIDAR_DEBUG_PRINT("Received valid message from the rplidar.")
			rplidar_ans_header_t* answer_header = reinterpret_cast<rplidar_ans_header_t *>(buffer+offset);
			_request_status = INVALID;

			switch(answer_header->type){
				case RPLIDAR_ANS_TYPE_DEVHEALTH:{
					// handle the device info message
		      if((answer_header->size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) >= sizeof(rplidar_response_device_health_t)){
		      	if((numbytes - offset - sizeof(rplidar_ans_header_t)) >= sizeof(rplidar_response_device_health_t)){
		      		_health = *(reinterpret_cast<rplidar_response_device_health_t *>(buffer+offset+sizeof(rplidar_ans_header_t)));
		      		showDeviceHealth();
		      		_state = IDLE; // change state to IDLE
		      		_request_status = DEVICE_HEALTH;
		      		bytes_decoded = offset + sizeof(rplidar_ans_header_t) + sizeof(rplidar_response_device_health_t);
		      	} else { _request_status = PARTIAL; }
		      } else {
		      	RPLIDAR_DEBUG_PRINT("Received DEVHEALTH packet from the rplidar but the SIZE MASK did not match.")
		      }
					break;}

				case RPLIDAR_ANS_TYPE_DEVINFO:{
					// handle the device info message
		      if((answer_header->size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) >= sizeof(rplidar_response_device_info_t)){
		      	if((numbytes - offset - sizeof(rplidar_ans_header_t)) >= sizeof(rplidar_response_device_info_t)){
				    	_info = *(reinterpret_cast<rplidar_response_device_info_t *>(buffer+offset+sizeof(rplidar_ans_header_t)));
				    	showDeviceInfo();
				    	_state = IDLE; // change state to IDLE
				    	_request_status = DEVICE_INFO;
		      		bytes_decoded = offset + sizeof(rplidar_ans_header_t) + sizeof(rplidar_response_device_info_t);
				    } else { _request_status = PARTIAL; }
		      } else {
		      	RPLIDAR_DEBUG_PRINT("Received DEVINFO packet from the rplidar but the SIZE MASK did not match.")
		      }
					break;}

				case RPLIDAR_ANS_TYPE_MEASUREMENT:{
					// handle the device info message
		      if((answer_header->size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) >= sizeof(rplidar_response_measurement_node_t)){
		      	if((numbytes - offset - sizeof(rplidar_ans_header_t)) >= sizeof(rplidar_response_measurement_node_t)){
				    	_info = *(reinterpret_cast<rplidar_response_device_info_t *>(buffer+offset+sizeof(rplidar_ans_header_t)));
				    	_state = SCANNING; // change state to SCANNING
				    	_request_status = MEASUREMENT_NODE;
		      		bytes_decoded = offset + sizeof(rplidar_ans_header_t) + sizeof(rplidar_response_measurement_node_t);
				    } else { _request_status = PARTIAL; }
		      } else {
		      	RPLIDAR_DEBUG_PRINT("Received MEASUREMENT NODE packet from the rplidar but the SIZE MASK did not match.")
		      }
					break;}

				default:
					break;
			}
		}
	}

	return bytes_decoded;
}

int RPLidar::handleScan(uint8_t* buffer, uint32_t numbytes)
{
	_measurement_status = DECODING;
	uint32_t bytes_decoded = 0;
	uint32_t offset = 0;
	while(((numbytes-offset)>=sizeof(rplidar_response_measurement_node_t)) && (_measurement_status == DECODING)){ //iterate all bits to check for start bits
		if(((buffer[offset] ^ (buffer[offset]>>1)) & 0x1) && (buffer[offset+1] & RPLIDAR_RESP_MEASUREMENT_CHECKBIT)){
			//answer found. Let's handle the message now
			//RPLIDAR_DEBUG_PRINT("Received valid measurement from the rplidar.")
			rplidar_response_measurement_node_t* node = reinterpret_cast<rplidar_response_measurement_node_t *>(buffer+offset);
			/*#ifdef RPLIDAR_DEBUGFLAG
				showMeasurement(*node);
			#endif //RPLIDAR_DEBUGFLAG*/
			_measurement_status = MEASUREMENT_NODE;

			addNodeToMeasurements(*node);
		}
	}

	if(_measurement_status == MEASUREMENT_NODE){
		return (offset + sizeof(rplidar_response_measurement_node_t));
	} else {
		return 0;
	}
}

bool RPLidar::configureHook()
{
  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  // set 3D ports
  _cal_lidar_local_node_port.setDataSample(example);
  _cal_lidar_global_node_port.setDataSample(example);

  // set RPLIDAR_NODE_BUFFER_SIZE-D ports
  example.resize(_lidar_data_length);
  _cal_lidar_angle_port.setDataSample(example);
  _cal_lidar_distance_port.setDataSample(example);
  _cal_lidar_x_port.setDataSample(example);
  _cal_lidar_y_port.setDataSample(example);
  _cal_lidar_quality_port.setDataSample(example);
  _cal_lidar_encoder_stamp_x_port.setDataSample(example);
  _cal_lidar_encoder_stamp_y_port.setDataSample(example);
  _cal_lidar_encoder_stamp_angle_port.setDataSample(example);
  _cor_lidar_distance_port.setDataSample(example);
  _cor_lidar_angle_port.setDataSample(example);

#ifndef RPLIDAR_TESTFLAG
	return true;
#else
	_usb_port_name = "/dev/ttyUSB0";
	return setPeriod(0.005);
#endif //RPLIDAR_TESTFLAG
}

bool RPLidar::startHook()
{
	if(USBInterface::startHook()){
		startScan();
		return true;
	} else {
		return false;
	}
}

void RPLidar::updateHook()
{
	//do nothing
	int numbytes = readBytes(_buffer + _buffer_offset, RPLIDAR_BUFFER_SIZE - _buffer_offset);

	if(numbytes>0){
		RPLIDAR_DEBUG_PRINT("Bytes received: " << numbytes)

		//decode the buffer while bytes are available
		numbytes += _buffer_offset;
		uint32_t decoding_offset = 0;
		uint32_t bytes_decoded = 0;
		do{
			switch(_state){
				case IDLE:
					RPLIDAR_DEBUG_PRINT("RPLidar internal error: received bytes which were not expected.")
					break;
				case PENDING:
					bytes_decoded = handleRequest(_buffer + decoding_offset, numbytes);
					break;
				case SCANNING:
					bytes_decoded = handleScan(_buffer + decoding_offset, numbytes);
					break;
				case STOP_SCANNING:
					bytes_decoded = handleScan(_buffer + decoding_offset, numbytes);
					break;
				case RESETTING:
					bytes_decoded = numbytes;
					_request_status = RESET;
					_state = IDLE;
					break;
			}
			decoding_offset += bytes_decoded;
			numbytes -= bytes_decoded;

			//RPLIDAR_DEBUG_PRINT("Decoding offset " << decoding_offset)
			//RPLIDAR_DEBUG_PRINT("Numbytes: " << numbytes)
		}while((bytes_decoded>0)&&(numbytes>0));

		//copy the remainder to the beginning of the buffer
		if(numbytes>0){
			memcpy(_buffer, _buffer + decoding_offset, numbytes);
		}
		_buffer_offset = numbytes;
		RPLIDAR_DEBUG_PRINT("Remaining bytes in the buffer: " << (int)_buffer_offset)
	} else if( _state == STOP_SCANNING ){
		//Scanning has stopped: go to idle
		_state = IDLE;
	}
}

void RPLidar::stopHook()
{
    // stop scanning (if we were scanning)
    if(_state == SCANNING){
	    stopScan();
	}

	// reset the component so we don't get into trouble when restarting the component
    reset();

    // close the usb device
	USBInterface::stopHook();
}

bool RPLidar::deviceInfo()
{
	return sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO);
}

bool RPLidar::deviceHealth()
{
	return sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH);
}

bool RPLidar::deviceReset()
{
	return sendCommand(RPLIDAR_CMD_RESET);
}

bool RPLidar::startScan()
{
	return sendCommand(RPLIDAR_CMD_SCAN);
}

bool RPLidar::stopScan()
{
	return sendCommand(RPLIDAR_CMD_STOP);
}

bool RPLidar::reset()
{
    // send reset request to the lidar
    deviceReset();

    // set the state to idle as we can predict what is going to happen, but cannot rely on the serial data because the link will be broken
    _state = IDLE;

    // set the buffers to their initial states
    _buffer_offset = 0;
    _node_buffer_fill = 0;
    _primary_node_buffer = _node_buffer1;
    _secondary_node_buffer = _node_buffer2;
}

void RPLidar::showState()
{
	switch(_state){
		case IDLE:
			std::cout << "IDLE: waiting for instructions." << std::endl;
			break;
		case PENDING:
			std::cout << "PENDING: waiting for the lidar to return info." << std::endl;
			break;
		case SCANNING:
			std::cout << "SCANNING: scanning the environment." << std::endl;
			break;
		case RESETTING:
			std::cout << "RESETTING: waiting for the reset message to be sent." << std::endl;
			break;
	}
}

void RPLidar::showDeviceInfo()
{
	printf("RPLIDAR S/N: ");
  for(uint32_t k = 0; k < 16 ; k++) {
    printf("%02X", _info.serialnum[k]);
  }

  printf("\n"
         "Firmware Ver: %d.%02d\n"
         "Hardware Rev: %d\n"
         , _info.firmware_version>>8
         , _info.firmware_version & 0xFF
         , (int)_info.hardware_version);
}

void RPLidar::showDeviceHealth()
{
	printf("RPLidar health status : ");
  switch (_health.status) {
  case RPLIDAR_STATUS_OK:
		printf("OK.");
		break;
  case RPLIDAR_STATUS_WARNING:
		printf("Warning.");
		break;
  case RPLIDAR_STATUS_ERROR:
		printf("Error.");
		break;
  }
  printf(" (errorcode: %d)\n", _health.error_code);
}

void RPLidar::showMeasurement(const rplidar_response_measurement_node_t &node)
{
	printf("%s theta: %03.2f Dist: %08.2f Qual: %2d\n",
        (node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
        (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0,
         node.distance_q2*0.00025,
         node.sync_quality && 0x3F);
}

void RPLidar::addNodeToMeasurements(const rplidar_response_measurement_node_t &node)
{
	double angle = (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)*M_PI/(64.0*180.0) - _angle_offset;
	double distance = node.distance_q2*0.00025; //node.distance_q2/4000.0

	if(distance > 0.1){ //check if the node is far enough, otherwise discard measurement
		//angle and distance both get a minus to make the reference frame compliant to the robot frame
		angle = -angle;
		distance = -distance;

		//add node to current buffer
		_primary_node_buffer[0][_node_buffer_fill] = angle;                             //lidar raw angle
		_primary_node_buffer[1][_node_buffer_fill] = distance;		                      //lidar raw distance
		_primary_node_buffer[2][_node_buffer_fill] = distance*cos(angle);               //lidar x value
		_primary_node_buffer[3][_node_buffer_fill] = distance*sin(angle);               //lidar y value
		_primary_node_buffer[4][_node_buffer_fill] = (node.sync_quality && 0x3F)/64.0;  //lidar quality

		// Read encoder data from the encoder port and add to the node as encoder stamp
		std::vector<double> pose(3,0.0), node(3,0.0);
		_cal_enc_pose_port.read(pose);

		for(int k=0;k<3;k++){
		  _primary_node_buffer[k+5][_node_buffer_fill] = pose[k]; //copy encoder stamp to the stamp vector
		  node[k] = _primary_node_buffer[k+2][_node_buffer_fill]; //write the node value to the vector v
		}
		_cal_lidar_local_node_port.write(node);

    // Calculate the calibrated lidar measurements in x and y
    node[0] = distance*cos(angle + pose[2]) + pose[0];
    node[1] = distance*sin(angle + pose[2]) + pose[1];
    _cal_lidar_global_node_port.write(node);

    // Increment node buffer fill count and switch buffers if necessary
		_node_buffer_fill++;
		if(_node_buffer_fill >= _lidar_data_length){
			//set to ports
			_cal_lidar_angle_port.write(*_primary_node_buffer);
			_cal_lidar_distance_port.write(*(_primary_node_buffer+1));
			_cal_lidar_x_port.write(*(_primary_node_buffer+2));
			_cal_lidar_y_port.write(*(_primary_node_buffer+3));
			_cal_lidar_quality_port.write(*(_primary_node_buffer+4));
			_cal_lidar_encoder_stamp_x_port.write(*(_primary_node_buffer+5));
			_cal_lidar_encoder_stamp_y_port.write(*(_primary_node_buffer+6));
			_cal_lidar_encoder_stamp_angle_port.write(*(_primary_node_buffer+7));

            correctInformation();

			//make other buffer current buffer
			std::vector<double> *ptemp = _primary_node_buffer;
			_primary_node_buffer = _secondary_node_buffer;
			_secondary_node_buffer = ptemp;

			//set fill to 0
			_node_buffer_fill = 0;

			RPLIDAR_DEBUG_PRINT("switched buffers.")
		}
	}
}

void RPLidar::correctInformation()
{

  double delta_x, delta_y, delta_theta, x_old, y_old, x_new, y_new;
  double rotAngle, movAngle;
  std::vector<double> x_cart(_lidar_data_length);
  std::vector<double> y_cart(_lidar_data_length);
  std::vector<double> theta_cart(_lidar_data_length);
  std::vector<double> angle(_lidar_data_length);
  std::vector<double> distance(_lidar_data_length);

  // De eerste positie wordt gelijkgezet met (0,0) en alle volgende punten worden relatief t.o.v. dit punt berekend
  x_cart[0] = 0;
  y_cart[0] = 0;
  theta_cart[0] = 0;

  // In de eerste for loop worden cartesiaanse coordinaten berekend in het het robotassenstelsel van op de eerste positie
  for(int i = 0; i < _lidar_data_length-1; i++){

    // De hoeveelheid van beweging in één richting volgens het assenstelsel van het punt i
    delta_x   = _primary_node_buffer[5][i+1] - _primary_node_buffer[5][i];
    delta_y   = _primary_node_buffer[6][i+1] - _primary_node_buffer[6][i];
    delta_theta = _primary_node_buffer[7][i+1] - _primary_node_buffer[7][i];
    movAngle = theta_cart[i] + delta_theta/2;

    //De hoek wordt beperkt tussen -M_PI en M_PI

    // De cartesiaanse coordinaten in het robotassenstelsel van de eerste positie, theta_cart is de hoek tussen het assenstelsel
    // op positie i en de allereerste positie. De veranderingen in bewegin moeten dus geprojecteerd (geroteerd) worden t.o.v.
    // het cartesiaans assenstelsel en opgeteld worden bij de vorige positie.
    x_cart[i+1]     = x_cart[i] + delta_x*cos(movAngle) - delta_y*sin(movAngle);
    y_cart[i+1]     = y_cart[i] + delta_y*cos(movAngle) + delta_x*sin(movAngle);
    theta_cart[i+1] = theta_cart[i] + delta_theta;

  }


  // In de tweede for loop worden de laserstralen eerst geroteerd
  for(int i = 0; i < _lidar_data_length; i++){

  // eerst wordt geroteerd zodat de coördinaten t.o.v. een assenstelsel zijn op het punt
  // van de tussenpositie van de robot, maar wel met dezelfde oriëntatie als de allereerste positie
    rotAngle = theta_cart[i];

    x_old = _primary_node_buffer[2][i];
    y_old = _primary_node_buffer[3][i];
    x_new = x_old*cos(rotAngle) - y_old*sin(rotAngle);
    y_new = x_old*sin(rotAngle) + y_old*cos(rotAngle);

    //Aangezien nu de huidige positie en de allerlaatste positie gekend zijn wordt dit assenstelsel (met dezelfde oriëntatie als
    // het cartesiaans assenstelsel), getransleerd naar het laatste punt in de beweging

    x_new = (x_cart[i] + x_new) - x_cart[_lidar_data_length-1];
    y_new = (y_cart[i] + y_new) - y_cart[_lidar_data_length-1];

    rotAngle = -theta_cart[_lidar_data_length-1];

    // Nu worden de laserstralen opnieuw geroteerd zodat hun coördinaten zoals deze opgemeten zouden worden kunnen berekend worden
    x_old = x_new;
    y_old = y_new;
    x_new = x_old*cos(rotAngle) - y_old*sin(rotAngle);
    y_new = x_old*sin(rotAngle) + y_old*cos(rotAngle);

    // Ten laatste worden de x, y van de laserstraal nog omgezet naar een angle en een distance
    angle[i]  = atan2(y_new,x_new);
    distance[i]  = sqrt(x_new*x_new + y_new*y_new);


  }

  // Hier worden ze op de poort geschreven
  _cor_lidar_distance_port.write(distance);
  _cor_lidar_angle_port.write(angle);

}

//ORO_CREATE_COMPONENT(RPLidar)
ORO_LIST_COMPONENT_TYPE(RPLidar)
