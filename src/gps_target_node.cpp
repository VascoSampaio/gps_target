/*
 * Collect NMEA data and publish.
 *
 * Copyright 2018 Instituto Superior Tecnico
 *
 * Written by Vasco Sampaio <vsampaio@isr.tecnico.ulisboa.pt>
 */

#include <gps_target/gps_target.h>

bool hex_decode(const unsigned char *in, size_t len, unsigned char *out){
        unsigned int i, t, hn, ln;
        for (t = 0,i = 0; i < len; i+=2,++t) {
                hn = in[i] > '9' ? in[i] - 'A' + 10 : in[i] - '0';
                ln = in[i+1] > '9' ? in[i+1] - 'A' + 10 : in[i+1] - '0';
                if(out[t] == (hn << 4 ) | ln);
					return true;
        }
	return false;        
}

/*
 * Open serial port device TTY @ 57600.
 * Return the serial port file descriptor, or -1 on failure.
 */

static int rtcm_open(const char *tty, int interrupt_min)
{
	int fd;
	struct termios t;
	unsigned i;

	fd = open(tty, O_RDWR | O_NOCTTY | O_NONBLOCK); //open file descriptor with no control over port and non blocking.
	if (fd < 0)
		return -1;	
	
	ioctl(fd, TIOCEXCL); //system call for exclusivity on the port. No other open to this file descriptor will be allowed

	memset(&t, 0, sizeof t);
	t.c_cc[VMIN] = interrupt_min;
	t.c_cflag = B115200 | CS8 | CREAD | CLOCAL | HUPCL; //baud rate, 8 bits, CREAD and CLOCAL are mandatory, let modem hang up

	if (tcsetattr(fd, TCSANOW, &t)) { //set parameters, t, to file descriptor, fd, immediately, TCSANOW
		close(fd);
		return -1;
	}
	return fd;
}

/*
 * Calculate UBX ckecksum for LEN bytes of data pointed by DATA
 */
static bool ubx_checksum(const unsigned char *data,  unsigned len, unsigned char ck[2], unsigned char comparator = 0)
{
	const unsigned char *buffer = data;
	unsigned char ck_a = 0;
	unsigned char ck_b = 0;

	while(len--){
		ck_a += *buffer++;
		ck_b += ck_a;				
	}

	ck[0] = ck_a;
	ck[1] = ck_b;

	if(comparator == ck_a || comparator == ck_b) {
		return true;
	}
	return false;
}

void write_for_checking(unsigned char *buf){
	message_checksum[4]  = buf[2];
	message_checksum[5]  = buf[3];
	ubx_checksum(message_checksum, 6, message_checksum+6);
}

void ubx_cfg(int fd, ubx_payload_valset* valset){
	
	uint8_t write_size = 10;
	unsigned char* buf;

	switch(valset->idValue.type().name()[0]){
		case 'h':
			write_size = 17;
			buf = (unsigned char*) malloc (write_size);
			memmove(buf + 14, &(boost::get<unsigned char>(valset->idValue)), sizeof(boost::get<unsigned char>(valset->idValue)));
			break;
		case 't':
			write_size = 18;
			buf = (unsigned char*) malloc (write_size);
			memmove(buf + 14, &(boost::get<uint16_t>(valset->idValue)), sizeof(boost::get<uint16_t>(valset->idValue)));
			break;
		case 'i':
			write_size = 20;
			buf = (unsigned char*) malloc (write_size);
			memmove(buf + 14, &(boost::get<int>(valset->idValue)), sizeof(boost::get<int>(valset->idValue)));
			break;
		default:
			buf = (unsigned char*) malloc (write_size);
			break;
	}

	buf[0]  = 0xb5; /*Header sync1*/
	buf[1]  = 0x62; /*Header sync2*/
	buf[2]  = 0x06; /*class ID: CFG*/
	buf[3]  = 0x8a;
	buf[4]  = write_size-8;
	buf[5]  = 0;    /*lenght MSB*/
	buf[6]  = 0x00; //version
	buf[7]  = 0x01; //layers
	buf[8]  = 0x00; //reserved
	buf[9]  = 0x00; //reserved
	buf[10] = valset->keyValue & 0xFF; 
	buf[11] = (valset->keyValue >>  8) & 0xFF;
	buf[12] = (valset->keyValue >> 16) & 0xFF;
	buf[13] = (valset->keyValue >> 24) & 0xFF;
     
	ubx_checksum(buf + 2, write_size-4, buf + write_size - 2);	
	write_for_checking(buf);

	// for (int i=0; i < write_size; i++)
		// std::cout << std::hex << (int)buf[i] << " ";
	// std::cout << " written\n";
 
	 if(write(fd, buf, write_size) != write_size)
		ROS_ERROR("GPS: write error UBX_CFG");

	free(buf);
}

static unsigned char getbyte(struct pollfd* pf, unsigned char rbuf[], unsigned char *&rp, uint8_t* bufcnt, uint8_t size)
{
    if ((rp - rbuf) >= *bufcnt) {/* buffer needs refill */        
		if (!poll(pf, 1, 2000)) {
			ROS_WARN("Read TIMEOUT");
			return false;
		}
		if (pfd->revents){
			*bufcnt = read(pf->fd, rbuf, size);
        	if (*bufcnt <= 0 || *bufcnt > size ) {
				return false;
        	}
        	rp = rbuf;
			// std::cout <<"\nBUFFER \n";
			// for (int i=0; i < *bufcnt; i++)
				// std::cout << std::hex << (int)rbuf[i] << " "; //std::cout << rbuf[i];
			// std::cout <<"    " << (int)*bufcnt << " buffer\n\n";
		}
    }
	return *rp++;
}

static bool parseUBX(struct pollfd* pf){

	uint8_t n = 0, BLEN = 100, MLEN = 8;
	unsigned char buf[BLEN], mesg[MLEN-2];
	unsigned char *rp = &buf[BLEN], *sync;
	int count=0;

	while (getbyte(pf, buf,rp, &n, BLEN) && (rp - buf) != n){
		if (*(rp-1) == 0Xb5){ //0XB5 for unsigned char
			sync = rp;
			if(*rp == 0X62){ //0X62 for unsigned char
			rp++;
				for(; (rp-sync) <MLEN;){
					// std::cout <<rp-2-sync << " " << (int)getbyte(pf, buf,rp, &n, BLEN) << " " << (int)message_checksum[rp-1-sync] << "\n";
					if(getbyte(pf, buf,rp, &n, BLEN) != message_checksum[rp-2-sync]){
						// std::cout << (int)message_checksum[rp-3-sync] << " expected\n";
						std::cout << "Expected " << (int)message_checksum[rp-2-sync];
						std::cout << " and got " << (int)*(rp-1) << " at position " << (rp-2-sync) << "\n";
						// std::cout <<  (int*)rp << "\n";  
						// rp--;
						return false;
					}
				}
				std::cout <<"\n";
				return true;							
			}
			else
				continue;
		}            		
	}
	// std::cout << "(int)\n";
	return false;
}

void parseNMEA(){
	unsigned char *rp = &nmeaBuffer[5];
	double divisor = 10;
	gps_gns.latitude = gps_gns.longitude = 0; 

	while(*++rp != ','){}
		// std::cout << *rp;

	// std::cout << "\n";
	for(;*++rp != ',';){
		if(*rp != '.' ){
			gps_gns.latitude += (*rp -'0') *divisor;
			// std::cout << *rp -'0';
			divisor /= 10;
		}
	}
	gps_gns.latitude = (int(gps_gns.latitude)+(gps_gns.latitude-int(gps_gns.latitude))*100/60); 
	// std::cout << "\n";
	rp +=2; divisor=100;
	for(;*++rp != ',';){
		if(*rp != '.' ){
			gps_gns.longitude += (*rp -'0') *divisor;
			// std::cout << *rp -'0';
			divisor /= 10;
		}
	}
	// std::cout << "\n";
	if (*++rp == 'W')
		gps_gns.longitude = -(int(gps_gns.longitude)+(gps_gns.longitude-int(gps_gns.longitude))*100/60);
	else 
		gps_gns.longitude = (int(gps_gns.longitude)+(gps_gns.longitude-int(gps_gns.longitude))*100/60);
// 
	rp++;
	for(;*++rp != ',';){
			// std::cout << *rp;
	}
	// std::cout << "\n";
// 

	actual_coordinate_geo.latitude  = gps_gns.latitude;
	actual_coordinate_geo.longitude = gps_gns.longitude;
	actual_coordinate_geo.altitude  = 0;
	geometry_msgs::Point32 target_pose = multidrone::geographic_to_cartesian(actual_coordinate_geo, origin_geo_);
	pub_gps.publish(target_pose);
	// ros::spinOnce();
	
}

static bool getNMEA(struct pollfd* pf){

		uint8_t n = 0, BLEN = 80, MLEN = 10;
		unsigned char buf[BLEN];
		unsigned char *rp = &buf[BLEN];
		unsigned char crc  = 0;
		enum {START_WAIT, RECEIVING_NMEA, RECEIVING_UBX, RECEIVING_UBX_PAYLOAD, MSG_RECEIVED_NMEA,MSG_RECEIVED_UBX} 
		state = START_WAIT; 
		char msg_id = -1, msg_clas= -1;
		int msg_lgt= -1;

		while(getbyte(pf, buf,rp, &n, BLEN) !=n){
			switch(state){
       		   	case START_WAIT:               		 // Waiting for start of message
					if(*(rp-1) == '$'){
						nmeaPtr = nmeaBuffer;		// Start character received...
						state = RECEIVING_NMEA;
					}
					else if(*(rp-1) == 0xb5){
						if(getbyte(pf, buf,rp, &n, BLEN) !=n){
							if(*(rp-1) == 0x62){
								ubxPtr = ubxBuffer;		// Start character received...
								msg_lgt = msg_id = msg_clas = -1;
								state = RECEIVING_UBX;								
							}
						}
					}                        	 //                        	 // and start receiving data
       		      	break;
       		   	case RECEIVING_NMEA:                       // Message Start received
       		      	if(*(rp-1) == '*'){              // If end of message...
						if (hex_decode(rp,2, &crc)){
							state = MSG_RECEIVED_NMEA;    // indicate we're done - don't process any more character until														  
							break;
						}
						state  = START_WAIT;
						nmeaPtr = nmeaBuffer;
						return false;
					}                             		      	 
       		      	else{
						*(nmeaPtr++) = *(rp-1); 
						crc ^= *(rp-1);
					}
       		        break;
					   
				case RECEIVING_UBX:                  // Message Start received
       		      	if(msg_clas == -1)
						*(ubxPtr++) = msg_clas = *(rp-1);
					else if(msg_id == -1)             // If end of message...
						*(ubxPtr++) = msg_id = *(rp-1);
					else if(msg_lgt == -1)              // If end of message...
						*(ubxPtr++) = msg_lgt = *(rp-1);
					else{
						*(ubxPtr++) = (*(rp-1) <<  8);
						msg_lgt |= (*(rp-1) <<  8);
						state = RECEIVING_UBX_PAYLOAD;
						// std::cout <<std::hex<< " " << (int)ubxBuffer[0]<< " " << (int)ubxBuffer[1]<< " " << (int)ubxBuffer[2] << " " <<(int)ubxBuffer[3];
					}
					break;
						
				case RECEIVING_UBX_PAYLOAD:	
					if(ubxPtr - ubxBuffer -4< msg_lgt){
						*(ubxPtr++) = *(rp-1);	
					}
					else{
						if (ubx_checksum(ubxBuffer,msg_lgt+4,ubxBuffer+4+msg_lgt,*(rp-1))){
 							if(getbyte(pf, buf,rp, &n, BLEN) !=n){
								if (*(ubxBuffer+5+msg_lgt) == *(rp-1)){
 									state = MSG_RECEIVED_UBX;
								}
								else{
									state = START_WAIT;
								}		
							}
							else{
								state = START_WAIT;
							}	
						}
						else{
							state = START_WAIT;
						}
					}		
					break;

			  	case MSG_RECEIVED_NMEA:
					std::cout <<"nmea: ";
					for(unsigned char* last = nmeaPtr;((nmeaPtr)-nmeaBuffer) > 0 ;)
				 	 	std::cout << nmeaBuffer[last-nmeaPtr--];
				 	std::cout << "   " << (int)(nmeaPtr-nmeaBuffer) << "\n";
				  	parseNMEA();
					state  = START_WAIT;
					break;

				case MSG_RECEIVED_UBX:
					ubxPtr = ubxBuffer;
					std::cout <<"UBX: ";
					for(;(ubxPtr-ubxBuffer) < msg_lgt+6;)
						std::cout <<std::hex<< " " << (int)*(ubxPtr++);
				 	std::cout << "  DONE  "  << (int)(ubxPtr - ubxBuffer) << "\n";
					// parseUBX();
					state  = START_WAIT;
					break;					
			}
       	}  	
	return false;
}
 
void main_with_exceptions(std::string &port_name, int vid, int pid){	
    libusbp::device device = libusbp::find_device_with_vid_pid(vid, pid);
    if (device){
		libusbp::serial_port port(device);
	    port_name = port.get_name();
		}
}

void rtcm_streamer(const mavros_msgs::RTCM::ConstPtr& _msg, struct pollfd* pf, int& counter){ //
  int a =0;
  unsigned char puf[_msg->data.size()];
//   _msg->data[ _msg->data.size()+1] = 13;
//   _msg->data[ _msg->data.size()+1] = 10;
	
	// memmove(puf,_msg->data,_msg->data.size());

	// ROS_INFO("WRITTEN");
	for(; a <_msg->data.size();a++){
		puf[a] = _msg->data[a];
		// std::cout <<std::hex << (int)puf[a]<< " ";
	}
	// std::cout <<"\n\n";
	// puf[a+1] =13; 
	// puf[a+2] =10;
	// std::cout <<std::hex << (int)puf[a+1]<< " "<< (int)puf[a+2];
	// std::cout << "\n"  <<"\n\n";
		
	if(_msg->data[0] ==0xb5){
		return;
	}

	if(write(pf[0].fd, puf, sizeof(puf)) != sizeof(puf))
		ROS_ERROR("RTCM: data not written");			
}

void signalHandler( int signum ) {
   close(pfd[0].fd);
   std::exit(signum);  
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_target");
	ros::NodeHandle n;
	ros::NodeHandle pnh("~");
	static std::string portName_;
	int product_id, vendor_id, sizer = 0;
	bool configured = false, port_opened = false;

	std::signal(SIGINT, signalHandler);

	pub_gps  = n.advertise<geometry_msgs::Point32>("/gps_message",1);

	pnh.param<int>("vend", vendor_id, 0x0403);
    pnh.param<int>("prod", product_id,0x6001);
	pnh.param<bool>("config", configured, false);

	std::vector<double> origin_geo_vector;
	pnh.getParam("origin_geo",origin_geo_vector);
  	origin_geo_.latitude  = origin_geo_vector[0];
  	origin_geo_.longitude = origin_geo_vector[1];
  	origin_geo_.altitude  = origin_geo_vector[2];

	pfd[0].events = POLLIN;


	ubx_payload_valset rate{0x30210001,    (uint16_t)pnh.param("rate",100), "rate"};
	ubx_payload_valset DGNSSTO{0x201100c4, (unsigned char) 20,"DGNSSTO"};
	ubx_payload_valset S_BAS{0x10930012,   (unsigned char) 0,"SBAS"};
	ubx_payload_valset GPS_ONLY{0x10930025,(unsigned char) 0,"GPS_ONLY"};	

	//FOR USB port
	
	ubx_payload_valset NMEA_OUT{0x10780002, (uint16_t)pnh.param("NMEA",1),"NMEA_OUT"};
	ubx_payload_valset UBX_OUT{0x10780001, (unsigned char)1,"UBX_OUT"};
	ubx_payload_valset NMEA_IN{0x10770002,(unsigned char) 0,"NMEA_IN"}; 
	ubx_payload_valset COMM_OUT{0x20910352,(unsigned char)1, "COMM_OUT"};
	ubx_payload_valset GNS{0x209100b8,(unsigned char)1,"GNS"};
	ubx_payload_valset GLL{0x209100cc,(unsigned char)0,"GLL"}; 
	ubx_payload_valset GSA{0x209100c2,(unsigned char)0,"GSA"};
	ubx_payload_valset GSV{0x209100c7,(unsigned char)0,"GSV"};
	ubx_payload_valset RMC{0x209100ae,(unsigned char)0,"RMC"};
	ubx_payload_valset VTG{0x209100b3,(unsigned char)0,"VTG"};
	ubx_payload_valset GGA{0x209100bd,(unsigned char)0,"GGA"};
	ubx_payload_valset RTCM{0x10770004,(unsigned char) 1,"RTCM"};
	ubx_payload_valset baudrate;
	
	//FOR UART port
	if (!pnh.param<bool>("USB", false)){ //UART
		ROS_WARN("UART");
		NMEA_OUT= {0x10740002, (unsigned char)pnh.param("NMEA",1),"NMEA_OUT"};
		UBX_OUT = {0x10740001, (unsigned char)1,"UBX_OUT"};
		NMEA_IN = {0x10730002, (unsigned char)0,"NMEA_IN"};
		GNS     = {0x209100b6, (unsigned char)1,"GNS"};
		GLL     = {0x209100ca, (unsigned char)0,"GLL"};
		GSA     = {0x209100c0, (unsigned char)0,"GSA"};
		GSV     = {0x209100c5, (unsigned char)0,"GSV"};
		RMC     = {0x209100ac, (unsigned char)0,"RMC"};
		VTG     = {0x209100b1, (unsigned char)0,"VTG"};
		GGA     = {0x209100bb, (unsigned char)0,"GGA"};
		COMM_OUT= {0x20910350, (unsigned char)0, "COMM_OUT"};
 		baudrate= {0x40520001, (int) 115200, "baudrate"};
	}
	// ubx_payload_valset RTCM_IN{0x10770004,1,"RTCM_IN"};
	// ubx_payload_valset RXM{0x2091026b,0,"RXM"};
	// ubx_payload_valset NAV{0x20910348,0,"NAV"};

	valset_map.insert(std::make_pair(0, &GGA));
	valset_map.insert(std::make_pair(1, &GNS));
	valset_map.insert(std::make_pair(2, &VTG));
	valset_map.insert(std::make_pair(3, &RMC));
	valset_map.insert(std::make_pair(4, &GSV));
	valset_map.insert(std::make_pair(5, &GLL));
	valset_map.insert(std::make_pair(6, &GSA));
	valset_map.insert(std::make_pair(7, &S_BAS));
	valset_map.insert(std::make_pair(8, &DGNSSTO)); 
	valset_map.insert(std::make_pair(9, &COMM_OUT));
	valset_map.insert(std::make_pair(10,&GPS_ONLY));
	valset_map.insert(std::make_pair(11,&rate));  
	valset_map.insert(std::make_pair(12,&NMEA_OUT));	
	// valset_map.insert(std::make_pair(13,&baudrate));  
	
	// valset_map.insert(std::make_pair(0, &UBX_OUT));
	// valset_map.insert(std::make_pair(1, &NMEA_IN));
	// valset_map.insert(std::make_pair(11,&RXM)); //USBINPROT-RTCM
	// valset_map.insert(std::make_pair(14,&NAV));

	timenow = ros::Time::now().toSec();
	ros::AsyncSpinner spinner(1);
	spinner.start();

	  while (ros::ok()) {
		if(!port_opened){
			main_with_exceptions(portName_, vendor_id, product_id);
			// portName_= "/dev/ttyACM0";
			pfd[0].fd = rtcm_open(portName_.c_str(),buf_size);
			if (pfd[0].fd < 0) {
				ROS_ERROR("RTCM: error opening %s", portName_.c_str());
				sleep(1);
				continue;
			}
			else {
				ROS_INFO("PORT OPENED %s with VMIN %d", portName_.c_str(), buf_size);
				port_opened = true;
			}
		} 
		// else if (NMEA_IN.idValue != 0){
		// 	ROS_INFO("Changing VMIN OPENED %s", portName_.c_str());
		// 	pfd[0].fd = rtcm_open(portName_.c_str(),80);
		// }
		else {
			ROS_INFO("Kepping VMIN OPENED %s", portName_.c_str());
			pfd[0].fd = rtcm_open(portName_.c_str(),buf_size);
		}

		if (!configured){
			int i;
			for(i=0; i < valset_map.size();){
				ubx_cfg(pfd[0].fd,valset_map[i]);
				usleep(250000);
				if(parseUBX(pfd)){
					std::cout << "Configured "<< valset_map[i]->item << "\n";
					i++;
				}
			}
			ROS_INFO("CONFIGURED");
			configured = true;
			char start_stream= 'a';
			if(write(pfd[0].fd, &start_stream , 1) != 1)
				ROS_ERROR("PIC32: FAILED TO START NMEA STREAM");
		 }
		else{
			sub_rtcm = n.subscribe<mavros_msgs::RTCM>("/rtcm_stream",3, std::bind(rtcm_streamer, std::placeholders::_1, pfd, sizer));
			while(ros::ok()) {
 				if(!getNMEA(pfd)){
					// break;
				 } 					
  	 		}
		}
		close(pfd[0].fd);
	}
	return 0;
}		