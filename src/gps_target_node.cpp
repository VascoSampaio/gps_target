/*
 * Collect NMEA data and publish.
 *
 * Copyright 2018 Instituto Superior Tecnico
 *
 * Written by Vasco Sampaio <vsampaio@isr.tecnico.ulisboa.pt>
 */

#include <gps_target/gps_target.h>

bool hex_decode(const char *in, size_t len,char *out){
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
static int rtcm_open(const char *tty)
{
	int fd;
	struct termios t;
	unsigned i;

	fd = open(tty, O_RDWR | O_NOCTTY | O_NONBLOCK); //open file descriptor with no control over port and non blocking.
	if (fd < 0)
		return -1;	
	
	ioctl(fd, TIOCEXCL); //system call for exclusivity on the port. No other open to this file descriptor will be allowed

	memset(&t, 0, sizeof t);
	t.c_cc[VMIN] = 80;
	t.c_cflag = B38400 | CS8 | CREAD | CLOCAL | HUPCL; //baud rate, 8 bits, CREAD and CLOCAL are mandatory, let modem hang up

	if (tcsetattr(fd, TCSANOW, &t)) { //set parameters, t, to file descriptor, fd, immediately, TCSANOW
		close(fd);
		return -1;
	}
	return fd;
}

/*
 * Calculate UBX ckecksum for LEN bytes of data pointed by DATA
 */
static void ubx_checksum(const unsigned char *data, unsigned len, unsigned char ck[2])
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
}

void print_write(unsigned char *buf, unsigned char length){
	std::cout << "Write is " << (int)length << " and data is \n";
	for (unsigned char *end = buf + length; buf < end; buf++)
		std::cout << int(*buf) << " ";
	std::cout << "\n";
}

static void ubx_cfg(int fd, int clas, int id){
	uint8_t write_size;
	unsigned char buf[28];
	buf[0] = 0xb5; /*Header sync1*/
	buf[1] = 0x62; /*Header sync2*/
	buf[2] = 0x06; /*class ID: CFG*/
	buf[5] = 0;    /*lenght MSB*/
	
	switch (clas){
		case  PRT :
			std::cout << "Configurating Port\n";
			ubx_cfg_prt.portID         = 3; /*USB*/
			ubx_cfg_prt.txReady        = 0x00;
			ubx_cfg_prt.inProtoMask    = 0x01; /*UBX*/
			ubx_cfg_prt.outProtoMask   = 0x03; 
			buf[3]                     = 0x00;
			buf[4]                     = 20;
			write_size                 = 28;
			memmove(buf + 6, &ubx_cfg_prt, write_size-2);
			break;

		 case UART : 
			std::cout << "Configurating UART\n";
		 	ubx_cfg_uart.portID        = 1;     //UART1
		 	ubx_cfg_uart.reserved1	   = 0;		
		 	ubx_cfg_uart.txReady       = 0x00;  //Disabled;
		 	ubx_cfg_uart.mode[0]       = 0xC0;  //No Parity, 8 bit, 1 stop bit;
		 	ubx_cfg_uart.mode[1]       = 0x08;     
		 	ubx_cfg_uart.mode[2]       = 0x00;     
		 	ubx_cfg_uart.mode[3]       = 0x00;     	
		 	ubx_cfg_uart.baudrate      = 38400;
		 	ubx_cfg_uart.inProtoMask   = 0x01;  /*UBX*/
		 	ubx_cfg_uart.outProtoMask  = 0x03;  /*UBX and NMEA*/
			buf[3]                     = 0x00;
			buf[4]                     = 20;
			write_size                 = 28;
			memmove(buf + 6, &ubx_cfg_uart, write_size-2);
			break;

		 case  MSG :
			ubx_cfg_msg.clas 		= 0XF0;
		 	ubx_cfg_msg.rate 		= 0;
			if(id == GNS){
				ubx_cfg_msg.rate 	= 1;
				ubx_cfg_msg.id   	= 0x0D;
				std::cout << "Configurating MSG\n";
			}
		 	if(id == GGA) ubx_cfg_msg.id   	= 0x00;
			if(id == GLL) ubx_cfg_msg.id   	= 0x01;
			if(id == GSA) ubx_cfg_msg.id   	= 0x02;
			if(id == GSV) ubx_cfg_msg.id   	= 0x03;
			if(id == RMC) ubx_cfg_msg.id   	= 0x04;
			if(id == VTG) ubx_cfg_msg.id  	= 0x05;

			buf[3]                  = 0x01;
			buf[4]                  = 3;
			write_size              = 11;
			memmove(buf + 6, &ubx_cfg_msg, write_size-2);
			break;

		 case  RATE :
			std::cout << "Configurating RATE\n";
		 	ubx_cfg_rate.measRate      = id; //Elapsed Time between messages
 			ubx_cfg_rate.navRate       = 1;  //Number of elapsed measurments that trigger a navigation epoch
			ubx_cfg_rate.timeRef       = 0;  //Time system to which measurements are aligned; 0 - UTC; 1 - GPS; 2- GLONASS
			buf[3]                     = 0x08;
			buf[4]                     = 6;
			write_size                 = 14;
			memmove(buf + 6, &ubx_cfg_rate, write_size-2);
			break; 
	}

	 ubx_checksum(buf + 2, write_size-4, buf + write_size - 2);	
	//  print_write(buf,write_size);
	 
	 if(write(fd, buf, write_size) != write_size)
		ROS_ERROR("GPS: write error UBX_CFG");
}

// void start_gps_stream(int fd){
	// char buf[1] = {'!'};
	// if(write(fd, buf, 1) != 1)
		// ROS_ERROR("FAILED TO START GPS STREAM");
// }
 
static char getbyte(struct pollfd* pfd, char rbuf[], char *&rp, uint8_t* bufcnt, uint8_t size)
{
	// std::cout << "IN FUNCTION " <<"\n\n";
	// std::cout << "rp is "   <<  (int)*rp   <<"\n";
	// std::cout << "rbuf is "  <<  &rbuf <<"\n";	
	// std::cout << "rdiff is " <<  rp-rbuf <<"\n";		

    if ((rp - rbuf) >= *bufcnt) {/* buffer needs refill */        
        if (!poll(pfd, 1, 3000)) {
			ROS_WARN("Read TIMEOUT");
			return false;
		}
		if (pfd->revents){
			*bufcnt = read(pfd->fd, rbuf, size);
        	if (*bufcnt <= 0 || *bufcnt > size ) {
				return false;
        	}
			// std::cout << "\nshit is " << (int)*bufcnt <<"\n";
        	rp = rbuf;
		}
    }
	// std::cout << "rp is "   <<   (int)*rp   <<"\n";
	// std::cout << "rbuf is "  <<  &rbuf <<"\n";	
	// std::cout << "rdiff is " <<  rp-rbuf <<"\n";
	// std::cout << "OUT OF FUNCTION " << "\n\n";

	return *rp++;
}

static bool parseUBX(struct pollfd* pfd){

	uint8_t n = 0, BLEN = 100, MLEN = 10;
	char buf[BLEN], mesg[MLEN-2];
	char *rp = &buf[BLEN], *sync;
	int count=0;

	// std::cout << "\nrp is "   <<  (int)*rp   <<"\n";
	// std::cout << "buf is "  <<  &buf <<"\n";	
	// std::cout << "diff is " << rp - buf <<"\n\n";

	//std::cout << "Trying to resync" << std::hex << *rp << "\n";

	while (getbyte(pfd, buf,rp, &n, BLEN) && count++ < 80){
		if (*rp == -75){ //0XB5 for unsigned char
			sync = rp;
			if(*++rp == 98){ //0X62 for unsigned char
				*rp++;
				for(; (rp-sync) <MLEN;)
					getbyte(pfd, buf,rp, &n, BLEN);							

				// std::cout << (int)mesg[0] << " is the msg_class\n";
				// std::cout << (int)mesg[1] << " is the id\n";
				// std::cout << (int)(mesg[2] |= mesg[3] <<8) << " is the length\n";
				// std::cout << (int)mesg[4] << " is the Class ID of the Acknowledged Message \n";
				// std::cout << (int)mesg[5] << " is the Message ID of the Acknowledged Message \n";
				// std::cout << (int)mesg[6] << " is the Checksum A \n";
				// std::cout << (int)mesg[7] << " is the Checksum B \n\n";
				return true;
			}
			else
				continue;
		}            		
	}
	return false;
}

void parseNMEA(){
	char *rp = &gpsBuffer[5];
	double divisor = 10; 

	while(*++rp != ',')
		std::cout << *rp;

	std::cout << "   ";
	for(;*++rp != ',';){
		if(*rp != '.' ){
			gps_gns.latitude += (*rp -'0') *divisor;
			std::cout << *rp -'0';
			divisor /= 10;
		}
	}
	std::cout << "\n";

	// std::cout << std::fixed;
    // std::cout << std::setprecision(7);
	// std::cout << (int(latitude)+(latitude-int(latitude))*100/60) << "\n";
	// do{std::cout << *rp; crc ^= *rp; if(rp - buf >= BLEN) return 0; }while(getbyte(fd, buf,rp, &n, BLEN) != ',');
	// crc ^= *rp;
	// //Longitude
	// divisor = 100;
	// while(getbyte(fd, buf,rp, &n, BLEN) != ','){
	// 	if(*rp != '.'){
	// 		longitude += (*rp -'0') *divisor;
	// 		divisor /= 10;
	// 	}
	// 	std::cout << *rp; crc ^= *rp;
	// }
	// std::cout << (int(longitude)+(longitude-int(longitude))*100/60) << "\n";
	
	// do{std::cout << *rp; crc ^= *rp; if(rp - buf >= BLEN) return 0; }while(getbyte(fd, buf,rp, &n, BLEN) != '*');
	// do{std::cout << *rp; if(rp - buf >= BLEN) return 0; }while(getbyte(fd, buf,rp, &n, BLEN) != '\n');
 	// std::cout << "crc " << std::hex << crc << "\n";
	// position_available = true;
}

static bool getNMEA(struct pollfd* pfd){

		uint8_t n = 0, BLEN = 80, MLEN = 10;
		char buf[BLEN], msg_class, id, length, sync, mesg[MLEN-2];
		char *rp = &buf[BLEN];
		char crc  = 0;
		enum {START_WAIT, RECEIVING, MSG_RECEIVED} state = START_WAIT; 

		while(getbyte(pfd, buf,rp, &n, BLEN)){
       		switch(state){
       		   	case START_WAIT:               		 // Waiting for start of message
					if(*(rp-1) == '$'){
						gpsPtr = gpsBuffer;		// Start character received...
						state = RECEIVING;
					}                        	 // and start receiving data
       		      	break;
       		   	case RECEIVING:                       // Message Start received
       		      	if(*(rp-1) == '*'){              // If end of message...
						if (hex_decode(rp,2, &crc)){
							state = MSG_RECEIVED;    // indicate we're done - don't process any more character until														  
							break;
						}
						state  = START_WAIT;
						gpsPtr = gpsBuffer;
						return false;
					}                             		      	 
       		      	else{
						*(gpsPtr++) = *(rp-1); 
						crc ^= *(rp-1);
						// std::cout << gpsBuffer[gpsPtr-1-gpsBuffer];
					}
       		        break;
			  	case MSG_RECEIVED:
				 	//  std::cout << "   " << (int)(gpsPtr-gpsBuffer) << "\n";
				  	parseNMEA();
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

void timerCallback(const ros::TimerEvent&){ 
	if(position_available){
		actual_coordinate_geo.latitude = latitude;
		actual_coordinate_geo.longitude = longitude;
		actual_coordinate_geo.altitude = 0;
		geometry_msgs::Point32 target_pose = multidrone::geographic_to_cartesian(actual_coordinate_geo, origin_geo_);
		pub_gps.publish(target_pose);
	}
	position_available = false;
}

void signalHandler( int signum ) {
	close(pfd[0].fd);
   std::exit(signum);  
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rtkbasestation");
	ros::NodeHandle n;
	ros::NodeHandle pnh("~");
	static std::string portName_;
	int product_id, vendor_id, size, configurator = 0, confi;
	int freq; 
	bool isValid, configured;

	std::signal(SIGINT, signalHandler);

	pub_gps = n.advertise<geometry_msgs::Point32>("/gps_message",1);
	ros::Timer timer_  = n.createTimer(ros::Duration(1/30), timerCallback); //33Hz

	pnh.param<int>("vend", vendor_id, 0x0403);
    pnh.param<int>("prod", product_id,0x6001);
	pnh.param<int>("rate", freq,100);
	pnh.param<int>("number", configurator,0);
	std::vector<double> origin_geo_vector;
	pnh.getParam("origin_geo",origin_geo_vector);

  	origin_geo_.latitude  = origin_geo_vector[0];
  	origin_geo_.longitude = origin_geo_vector[1];
  	origin_geo_.altitude  = origin_geo_vector[2];

	pfd[0].events = POLLIN;
	timenow = ros::Time::now().toSec();

  	while (ros::ok()) {
		ros::spinOnce();
		main_with_exceptions(portName_, vendor_id, product_id);
		pfd[0].fd = rtcm_open(portName_.c_str());

		if (pfd[0].fd < 0) {
			ROS_ERROR("RTCM: error opening %s", portName_.c_str());
			sleep(1);
			continue;
		}
		else
		ROS_INFO("Successfull %s", portName_.c_str());
	
		while (isValid && ros::ok()) {
 			switch(configurator){
			 	case  PRT :
			 		ubx_cfg(pfd[0].fd, PRT, 1);       //configure USB port
 					//sleep(1.0);
					 if(parseUBX(pfd)){
						configurator++;
						continue;
					}
					break;
				 case  UART :
					ubx_cfg(pfd[0].fd,UART,1); 		//configure UART port
					 if(parseUBX(pfd)){
						configurator++;
						continue;
					}
					break;

				case  MSG :
					for (int i = 0; i < GGA+1;){
						ubx_cfg(pfd[0].fd, MSG, i); 
						if(parseUBX(pfd)){ 
							if (i == GGA){
								configurator++;
								break;
							}
							i++;
						}
					}
					break;
					
					case  RATE :
					ubx_cfg(pfd[0].fd,RATE,freq); //configure UART port
					if(parseUBX(pfd)){
						configurator++;
						continue;
					}
					break;		

				 case CFG :
				 	std::cout << "CONFIGURED\n";
					configured   = true;
					configurator = PRT;
					isValid      = false;
					break;

				 default : 	
					std::cout << "DEFAULTED\n";	
					isValid     = false;
					configured  = false;
					break;
			}
		}
		if (configured){
			while(ros::ok()) {
 				if(!getNMEA(pfd)){
					 isValid = true;
					 break;
				 } 					
  	 		}
		}
		close(pfd[0].fd);
	}

	return 0;
}