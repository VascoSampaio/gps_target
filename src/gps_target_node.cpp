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

void write_for_checking(unsigned char *buf){
	message_checksum[4]  = buf[2];
	message_checksum[5]  = buf[3];
	ubx_checksum(message_checksum, 6, message_checksum+6);
}

void ubx_cfg(int fd, ubx_payload_valset* valset){
	
	std::cout << "Configurating "<< valset->item << "\n";
	uint8_t write_size = 10;
	unsigned char* buf;
	char type_id = valset->idValue.type().name()[0];

	switch(type_id){
		case 'c':
			write_size = 11;
			buf = (unsigned char*) malloc (write_size);
			memmove(buf + 10, &(boost::get<char>(valset->idValue)), sizeof(boost::get<char>(valset->idValue)));
			break;
		case 't':
			write_size = 12;
			buf = (unsigned char*) malloc (write_size);
			memmove(buf + 10, &(boost::get<uint16_t>(valset->idValue)), sizeof(boost::get<uint16_t>(valset->idValue)));
			break;
		case 'i':
			write_size = 14;
			buf = (unsigned char*) malloc (write_size);
			memmove(buf + 10, &(boost::get<int>(valset->idValue)), sizeof(boost::get<int>(valset->idValue)));
			break;
		default:
			buf = (unsigned char*) malloc (write_size);
			break;
	}

	buf[0] = 0xb5; /*Header sync1*/
	buf[1] = 0x62; /*Header sync2*/
	buf[2] = 0x06; /*class ID: CFG*/
	buf[3] = 0x8a;
	buf[4] = sizeof(valset);
	buf[5] = 0;    /*lenght MSB*/
	buf[6] = 0x00; //version
	buf[7] = 0x01; //layers
	buf[8] = 0x00; //reserved
	buf[6] = valset->keyValue & 0xFF; 
	buf[7] = (valset->keyValue >>  8) & 0xFF;
	buf[8] = (valset->keyValue >> 16) & 0xFF;
	buf[9] = (valset->keyValue >> 24) & 0xFF;

     
	ubx_checksum(buf + 2, write_size-4, buf + write_size - 2);	
	write_for_checking(buf);

	for (int i=0; i < write_size; i++)
		std::cout << std::hex << (int)buf[i] << " ";
	std::cout << " written\n";
 
	 if(write(fd, buf, write_size) != write_size)
		ROS_ERROR("GPS: write error UBX_CFG");

	free(buf);
}

static char getbyte(struct pollfd* pf, char rbuf[], char *&rp, uint8_t* bufcnt, uint8_t size)
{
    if ((rp - rbuf) >= *bufcnt) {/* buffer needs refill */        
        if (!poll(pf, 1, 3000)) {
			ROS_WARN("Read TIMEOUT");
			return false;
		}
		if (pfd->revents){
			*bufcnt = read(pf->fd, rbuf, size);
        	if (*bufcnt <= 0 || *bufcnt > size ) {
				return false;
        	}
        	rp = rbuf;
		}
    }
	return *rp++;
}

static bool parseUBX(struct pollfd* pf){

	uint8_t n = 0, BLEN = 100, MLEN = 10;
	char buf[BLEN], mesg[MLEN-2];
	char *rp = &buf[BLEN], *sync;
	int count=0;

	while (getbyte(pf, buf,rp, &n, BLEN) && (rp - buf) != n){
		if (*rp == -75){ //0XB5 for unsigned char
			sync = rp;
			if(*++rp == 98){ //0X62 for unsigned char
			rp++;
				for(; (rp-sync) <MLEN;){
					std::cout << rp-sync<< " " << rp-3-sync <<"\n";
					if((unsigned char)getbyte(pf, buf,rp, &n, BLEN) != message_checksum[rp-3-sync]){
						// std::cout << (int)message_checksum[rp-3-sync] << " expected\n";
						std::cout << "Expected " << (int)message_checksum[rp-3-sync];
						std::cout << " and got " << (int)*(rp-1) << " at position " << (rp-3-sync) << "\n";
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
	return false;
}

void parseNMEA(){
	char *rp = &gpsBuffer[5];
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

	rp++;
	for(;*++rp != ',';){
			// std::cout << *rp;
	}
	// std::cout << "\n";


	actual_coordinate_geo.latitude  = gps_gns.latitude;
	actual_coordinate_geo.longitude = gps_gns.longitude;
	actual_coordinate_geo.altitude  = 0;
	geometry_msgs::Point32 target_pose = multidrone::geographic_to_cartesian(actual_coordinate_geo, origin_geo_);
	pub_gps.publish(target_pose);
	ros::spinOnce();
	
}

static bool getNMEA(struct pollfd* pf){

		uint8_t n = 0, BLEN = 80, MLEN = 10;
		char buf[BLEN], msg_class, id, length, sync, mesg[MLEN-2];
		char *rp = &buf[BLEN];
		char crc  = 0;
		enum {START_WAIT, RECEIVING, MSG_RECEIVED} state = START_WAIT; 

		while(getbyte(pf, buf,rp, &n, BLEN)){
			if (*rp == 0xb5 ||*rp == 0x62) {
				std::cout << (int)*--rp <<" ";
				for(int i = 0; i < 9; i++)
					std::cout << std::hex << (int)*++rp <<" ";
				std::cout <<"\n";
			}
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
						std::cout << gpsBuffer[gpsPtr-1-gpsBuffer];
						ros::spinOnce();
					}
       		        break;
			  	case MSG_RECEIVED:
				 	std::cout << "   " << (int)(gpsPtr-gpsBuffer) << "\n";
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

void rtcm_streamer(const mavros_msgs::RTCM::ConstPtr& _msg, struct pollfd* pf, int& counter){ //
	char puf[1024];
 
	memmove(puf, &_msg->data, _msg->data.size());
	// counter++;

	// if(counter ==3){		
		if(write(pf[0].fd, puf, _msg->data.size()) != _msg->data.size())
			ROS_ERROR("RTCM: data not written");
		// counter =0;
		// for(int i=0; i <_msg->data.size();i++)
		// std::cout << (int)_msg->data[i]<< " ";
		// std::cout << "\n";
	// }
	
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
	bool configured = false;
	std::map<int, ubx_payload_valset*> valueId_map;

	ubx_payload_valset rate{0x30210001, "rate", (uint16_t) pnh.param("rate",100)};
	ubx_payload_valset RTCM{0x10770004,"RTCM", (char) 1};
	// ubx_payload_valset<char> DGNSSTO{0x201100c4,10,"DGNSSTO"};
	// ubx_payload_valset<char> S_BAS{0x10360002,0,"SBAS"};
	// ubx_payload_valset<char> GNS{0x209100b8,1,"GNS"};
	// ubx_payload_valset<char> GLL{0x209100cc,0,"GLL"}; 
	// ubx_payload_valset<char> GSA{0x209100c2,0,"GSA"};
	// ubx_payload_valset<char> GSV{0x209100c7,0,"GSV"};
	// ubx_payload_valset<char> RMC{0x209100ae,0,"RMC"};
	// ubx_payload_valset<char> VTG{0x209100b3,0,"VTG"};
	// ubx_payload_valset<char> GGA{0x209100bd,0,"GGA"};

//CREATE MAP
	valueId_map.insert(std::pair<int, ubx_payload_valset*>(0, &rate)); //RATE
	// valueId_map.insert(std::pair<int, ubx_payload_valset*>(0,&GSA));
	// valueId_map.insert(std::pair<int, ubx_payload_valset*>(1, &RTCM)); //USBINPROT-RTCM
	// valueId_map.insert(std::pair<int, ubx_payload_valset*>(2, &DGNSSTO)); //DGNSSTO
	// valueId_map.insert(std::pair<int, ubx_payload_valset*>(3, &S_BAS)); //SBAS
	// valueId_map.insert(std::pair<int, ubx_payload_valset*>(4, &GNS)); //SBAS
	// valueId_map.insert(std::pair<int, ubx_payload_valset*>(5, &GGA));
	// valueId_map.insert(std::pair<int, ubx_payload_valset*>(6, &VTG));
	// valueId_map.insert(std::pair<int, ubx_payload_valset*>(7, &RMC));
	// valueId_map.insert(std::pair<int, ubx_payload_valset*>(8, &GSV));
	// valueId_map.insert(std::pair<int, ubx_payload_valset*>(9, &GNS)); 
	// valueId_map.insert(std::pair<int, ubx_payload_valset*>(10,&GLL)); 
	// valueId_map.insert(std::pair<int, ubx_payload_valset*>(11,&GSA));

	std::signal(SIGINT, signalHandler);

	pub_gps  = n.advertise<geometry_msgs::Point32>("/gps_message",1);

	pnh.param<int>("vend", vendor_id, 0x0403);
    pnh.param<int>("prod", product_id,0x6001);

	// pnh.param<int>("number", configurator,0);
	pnh.param<bool>("config", configured, false);
	std::vector<double> origin_geo_vector;
	pnh.getParam("origin_geo",origin_geo_vector);

  	origin_geo_.latitude  = origin_geo_vector[0];
  	origin_geo_.longitude = origin_geo_vector[1];
  	origin_geo_.altitude  = origin_geo_vector[2];

	pfd[0].events = POLLIN;
	timenow = ros::Time::now().toSec();
  	while (ros::ok()) {
		main_with_exceptions(portName_, vendor_id, product_id);
		pfd[0].fd = rtcm_open("/dev/ttyACM0", 80);//portName_.c_str());

		if (pfd[0].fd < 0) {
			ROS_ERROR("RTCM: error opening %s", portName_.c_str());
			sleep(1);
			continue;
		}
		else
			ROS_INFO("Successfull %s", portName_.c_str());
	
		if (!configured){
			for(int i=0; i < valueId_map.size();){
					ubx_cfg(pfd[0].fd,valueId_map[i]);
					usleep(250000);
					if(parseUBX(pfd))
						i++;
			}
			ROS_INFO("CONFIGURED");
			configured = true;
		}else{
			sub_rtcm = n.subscribe<mavros_msgs::RTCM>("/rtcm_stream",3, std::bind(rtcm_streamer, std::placeholders::_1, pfd, sizer));
			while(ros::ok()) {
 				if(!getNMEA(pfd)){
					 break;
				 } 					
  	 		}
		}
		close(pfd[0].fd);
	}
	return 0;
}		