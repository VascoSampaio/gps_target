/*
 * Collect RTCM data and publish.
 *
 * Copyright 2018 Instituto Superior Tecnico
 *
 * Written by Bruno Gomes <bgomes@isr.tecnico.ulisboa.pt>
 */

#include <ros/ros.h>
#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <mavros_msgs/RTCM.h>
#include <sys/ioctl.h>
#include <sstream>
#include <libusbp-1/libusbp.hpp>

#include <bitset>
#include <string>
#include <iostream>

ros::Publisher pub_rtcm;
bool here_survey_completed = false;
bool here_gps;
int survey_min_duration_;
int survey_min_accuracy_;
double timenow;

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
	t.c_cc[VMIN] = 1;
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


/*
 * Poll for message class and id with a rate
 */
static void ubx_cfg_msg(int fd, int clas, int id, int rate)
{
	unsigned char buf[11];

	buf[0] = 0xb5; /*Header sync1*/
	buf[1] = 0x62; /*Header sync2*/
	buf[2] = 0x06; /*class ID: CFG*/
	buf[3] = 0x01; /*message ID: MSG*/
	buf[4] = 3;    /*lenght LSB*/
	buf[5] = 0;    /*lenght MSB*/
	buf[6] = clas;
	buf[7] = id;
	buf[8] = rate;
	ubx_checksum(buf + 2, 4 + 3, buf + 9);

	if(write(fd, buf, 11) != 11)
		ROS_ERROR("RTCM: write error UBX_CFG_MSG");
}

/*
 * Configure update rate
 */
static void ubx_cfg_rate(int fd, unsigned short ms, unsigned short cycle)
{
	/* CFG-RATE */
	struct ubx_payload_tx_cfg_rate{
		unsigned short	measRate;
		unsigned short	navRate;
		unsigned short	timeRef;
	} ubx_cfg_rate;
	
	unsigned char buf[14];

	buf[0] = 0xb5; /*Header sync1*/
	buf[1] = 0x62; /*Header sync2*/
	buf[2] = 0x06; /*class ID: CFG*/
	buf[3] = 0x08; /*message ID: MSG*/
	buf[4] = 6;    /*lenght LSB*/
	buf[5] = 0;    /*lenght MSB*/

	ubx_cfg_rate.measRate = ms;
 	ubx_cfg_rate.navRate  = cycle;
	ubx_cfg_rate.timeRef  = 0;
	
	memmove(buf + 6, &ubx_cfg_rate, 6);
	
	std::cout << "\n ";
	ubx_checksum(buf + 2, 4 + 6, buf + 12);	

	if(write(fd, buf, 14) != 14)
		ROS_ERROR("RTCM: write error UBX_CFG_RATE");
}

/*
 * Configure GPS USB interface to disable output of NMEA messages
 */
static void ubx_cfg_prt(int fd, bool UBX, bool NMEA, bool RTCM)
{
	/* CFG-PRT */
	struct ubx_payload_tx_cfg_prt{
		unsigned char	portID;
		unsigned char	reserved1;
		unsigned short	txReady;
		unsigned char	reserved2[8];
		unsigned short	inProtoMask;
		unsigned short	outProtoMask;
		unsigned char	reserved3[2];
		unsigned char	reserved4[2];
	} ubx_cfg_prt;

	ubx_cfg_prt.txReady      = 0x00;
	ubx_cfg_prt.portID       = 3; /*USB*/
	ubx_cfg_prt.inProtoMask  = 0x01; /*UBX*/
	ubx_cfg_prt.outProtoMask = 0x00;
	if (UBX) ubx_cfg_prt.outProtoMask  |= 0x01;  /*UBX*/
	if (NMEA) ubx_cfg_prt.outProtoMask |= 0x02; /*NMEA*/
	if (RTCM) ubx_cfg_prt.outProtoMask |= 0x20; /*RTCM3*/

	unsigned char buf[28];

	buf[0] = 0xb5; /*Header sync1*/
	buf[1] = 0x62; /*Header sync2*/
	buf[2] = 0x06; /*class ID: CFG*/
	buf[3] = 0x00; /*message ID: PRT*/
	buf[4] = 20;   /*lenght LSB*/
	buf[5] = 0;    /*lenght MSB*/

	memmove(buf + 6, &ubx_cfg_prt, 20);

	ubx_checksum(buf + 2, 4 + 20, buf + 26);

	if(write(fd, buf, 28) != 28)
		ROS_ERROR("RTCM: write error UBX_CFG_TMODE3");
}

static void ubx_cfg_prt_uart(int fd, int id, bool UBX, bool NMEA, bool RTCM)
{
	/* CFG-PRT */
	struct ubx_payload_tx_cfg_prt{
		unsigned char	portID;
		unsigned char	reserved1;
		unsigned short	txReady;
		unsigned char	mode[4]={0,0,0,0};
		unsigned char	badurate[4];
		unsigned short	inProtoMask;
		unsigned short	outProtoMask;
		unsigned char	flags[2];
		unsigned char	reserved2[2];
	} ubx_cfg_prt;
	
	ubx_cfg_prt.portID                  = id;   /*UART1*/
	// ubx_cfg_prt.txReady                 = 0x25D1;//0b0010010111010001;
	// ubx_cfg_prt.mode[0]                 = 0b11000000;
	// ubx_cfg_prt.mode[1]                 = 0b00000100;
	ubx_cfg_prt.inProtoMask             = 0x01;  /*UBX*/
	ubx_cfg_prt.outProtoMask            = 0x00;
	if (UBX) ubx_cfg_prt.outProtoMask  |= 0x01;  /*UBX*/
	if (NMEA) ubx_cfg_prt.outProtoMask |= 0x02;  /*NMEA*/
	if (RTCM) ubx_cfg_prt.outProtoMask |= 0x20;  /*RTCM3*/

	unsigned char buf[28];

	buf[0] = 0xb5; /*Header sync1*/
	buf[1] = 0x62; /*Header sync2*/
	buf[2] = 0x06; /*class ID: CFG*/
	buf[3] = 0x00; /*message ID: PRT*/
	buf[4] = 20;   /*lenght LSB*/
	buf[5] = 0;    /*lenght MSB*/

	memmove(buf + 6, &ubx_cfg_prt, 20);

	ubx_checksum(buf + 2, 4 + 20, buf + 26);

	if(write(fd, buf, 28) != 28)
		ROS_ERROR("RTCM: write error UBX_CFG_TMODE3");
}

/*
 * Set time mode 3 settings
 */
static void ubx_cfg_tmode(int fd, unsigned short flags, unsigned int durmin, unsigned int acclim)
{
	/* CFG-TMODE3 ublox 8 (protocol version >= 20) */
	static struct ubx_payload_tx_cfg_tmode3 {
		unsigned char version;
		unsigned char reserved1;
		unsigned short flags;
		int ecefXOrLat;
		int ecefYOrLon;
		int ecefZOrAlt;
		char ecefXOrLatHP;
		char ecefYOrLonHP;
		char ecefZOrAltHP;
		unsigned char reserved2;
		unsigned int fixedPosAcc;
		unsigned int svinMinDur;
		unsigned int svinAccLimit;
		unsigned char reserved3[8];
	}ubx_cfg_tmode3;

	ubx_cfg_tmode3.flags = flags;
	ubx_cfg_tmode3.svinMinDur = durmin;
	ubx_cfg_tmode3.svinAccLimit = acclim;

	unsigned char buf[48];

	buf[0] = 0xb5; /*Header sync1*/
	buf[1] = 0x62; /*Header sync2*/
	buf[2] = 0x06; /*class ID: CFG*/
	buf[3] = 0x71; /*message ID: TMODE3*/
	buf[4] = 40; /*lenght LSB*/
	buf[5] = 0; /*lenght MSB*/

	memmove(buf + 6, &ubx_cfg_tmode3, 40);

	ubx_checksum(buf + 2, 4 + 40, buf + 46);

	if(write(fd, buf, 48) != 48)
		ROS_ERROR("RTCM: write error UBX_CFG_TMODE3");
}


/*
 * Restart Survey-in mode
 */
static bool here_cfg_commands(int fd)
{
	ubx_cfg_prt(fd,1,0,1); /*cfg USB*/

	/*stop rtcm messages*/
	ubx_cfg_msg(fd, 0xf5, 0x05, 0); /*RTCM3 1005*/
	ubx_cfg_msg(fd, 0xf5, 0x4d, 0); /*RTCM3 1077*/
	ubx_cfg_msg(fd, 0xf5, 0x57, 0); /*RTCM3 1087*/

	/*stop survey in mode */
	ubx_cfg_tmode(fd, 0, 0, 0);
	ubx_cfg_msg(fd, 0x01, 0x3b, 0); /*NAV-SVIN*/

	sleep(0.5);

	/*enable survey in mode. select min duration (s) and min accuracy (0.1mm) */
	ubx_cfg_tmode(fd, 1, survey_min_duration_, survey_min_accuracy_);

	/*request status of survey-in*/
	ubx_cfg_msg(fd, 0x01, 0x3b, 1); /*NAV-SVIN*/
}


/*
 * Read Survey-in status
 */
static bool here_read_ubx_svin(int fd)
{
	int n;
	static unsigned int head;
	unsigned tail;
	static unsigned char buf[512];
	unsigned int packet_size;

	struct ubx_payload_rx_nav_svin{
		unsigned char version;
		unsigned char reserved1[3];
		unsigned int iTOW;
		unsigned int dur;
		int meanX;
		int meanY;
		int meanZ;
		char meanXHP;
		char meanYHP;
		char meanZHP;
		char reserved2;
		unsigned int meanAcc;
		unsigned int obs;
		unsigned char valid;
		unsigned char active;
		unsigned char reserved3[2];
	} ubx_nav_svin;

	n = read(fd, buf + head, sizeof buf -head);

	if(n < 0)
		return false;

	head += n;

	for(tail = 0; head - tail > 8; tail ++)	{

		if(buf[tail] == 0xb5 && buf[tail+1] == 0x62){

			packet_size = buf[tail+4] + (buf[tail+5] << 8) + 8;

			if(packet_size > head - tail)
				break;

			if(buf[tail+2] != 0x01 || buf[tail+3] != 0x3B){ /*if not svin message*/

				//ROS_WARN("RTCM: DBG RX message with class %x and ID %x", buf[tail+2], buf[tail+3]);

				tail += packet_size -1;
				continue;
			}

			memmove(&ubx_nav_svin, buf+tail+6, sizeof ubx_nav_svin);

			ROS_INFO("RTCM: time %d mean acc %d %d %d", ubx_nav_svin.dur, ubx_nav_svin.meanAcc/10, ubx_nav_svin.valid, ubx_nav_svin.active);

			if(ubx_nav_svin.valid && !ubx_nav_svin.active){
				here_survey_completed = true;

				ROS_INFO("RTCM: SURVEY completed. Streaming rtcm3");

				ubx_cfg_msg(fd, 0x01, 0x3b, 0); /*NAV-SVIN*/
				ubx_cfg_msg(fd, 0xf5, 0x05, 1); /*RTCM3 1005*/
				ubx_cfg_msg(fd, 0xf5, 0x4d, 1); /*RTCM3 1077*/
				ubx_cfg_msg(fd, 0xf5, 0x57, 1); /*RTCM3 1087*/
			}
		}
	}


	if(tail){
		head -= tail;
		if(head)
			memmove(buf, buf+tail, head);
	}

	return true;
}


void double2binary(double c){
union myUnion {
    double dValue;
    uint64_t iValue;
};

myUnion myValue;
myValue.dValue = c;
std::cout << myValue.iValue;
}

/*
 * Read GPS and publish MAVROS rtcm stream
 */
static bool readwrite(int fd, int a)
{
	mavros_msgs::RTCM rtcm;
	char buf[a];
	int i, n;

	n = read(fd, buf, 1024);

	if(n <= 0)
		return false;

	for(i=0; i<n; i++)
		std::cout << (int)buf[i] << " ";
	std::cout << "\n";

	rtcm.header.stamp = ros::Time::now();
	pub_rtcm.publish(rtcm);

	return true;
}

void main_with_exceptions(std::string &port_name, int vid, int pid){	
    libusbp::device device = libusbp::find_device_with_vid_pid(vid, pid);
    if (device){
		libusbp::serial_port port(device);
	    port_name = port.get_name();
		}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "rtkbasestation");
	ros::NodeHandle n;
	ros::NodeHandle pnh("~");
	static std::string portName_;
	struct pollfd pfd[1];
	int product_id, vendor_id, size;


	/*TODO: parse GPS brand name to create different configs according to GPS*/
	pnh.param<bool>("HERE_GPS", here_gps, true);

	pub_rtcm = n.advertise<mavros_msgs::RTCM>("/rtcm_stream",1);

	pnh.param<int>("vend", vendor_id, 0x0403);
    pnh.param<int>("prod", product_id,0x6001);
	pnh.param<int>("size", size,20);
	int c;
	char buf[size];
	pnh.param<int>("c", c,6);

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

		ROS_INFO("Successfull %s", portName_.c_str());


		while(ros::ok()) {
			std::cin >> product_id; 
			std::cout << "\\n\n\n";
			std::cout << "\nWrite  ";
			for (int i = 0; i < sizeof(buf) ; i++){
				buf[i] = (uint8_t)c*(i+1);
				std::cout << (int)buf[i] << " ";
			}
			
			std::cout << "\n";
   			if(write(pfd[0].fd, buf, sizeof(buf)) <=0){
				ROS_ERROR("RTCM: write error UBX_CFG_TMODE3");
				break;
			}			
			ros::spinOnce();
			if (!poll(pfd, 1, 3000)) {
				ROS_WARN("RTCM: read gps rtcm data timeout");
				continue;
			}
			std::cout << "\nRead   ";
			if (pfd[0].revents)
				if(!readwrite(pfd[0].fd, sizeof(buf)))
					break;
			sleep(1.0);
 	 	}

		close(pfd[0].fd);
	}

	return 0;
}
