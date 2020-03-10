#ifndef GPS_TARGET_H
#define GPS_TARGET_H

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
#include <unistd.h>
#include <csignal>

#include <multidrone_kml_parser/geographic_to_cartesian.hpp>
#include <geometry_msgs/Point32.h>

enum general_configuration{UART, RATE, DGNSS, NAV, MSG, CFG, PRT};
enum messages_configutation{GNS = 0, GLL = 1, GSA = 2, GSV = 3, RMC = 4, VTG = 5,  GGA= 6};

double latitude;
double longitude;
double timenow;
bool   position_available;
char   gpsBuffer[80];
unsigned char message_checksum[8]={5,1,2,0,0,0,0,0};
char*  gpsPtr = gpsBuffer;
struct pollfd pfd[1];

geographic_msgs::GeoPoint origin_geo_;
geographic_msgs::GeoPoint actual_coordinate_geo;
ros::Publisher            pub_gps;
ros::Subscriber           sub_rtcm;
// ros::Timer                timer_;


/* GNS-GPS*/
struct gps_gns_payload{ 
  unsigned char	 messageID[5];
  int 	         UTC;
  double	     latitude = 0;
  unsigned char	 latDir;
  double	     longitude = 0;
  unsigned short longDir;      
  unsigned char  modeIndicator[2];      
  uint8_t     	 noSat;
  uint8_t     	 HDOP;
  double         height;
  double         geoidal;
  uint16_t       stationID;
  unsigned char  checksum[2];
} gps_gns;


/* CFG-PRT*/
struct ubx_payload_prt{ 
	unsigned char	  portID;
	unsigned char	  reserved1;
	unsigned short	  txReady;
	unsigned char	  reserved2[8];
	unsigned short	  inProtoMask;
	unsigned short	  outProtoMask;     // if (UBX) ubx_cfg_prt.outProtoMask  |= 0x01;      /*UBX*/
	unsigned char	  reserved3[2];     // if (NMEA) ubx_cfg_prt.outProtoMask |= 0x02;      /*NMEA*/
	unsigned char	  reserved4[2];     // if (RTCM) ubx_cfg_prt.outProtoMask |= 0x20;      /*RTCM3*/
} ubx_cfg_prt;

     /* CFG-UART */
struct ubx_payload_uart{
  unsigned char		portID;
  unsigned char		reserved1;
  unsigned short	txReady;
  unsigned char		mode[4] = {0,0,0,0};
  unsigned int		baudrate;
  unsigned short	inProtoMask;
  unsigned short	outProtoMask;
  unsigned char		flags[2]    = {0,0};
  unsigned char		reserved2[2]= {0,0};
} ubx_cfg_uart;

/* CFG-DGNSS*/
struct ubx_payload_dgnss{ 
	unsigned char	  dgnssMode;
	unsigned char	  reserved1[3] ={0,0};
} ubx_cfg_dgnss;

/* CFG-NAV5- */
struct ubx_payload_nav5{
  unsigned short mask;
  unsigned char	 dynModel;
  unsigned char	 fixMode;
  signed   int	 fixedAlt;
  unsigned int	 fixedAltVar[4];
  unsigned short minElev;
  unsigned char  drLimit;
  unsigned short pDop;
  unsigned short tDop;
  unsigned short pacc;
  unsigned short tacc;
  unsigned char	 staticHoldThresh;
  unsigned char  dgnssTimeout;
  unsigned char	 cnoThreshNumSVs;
  unsigned char	 cnoThresh;
  unsigned char	 reserved1[2];
  unsigned short staticHoldMaxDist;
  unsigned char  utcStandard;
  unsigned char  reserved2[5];
} ubx_cfg_nav;

/* CFG-MSG */
struct ubx_payload_msg{
	unsigned char	clas;
	unsigned char	id;
	unsigned char	rate;
} ubx_cfg_msg;

/* CFG-RATE */
struct ubx_payload_rate{
  unsigned short	measRate;
  unsigned short	navRate;
  unsigned short	timeRef;
} ubx_cfg_rate;

#endif