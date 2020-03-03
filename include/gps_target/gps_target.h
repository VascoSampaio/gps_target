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

#include <multidrone_kml_parser/geographic_to_cartesian.hpp>
#include <geometry_msgs/Point32.h>

enum general_configuration{PRT = 0, UART = 1, MSG = 2, RATE = 3, CFG = 4};
enum messages_configutation{GNS = 0, GLL = 1, GSA = 2, GSV = 3, RMC = 4, VTG = 5,  GGA= 6};

double latitude;
double longitude;
double timenow;
bool   position_available;

geographic_msgs::GeoPoint origin_geo_;
geographic_msgs::GeoPoint actual_coordinate_geo;
ros::Publisher pub_gps;
ros::Timer timer_;


/* GNS-CFG*/
struct gps_gns_payload{ 
	unsigned char	    messageID[5];
	int 	                     UTC;
	double	              latitude;
	unsigned char	          latDir;
	double	             longitude;
	unsigned short	       longDir;      
	unsigned char modeIndicator[2];      
	uint8_t     	           noSat;
  uint8_t     	            HDOP;
  double                  height;
  double                 geoidal;
  uint16_t             stationID;
  unsigned char      checksum[2];
} gps_gns;


    /* CFG-PRT*/
struct ubx_payload_prt{ 
	unsigned char	  portID;
	unsigned char	  reserved1;
	unsigned short	txReady;
	unsigned char	  reserved2[8];
	unsigned short	inProtoMask;
	unsigned short	outProtoMask;     // if (UBX) ubx_cfg_prt.outProtoMask  |= 0x01;      /*UBX*/
	unsigned char	  reserved3[2];     // if (NMEA) ubx_cfg_prt.outProtoMask |= 0x02;      /*NMEA*/
	unsigned char	  reserved4[2];     // if (RTCM) ubx_cfg_prt.outProtoMask |= 0x20;      /*RTCM3*/
} ubx_cfg_prt;

     /* CFG-PRT */
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