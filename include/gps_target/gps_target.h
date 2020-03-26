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
#include<map>
#include "boost/variant.hpp"

#include <multidrone_kml_parser/geographic_to_cartesian.hpp>
#include <geometry_msgs/Point32.h>

// enum general_configuration{SBAS, RATE, VALSET, MSG, CFG, PRT, DGNSS, NAV, UART};
// enum messages_configutation{GNS, GLL, GSA, GSV, RMC, VTG,  GGA};

double          latitude, longitude, timenow;
unsigned char   nmeaBuffer[80];
unsigned char   ubxBuffer[60];
unsigned char*  nmeaPtr = nmeaBuffer;
unsigned char*  ubxPtr = ubxBuffer;
uint8_t         buf_size = 100;

 /* GNS-GPS*/
struct gps_gns_payload{ 
  unsigned char	 messageID[5];
  int 	         UTC;
  double	       latitude = 0;
  unsigned char	 latDir;
  double	       longitude = 0;
  unsigned short longDir;      
  unsigned char  modeIndicator[2];      
  uint8_t     	 noSat;
  uint8_t     	 HDOP;
  double         height;
  double         geoidal;
  uint16_t       stationID;
  unsigned char  checksum[2];
} gps_gns;


unsigned char   message_checksum[8]={5,1,2,0,0,0,0,0};
struct pollfd   pfd[1];

geographic_msgs::GeoPoint origin_geo_;
geographic_msgs::GeoPoint actual_coordinate_geo;
ros::Publisher            pub_gps;
ros::Subscriber           sub_rtcm;



// class MyFieldInterface{
  // public:
  // int m_Size; // of course use appropriate access level in the real code...
  //  ~MyFieldInterface() = default;   
// };

// template <class T> 
struct ubx_payload_valset{ /*: public MyFieldInterface {*/
public: 
  int                                    keyValue;
  unsigned char                            idValue;
  std::string                                item;  
};

// static void ubx_checksum(const unsigned char *data, unsigned len, unsigned char ck[2], unsigned char comparator[2] = {0,0});
void write_for_checking(unsigned char *buf);
void ubx_cfg(int fd, ubx_payload_valset* valset);
std::map<int,ubx_payload_valset*> valset_map ;  	

#endif