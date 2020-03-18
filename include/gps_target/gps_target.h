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

#include <multidrone_kml_parser/geographic_to_cartesian.hpp>
#include <geometry_msgs/Point32.h>

// enum general_configuration{SBAS, RATE, VALSET, MSG, CFG, PRT, DGNSS, NAV, UART};
// enum messages_configutation{GNS, GLL, GSA, GSV, RMC, VTG,  GGA};

double          latitude, longitude, timenow;
char            gpsBuffer[80];
char*           gpsPtr = gpsBuffer;
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

unsigned char   message_checksum[8]={5,1,2,0,0,0,0,0};
struct pollfd   pfd[1];

geographic_msgs::GeoPoint origin_geo_;
geographic_msgs::GeoPoint actual_coordinate_geo;
ros::Publisher            pub_gps;
ros::Subscriber           sub_rtcm;

template <class T> struct Configurations
      {
          int         keyValue;
          T           idValue;
          std::string config;
      };

// static void ubx_cfg(int fd, Configurations<class T>* configuration);
// template<> std::map<int,Configurations*<typename N>>;

#endif