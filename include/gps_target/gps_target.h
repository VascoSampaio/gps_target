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
#include <map>
#include <thread>
#include <mutex>
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
uint8_t         buf_size = 80;
uint8_t         count_cfg = 0;
bool            configured = false;



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
  int                                                   keyValue;
  boost::variant<unsigned char, uint16_t, int>          idValue;  
  std::string                                           item;  
  std::string                                           Print();
};

// static void ubx_checksum(const unsigned char *data, unsigned len, unsigned char ck[2], unsigned char comparator[2] = {0,0});
void write_for_checking(unsigned char *buf);
void ubx_cfg(int fd, ubx_payload_valset* valset);
std::map<int,ubx_payload_valset*> valset_map ;  	

ubx_payload_valset DGNSSTO{0x201100c4, (unsigned char) 20,"DGNSSTO"};
ubx_payload_valset S_BAS{0x10930012,   (unsigned char) 0,"SBAS"};
ubx_payload_valset GPS_ONLY{0x10930025,(unsigned char) 0,"GPS_ONLY"}; 
ubx_payload_valset rate;
  //FOR USB port

ubx_payload_valset NMEA_OUT;
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

// ubx_payload_valset RTCM_IN{0x10770004,1,"RTCM_IN"};
// ubx_payload_valset RXM{0x2091026b,0,"RXM"};
// ubx_payload_valset NAV{0x20910348,0,"NAV"};

#endif