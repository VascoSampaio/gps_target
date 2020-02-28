#ifndef GIMBAL_INTERFACE_H
#define GIMBAL_INTERFACE_H

#include <ros/ros.h>
#include <ros/timer.h>
#include <ros/console.h>
#include <chrono>
#include <thread>
#include <boost/bind.hpp>
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <multidrone_msgs/GimbalStatus.h>
#include <multidrone_msgs/CameraStatus.h>
#include <multidrone_msgs/CameraControl.h>
#include <cmath>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

#include <boost/thread.hpp>
#include <libusbp-1/libusbp.hpp>
#include <eigen_conversions/eigen_msg.h>
#include "tf_conversions/tf_eigen.h"
#include <Eigen/Dense>
#include <signal.h>
#include <bitset>


enum STATE
{
  STATE_WAIT, STATE_GOT_MARKER, STATE_GOT_ID, STATE_GOT_LEN, STATE_GOT_HEADER, STATE_GOT_DATA
};

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

// CMD_CONTROL
typedef struct __attribute__((packed)) {
  // uint8_t mode;                            // legacy format: mode is common for all axes
  uint8_t mode[3];	                          // in extended format (firmware ver. 2.55b5+): mode is set independently for each axes
  int16_t speedROLL;
  int16_t angleROLL;
  int16_t speedPITCH;
  int16_t anglePITCH;
  int16_t speedYAW;
  int16_t angleYAW;
} SBGC_cmd_control_t;

// CMD_REALTIME_DATA_3, CMD_REALTIME_DATA_4
typedef struct __attribute__((aligned)) {
  struct {
    int16_t   acc_data;
    int16_t   gyro_data;
  } sensor_data[3];                               // ACC and Gyro sensor data (with calibration) for current IMU (see cur_imu field)
  
  uint16_t    serial_error_cnt;                   // counter for communication errors
  uint16_t    system_error;                       // system error flags, defined in SBGC_SYS_ERR_XX
  uint8_t     reserved1[4];
  int16_t     rc_raw_data[SBGC_RC_NUM_CHANNELS];  // RC signal in 1000..2000 range for ROLL, PITCH, YAW, CMD, EXT_ROLL, EXT_PITCH channels
  int16_t     imu_angle[3];                       // ROLL, PITCH, YAW Euler angles of a camera, 16384/360 degrees
  int16_t     frame_imu_angle[3];                 // ROLL, PITCH, YAW Euler angles of a frame, if known
  int16_t     target_angle[3];                    // ROLL, PITCH, YAW target angle
  uint16_t    cycle_time_us;                      // cycle time in us. Normally should be 800us
  uint16_t    i2c_error_count;                    // I2C errors counter
  uint8_t     reserved2;
  uint16_t    battery_voltage;                    // units 0.01 V
  uint8_t     state_flags1;                       // bit0: motor ON/OFF state;  bits1..7: reserved
  uint8_t     cur_imu;                            // actually selecteted IMU for monitoring. 1: main IMU, 2: frame IMU
  uint8_t     cur_profile;                        // active profile number starting from 0
  uint8_t     motor_power[3];                     // actual motor power for ROLL, PITCH, YAW axis, 0..255

  // Fields below are filled only for CMD_REALTIME_DATA_4 command
  int16_t     rotor_angle[3];                     // relative angle of each motor, 16384/360 degrees
  uint8_t     reserved3;
  int16_t     balance_error[3];                   // error in balance. Ranges from -512 to 512,  0 means perfect balance.
  uint16_t    current;                            // Current that gimbal takes, in mA.
  int16_t     magnetometer_data[3];               // magnetometer sensor data (with calibration)
  int8_t      imu_temp_celcius;                   // temperature measured by the main IMU sensor, in Celsius
  int8_t      frame_imu_temp_celcius;             // temperature measured by the frame IMU sensor, in Celsius
  uint8_t     IMU_G_ERR;
  uint8_t     IMU_H_ERR;
  int16_t     MOTOR_OUT[3];
  uint8_t     reserved4[30];
} SBGC_cmd_realtime_data_t;

class SerialCommand {
  public:
    uint8_t pos;
    uint8_t id;
    uint8_t data[SBGC_CMD_DATA_SIZE];
    uint8_t len;

    void init(uint8_t _id) {
      id = _id;
      len = 0;
      pos = 0;
    }
};

class GimbalInterface  {

	struct BMMCCcmdtiming
	{
		unsigned char function;
		unsigned char value;
		ros::Time timeout;
	};
  std::vector<BMMCCcmdtiming> BMMCCcmd;
  enum{RECORD_CH, IRIS_CH, FOCUS_CH, AFOCUS_CH, ZOOM_CH, ISO_CH, SHUTTER_CH, WHITE_CH, AUDIO_CH, FRAME_CH, CODEC_CH, NFUNCTIONS};
  int BMMCCdata[NFUNCTIONS];

public:

  ros::Publisher status_pub;
  ros::Publisher euler_pub;
  ros::Publisher vel_pub;
  ros::Publisher power_pub;

  ros::Subscriber cmd_sub;
  ros::Subscriber droneInfo_sub;

  boost::asio::io_service io_service_;
  serial_port_ptr port_;
  boost::mutex mutex_;

  std::string portName_;
  // int baud_;

  char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE];
  std::string read_buf_str_;

  char end_of_line_char_ = '>';
  
  SerialCommand cmd_in, cmd_out;
  int len;
  uint8_t checksum = 0;
  enum STATE state = STATE_WAIT;

  int drone_id_, gimbal_id;

  ros::Timer write_to_gimbal_timer;
  ros::Timer gimbal_timer_pub;

  Eigen::Quaterniond gimbal_quat;
  geometry_msgs::Vector3 gimbal_euler;
  geometry_msgs::Vector3 gimbal_euler_;
  geometry_msgs::Vector3 diff;
  geometry_msgs::Vector3 diff_;
  std::vector<geometry_msgs::Vector3> gimbal_ang_vel = std::vector<geometry_msgs::Vector3>(4);  
  geometry_msgs::Vector3 gimbal_motor_power;
  double offset = 0;
  double gimbal_yaw_;
  double drone_yaw_;  
  Eigen::Quaterniond  drone_att_;
  bool   has_gimbal_status_;
  bool   has_gimbal_calibration = false;
  bool   motorON = true;
  double t_0 = 0;
  double tim = 0;
  int    count = 0;
  int initial_pitch = -90;

  //Camera
  

	ros::Publisher cstatus_publisher;
	multidrone_msgs::CameraStatus cs;
	unsigned int SBUSreadData[SBUS_NUMBER_CHANNELS];
	int SBUSch[NFUNCTIONS];
  ros::Timer camera_timer_pub;
	ros::ServiceServer camera_control_service;


  bool cameracontrolServiceCallback (multidrone_msgs::CameraControl::Request &req, multidrone_msgs::CameraControl::Response &res);
	void pushBMMCCcmd(unsigned char BMMCCfunction, unsigned char value, ros::Time timeout);
	void checkBMMCCcmd(void);
	bool send_camera_cmd(unsigned int* cameradata, int arraysize);
	void timerCallback_camera_status(const ros::TimerEvent& event);
	void timerCallback_write_camera(const ros::TimerEvent& event);
  void camera_reset();


  void init();
  void start();
  void async_read_some_();
  void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred);
  void parseData(const std::string &data);
  void SBGC_cmd_control_pack(int command, SBGC_cmd_control_t &p, SerialCommand &cmd);
  uint8_t SBGC_cmd_realtime_data_unpack(SBGC_cmd_realtime_data_t &p, SerialCommand &cmd);
  int write_some(const std::string &buf); // it's public so that camera can also use this function
  int write_some(const char *buf, const int &size); // it's public so that camera can also use this function
  void send_cmd();
  void timerCallback(const ros::TimerEvent&);
  void timerCallbackPub(const ros::TimerEvent&);
  void cmd_callback(const geometry_msgs::Vector3::ConstPtr& msg);
  void droneInfo_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void turnOff();
  void turnOn();
  void calibration();
  double getDiff();

// friend bool send_camera_cmd(unsigned int* cameradata, int noofbytes);



// Constructor
  GimbalInterface(){};

// Destructor
  ~GimbalInterface(){};
  void initi(int _argc, char** _argv);
  void stop();
};

#endif