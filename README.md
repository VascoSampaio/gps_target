# gps_target
Gps target PiC32, ublox M9N

Code for deploying in both PIC32, Computer and Teensy board to have a mobile ublox GPS sending location info - Position, Velocity, Performance Measurements - through radio - CC1125 - to a base station. Differential mode accomplished by using SBAS and in progress with RTK. Analyzing tools to check the location info both in real time and with rosbags. 

## PIC32

- Code in /gps_target_PIC.X.
- Interface with CC1125 to configure and get messages from the GPS ans dend them partially parsed to the receiver. 
- Interface to receive RTK messeges and send them to the GPS to achieve differential mode (TO BE FINISHED).

## Teensy

- Code in /teensy_library.
- Interface with CC1125 to deploy a receiver which sends to serial the received messages.
- Able to get messages from RTK and send them partially parsed through radio.
- Orange LED always on when in RX mode and beeping when it is sending RTK messages.

## Computer

- Code in /src and /include.
- Interface with teensy or GPS responsible for parsing the NMEA/PUBX/UBX messages ignoring faulty messages.
- Responsible for sending RTK messages and publishing topics with the desired information
- Debug mode availabre where all the messsages are printed in the terminal - flag 'debug'
- When directly configuring GPS - flag 'config' - it can use normal GNSS messages or PUBX with more information - flag 'survey'.

## Analyzer

- Code in /Analyzer.
- Plotting of the infoirmation that comes from the topic /surveymessage.


