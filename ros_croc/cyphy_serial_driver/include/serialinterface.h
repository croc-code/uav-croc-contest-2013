/*  
 *  Mikrokopter FlightControl Serial Interface
 *  Copyright (C) 2010, Cyphy Lab.
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  https://wiki.qut.edu.au/display/cyphy
 *
 *
 *  Note that the part of this code,(SerialInterface(), ~SerialInterface(),output() is borrowed from CCNY asctec_autopilot serial interface communication 
 *  package. However, the core part of this code, getdata(),ParsingData(), and Decode64() were written by Inkyu.
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MIKO_FLIGHTCONTROL_SERIALINTERFACE_H
#define MIKO_FLIGHTCONTROL_SERIALINTERFACE_H

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>
#include <math.h>

#include <ros/ros.h>

//#include "crc16.h"
//#include "telemetry.h"
#include "flightcontrol.h"

#define TXD_BUFFER_LEN  300
#define RXD_BUFFER_LEN  300
#define MAX_PITCH_THRESHOLD 360
#define MAX_ROLL_THRESHOLD 360
#define MAX_YAW_THRESHOLD 360
#define MAX_HEIGHT_THRESHOLD 200
#define MAX_ACC_THRESHOLD 400
//#define FC_DEBUG

namespace miko
{
  class SerialInterface
  {
  public:
    SerialInterface (std::string port, uint32_t speed);
    ~SerialInterface ();

    void output (char *output, int len);
    void output (unsigned char *output, int len);
   // bool getPackets (Telemetry *telemetry);
    void Decode64(void);
    void ParsingData(void);
   // void sendControl (Telemetry *telemetry);
    void dumpDebug (void);
    int getdata (unsigned char *buf, int len);
 //   bool getPacket (char *spacket, unsigned char &packet_type, unsigned short &packet_crc, unsigned short &packet_size);

    uint32_t serialport_bytes_rx_;
    uint32_t serialport_bytes_tx_;
    int *scan;
    bool status;
    int pt[800];
    int counter;


    bool Initialized;
    int count;

 private:
	  speed_t bitrate (int);
    void flush ();
    void drain ();
    void stall (bool);
    int wait (int);

    int dev_;
    std::string serialport_name_;
    uint32_t serialport_speed_;
    speed_t serialport_baud_;
  };
}
#endif
