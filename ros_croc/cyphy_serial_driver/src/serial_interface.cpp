/*
 *  Mikrokopter FlightControl Serial Interface
 *  Copyright (C) 2010, Cyphy Lab.
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  https://wiki.qut.edu.au/display/cyphy
 *  Note that the part of this code,(SerialInterface(), ~SerialInterface(),output()) is borrowed from CCNY asctec_autopilot serial interface communication 
 *  package. However, the core part of this code, getdata(),ParsingData(), and Decode64() were written by Inkyu.
 *
 *  Fixed and modified by Mike Charikov, 2013, CROC Inc. 
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

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>

#include <ros/ros.h>

#include "flightcontrol.h"
#include "serialinterface.h"

#include <iostream>

// C++ is a horrible version of C
extern "C" {
  #include <unistd.h>
  #include <fcntl.h>
}

static const std::string base64_chars = 
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";


extern unsigned char g_ReceivedBytes;
extern unsigned char *g_pRxData;
extern unsigned char g_RxDataLen;
extern unsigned char g_rxd_buffer[RXD_BUFFER_LEN];
extern unsigned char g_buf[300];
extern unsigned char g_buf_debug[300];
extern struct str_Data3D g_Data3D,g_Data3D_Temp;
extern DebugOut_t g_DebugData,g_DebugData_Temp;


namespace miko
{
  SerialInterface::SerialInterface (std::string port, uint32_t speed):serialport_name_ (port), serialport_speed_ (speed)
  {
    struct termios tio;
      status = false;
      serialport_baud_ = bitrate (serialport_speed_);
      ROS_INFO ("Initializing serial port...");

      dev_ = open(serialport_name_.c_str (),O_RDWR | O_NOCTTY | O_NDELAY);
      ROS_DEBUG ("dev: %d", dev_);
      ROS_ASSERT_MSG (dev_ != -1, "Failed to open serial port: %s %s", serialport_name_.c_str (), strerror (errno));

      ROS_ASSERT_MSG (tcgetattr (dev_, &tio) == 0, "Unknown Error: %s", strerror (errno));

      cfsetispeed (&tio, serialport_baud_);
      cfsetospeed (&tio, serialport_baud_);

      tio.c_iflag = 0;
      tio.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
      tio.c_iflag |= IGNBRK;

      tio.c_oflag = 0;
      tio.c_oflag &= ~(OPOST | ONLCR);

      tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
      tio.c_cflag &= ~(PARENB | CRTSCTS | CSTOPB);

      tio.c_lflag = 0;
      tio.c_lflag |= NOFLSH;
      tio.c_lflag &= ~(ISIG | IEXTEN | ICANON | ECHO | ECHOE);

      ROS_ASSERT_MSG (tcsetattr (dev_, TCSADRAIN, &tio) == 0, "Unknown Error: %s", strerror (errno));

      tio.c_cc[VMIN] = 0;
      tio.c_cc[VTIME] = 0;

      tcflush (dev_, TCIOFLUSH);

	  Initialized=false;

      ROS_ASSERT_MSG (dev_ != NULL, "Could not open serial port %s", serialport_name_.c_str ());
      ROS_INFO ("Successfully connected to %s, Baudrate %d\n", serialport_name_.c_str (), serialport_speed_);
  }

  SerialInterface::~SerialInterface ()
  {
    ROS_DEBUG ("Destroying Serial Interface");
    flush ();
    close (dev_);
  }

  void SerialInterface::flush ()
  {
    tcflush (dev_, TCIOFLUSH);
  }

  void SerialInterface::drain ()
  {
    ROS_ASSERT_MSG (tcdrain (dev_) == 0, "Drain Error: %s", strerror (errno));
  }

  int SerialInterface::wait (int bytes_requested)
  {
    int bytes_available=0;
    unsigned int i=0;

    while (bytes_available < bytes_requested)
    {
      ioctl(dev_,FIONREAD,&bytes_available);
      usleep(1);
      if (i>650 && bytes_available < bytes_requested)
      {
        ROS_ERROR("Timeout: %d bytes available %d bytes requested",bytes_available,bytes_requested);
        return bytes_available;
      }
      i++;
    }
    return bytes_available;
  }

  speed_t SerialInterface::bitrate (int Bitrate)
  {
    switch (Bitrate)
    {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      default:                 // invalid bitrate
        return B0;
    }
  }
  void SerialInterface::output (char *output, int len)
  {
    int i;
    ROS_DEBUG ("SerialInterface::output()");
    serialport_bytes_tx_ += len;
    i = write (dev_, output, len);
    if (i != len)
    {
      ROS_ERROR ("Error wrote %d out of %d element(s): %s", i, len, strerror (errno));
      ROS_BREAK ();
    }
    ROS_DEBUG ("Write completed");
  }

  void SerialInterface::output (unsigned char *output, int len)
  {
    int i;
    ROS_DEBUG ("SerialInterface::output()");
    serialport_bytes_tx_ += len;
    i = write (dev_, output, len);
    if (i != len)
    {
      ROS_ERROR ("Error wrote %d out of %d element(s): %s", i, len, strerror (errno));
      ROS_BREAK ();
    }
  }

  int SerialInterface::getdata (unsigned char *buf, int len)
  {

    int i;
    int nLength;
    int ReceivedBytes=0;
    nLength = read(dev_, g_buf, len);

    //ROS_INFO(">>> %d >>> %s", nLength, g_buf);
 	
    ReceivedBytes = nLength;
    static uint16_t crc;
    static uint8_t ptr_rxd_buffer = 0;
    uint8_t crc1, crc2;
    uint8_t c;

    for(i=0;i<nLength;i++)
    {
		c = g_buf[i];  // catch the received byte

		// the rxd buffer is unlocked
		if((ptr_rxd_buffer == 0) && (c == '#')) // if rxd buffer is empty and syncronisation character is received
		{
			g_rxd_buffer[ptr_rxd_buffer++] = c; // copy 1st byte to buffer
			crc = c; // init crc
		}
		else if (ptr_rxd_buffer < RXD_BUFFER_LEN) // collect incomming bytes
		{
			if(c != '\r') // no termination character
			{
				g_rxd_buffer[ptr_rxd_buffer++] = c; // copy byte to rxd buffer
				crc += c; // update crc
			}
			else // termination character was received
			{
				// the last 2 bytes are no subject for checksum calculation
				// they are the checksum itself
				crc -= g_rxd_buffer[ptr_rxd_buffer-2];
				crc -= g_rxd_buffer[ptr_rxd_buffer-1];
				// calculate checksum from transmitted data
				crc %= 4096;
				crc1 = '=' + crc / 64;
				crc2 = '=' + crc % 64;
				// compare checksum to transmitted checksum bytes
				if((crc1 == g_rxd_buffer[ptr_rxd_buffer-2]) && (crc2 == g_rxd_buffer[ptr_rxd_buffer-1]))
				{   // checksum valid
					g_rxd_buffer[ptr_rxd_buffer] = '\r'; // set termination character
					ReceivedBytes = ptr_rxd_buffer + 1;// store number of received bytes
					// if 2nd byte is an 'R' enable watchdog that will result in an reset
				}
				else
				{	// checksum invalid
					//rxd_buffer_locked = FALSE; // unlock rxd buffer
					ptr_rxd_buffer = 0; // reset rxd buffer pointer
					return -1;
				}
				ptr_rxd_buffer = 0; // reset rxd buffer pointer
			}
		}
		else // rxd buffer overrun
		{
			ptr_rxd_buffer = 0; // reset rxd buffer
		}
    }// for


    return ReceivedBytes;
  }

void SerialInterface::ParsingData(void)
{


       	switch((g_rxd_buffer[1]-98))
	{
                case 0: //FC_ADDRESS:
			switch(g_rxd_buffer[2])
			{
				case 67:
					memcpy(&g_Data3D, (uint8_t*)g_pRxData, sizeof(g_Data3D));
					break;
				case 68:
		                        //ROS_INFO("QQQQQQ %d QQQQQQ %d", g_RxDataLen, sizeof(g_DebugData));
                                        memcpy(&g_DebugData_Temp, (uint8_t*)g_pRxData, sizeof(g_DebugData));

					if(
							 ( abs(g_DebugData_Temp.Analog[ANGLE_PITCH])>1000) ||
							 ( abs(g_DebugData_Temp.Analog[ANGLE_ROLL])>1000) ||
							 ( abs(g_DebugData_Temp.Analog[ANGLE_YAW])>1000) ||
							 ( abs(g_DebugData_Temp.Analog[ACC_X])>700) ||
					 		 ( abs(g_DebugData_Temp.Analog[ACC_Y])>700) ||
					 		 ( abs(g_DebugData_Temp.Analog[ACC_Z_ADC])>700)
					  )
					  {
					  	/*ROS_INFO("Wrong packet received: %d %d %d %d %d %d ", g_DebugData_Temp.Analog[ANGLE_PITCH],
												      g_DebugData_Temp.Analog[ANGLE_ROLL],
												      g_DebugData_Temp.Analog[ANGLE_YAW],
							 					      g_DebugData_Temp.Analog[ACC_X],
					 		 					      g_DebugData_Temp.Analog[ACC_Y],
					 		 					      g_DebugData_Temp.Analog[ACC_Z_ADC]); */
						
					  }
					  else 
						g_DebugData = g_DebugData_Temp;
						/*ROS_INFO("Good packet received: %d %d %d %d %d %d ", g_DebugData_Temp.Analog[ANGLE_PITCH],
												      g_DebugData_Temp.Analog[ANGLE_ROLL],
												      g_DebugData_Temp.Analog[ANGLE_YAW],
							 					      g_DebugData_Temp.Analog[ACC_X],
					 		 					      g_DebugData_Temp.Analog[ACC_Y],
					 		 					      g_DebugData_Temp.Analog[ACC_Z_ADC]);*/
					  break;

			}
	}
	g_pRxData = 0;
	g_RxDataLen = 0;
}


 void SerialInterface::Decode64(void)
{
	uint8_t a,b,c,d;
	uint8_t x,y,z;
	uint8_t ptrIn = 3;
	uint8_t ptrOut = 3;
	uint8_t len = g_ReceivedBytes - 6;

	while(len)
	{
		a = g_rxd_buffer[ptrIn++] - '=';
		b = g_rxd_buffer[ptrIn++] - '=';
		c = g_rxd_buffer[ptrIn++] - '=';
		d = g_rxd_buffer[ptrIn++] - '=';

		x = (a << 2) | (b >> 4);
		y = ((b & 0x0f) << 4) | (c >> 2);
		z = ((c & 0x03) << 6) | d;

                

		if(len--) g_rxd_buffer[ptrOut++] = x; else break;
		if(len--) g_rxd_buffer[ptrOut++] = y; else break;
		if(len--) g_rxd_buffer[ptrOut++] = z; else break;
                
	}
	g_pRxData = &g_rxd_buffer[3];
	g_RxDataLen = ptrOut - 3;

	
}
 


}



