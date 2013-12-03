/*
 *  Mikrokopter Flight control Serial Interface
 *  Based on CYPHY lab realization
 *  Copyright (C) 2010, CYPHY lab
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  http://wiki.qut.edu.au/display/cyphy
 *
 *  Fixed and modified by Mike Charikov, 2013 CROC Inc.
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

#include "flightcontrol.h"

ros::Timer timer;
ros::WallTimer walltimer;

unsigned char g_buf[300];
unsigned char g_buf_debug[300];
unsigned char g_txd_buffer[TXD_BUFFER_LEN];
unsigned char g_rxd_buffer[RXD_BUFFER_LEN];
unsigned char g_ReceivedBytes;
unsigned char *g_pRxData;
unsigned char g_RxDataLen;


struct str_Data3D g_Data3D,g_Data3D_Temp;

DebugOut_t g_DebugData,g_DebugData_Temp;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "flightcontrol");
   miko::FlightControl flightcontrol;
   
   ros::spin();
   return 0;
}

namespace miko
{
  FlightControl::FlightControl()
  {
	ROS_INFO ("Creating FlightControl Interface");
	ros::NodeHandle nh;
	
	pub = nh.advertise<cyphy_serial_driver::mikoImu>("mikoImu", 100);
    pub_stdImu = nh.advertise<sensor_msgs::Imu>("imu/data", 100);

	mikoCmdSubscriber = nh.subscribe("c_command", 100, &FlightControl::mikoCmdCallback, this);

    // parameter Setting
    if (!nh.getParam("/MikoControl/device", port_)) port_ = "/dev/ttyUSB1";
    if (!nh.getParam("/MikoControl/baud", speed_)) speed_ = 57600;
    if (!nh.getParam("/MikoControl/frequency", freq_)) freq_ = 50.0;


    if (freq_ <= 0.0) ROS_FATAL ("Invalid frequency param");
	ros::Duration d (1.0 / freq_);

	// Reference input initialization
	ExternControl.Digital[0] =0;
	ExternControl.Digital[1] =0;
	ExternControl.RemoteButtons =0;
	ExternControl.Pitch =0;
	ExternControl.Roll =0;
	ExternControl.Yaw =0;
	ExternControl.Throttle =0;
	ExternControl.Height =0;
	ExternControl.free =0;
	ExternControl.Frame =0;
	ExternControl.Config =1;
	Throttle_Direction=true;

	g_DebugData.Digital[0]=0;
	g_DebugData.Digital[1]=0;
	for(int i=0;i<32;i++) g_DebugData.Analog[i]=0;

	g_ReceivedBytes=0;
	g_pRxData=0;
	g_RxDataLen=0;
		
    	// **** set up intefaces
    serialInterface_ = new miko::SerialInterface(port_, speed_);
	serialInterface_->serialport_bytes_rx_ = 0;
	serialInterface_->serialport_bytes_tx_ = 0;


	timer= nh.createTimer(ros::Duration(0.020), &FlightControl::spin,this);  //50ms timer 20Hz
	ROS_INFO ("Serial setup finished");
  }
  

  FlightControl::~FlightControl()
  {
    
    ROS_INFO ("Destroying FlightControl Interface");
  }


  void FlightControl::spin(const ros::TimerEvent & e)
  {
	int nLength=0;
	uint8_t interval=5;
	
	SendOutData('d', FC_ADDRESS, 1, &interval, sizeof(interval)); // Request debug data from FC
	mikoImu.header.stamp = ros::Time::now();

	//ros::WallDuration(0.01).sleep(); // 10ms delay
	nLength=serialInterface_->getdata(g_rxd_buffer, 255);
	
	if(nLength>0)
	{

                serialInterface_->Decode64();
		serialInterface_->ParsingData();
             
		//=================================================================
		// Coordinate and unit conversion from Mikrokopter to Cyphy model.
		//=================================================================

		// Put negative(minus) all angles to covert all axies into a right hand coordinate.
		// Please have a look MK Coordinate system V1.0 documentation in order to understand coordinate systems.
		
		MyAttitude.AnglePitch = -DEG_TO_RAD(g_DebugData.Analog[ANGLE_PITCH]/10.);
		MyAttitude.AngleRoll = -DEG_TO_RAD(g_DebugData.Analog[ANGLE_ROLL]/10.);

		// To make IMU Yaw coordinate same as Laser Yaw coordinate.
		if(g_DebugData.Analog[ANGLE_YAW]>=180) 
			MyAttitude.AngleYaw = (double)(g_DebugData.Analog[ANGLE_YAW]-360.);
		else 
			MyAttitude.AngleYaw = (double)g_DebugData.Analog[ANGLE_YAW];

		MyAttitude.AngleYaw = DEG_TO_RAD(MyAttitude.AngleYaw);

		// Normalize Acceleration data as 1G.
		// Please have a look MK Coordinate system V1.0 documentation in order to understand coordinate systems.
		MyAttitude.ACCX = -(g_DebugData.Analog[ACC_X]/606.)*g;
		MyAttitude.ACCY = (g_DebugData.Analog[ACC_Y]/603.)*g;
		//MyAttitude.ACCZ = (g_DebugData.Analog[ACC_Z_FILTER]/15.5); // m/s^2
		MyAttitude.ACCZ = (g_DebugData.Analog[ACC_Z_ADC]/15.5); // m/s^2

		//
		//	The unit of angle is radian and m/s^2 for acceleration (not g)
		//
		mikoImu.anglePitch = MyAttitude.AnglePitch; 
		mikoImu.angleRoll = MyAttitude.AngleRoll;
		mikoImu.angleYaw = MyAttitude.AngleYaw;
		mikoImu.linear_acceleration.x = MyAttitude.ACCX;
		mikoImu.linear_acceleration.y = MyAttitude.ACCY;
		mikoImu.linear_acceleration.z = MyAttitude.ACCZ;
		mikoImu.stick_throttle = g_DebugData.Analog[GAS];
		mikoImu.barome_height = g_DebugData.Analog[HEIGHT];

        stdImuMsg.linear_acceleration.x=MyAttitude.ACCX;
        stdImuMsg.linear_acceleration.y=MyAttitude.ACCY;
        stdImuMsg.linear_acceleration.z=MyAttitude.ACCZ;

        tfImuQuaternion = tf::createQuaternionFromRPY(MyAttitude.AngleRoll, MyAttitude.AnglePitch, MyAttitude.AngleYaw);
        tf::quaternionTFToMsg(tfImuQuaternion, geometryImuQuaternion);

        stdImuMsg.orientation = geometryImuQuaternion;
        stdImuMsg.angular_velocity_covariance[0]=-1;
        stdImuMsg.linear_acceleration_covariance[0]=-1;

		if(g_DebugData.Analog[BATT]<BATT_MAX) mikoImu.batt = g_DebugData.Analog[BATT];


		for (int i=0; i<32; i++) mikoImu.debugData[i]=g_DebugData.Analog[i];



		pub.publish(mikoImu);
        pub_stdImu.publish(stdImuMsg);
	}
else
	ROS_INFO("crc bad!");
}

  void FlightControl::SendOutData(uint8_t cmd, uint8_t addr, uint8_t numofbuffers, ...) // uint8_t *pdata, uint8_t len, ...
  { 
	va_list ap;
	uint16_t pt = 0;
	uint8_t a,b,c;
	uint8_t ptr = 0;

	uint8_t *pdata = 0;
	int len = 0;

	g_txd_buffer[pt++] = '#';			// Start character
	g_txd_buffer[pt++] = 'a' + addr;	// Address (a=0; b=1,...)
	g_txd_buffer[pt++] = cmd;			// Command

		va_start(ap, numofbuffers);
		if(numofbuffers)
		{
			pdata = va_arg(ap, uint8_t*);
			len = va_arg(ap, int);
			ptr = 0;
			numofbuffers--;
		}

		while(len)
		{
			if(len)
			{
				a = pdata[ptr++];
				len--;
				if((!len) && numofbuffers)
				{
					pdata = va_arg(ap, uint8_t*);
					len = va_arg(ap, int);
					ptr = 0;
					numofbuffers--;
				}
			}
			else a = 0;
			if(len)
			{
				b = pdata[ptr++];
				len--;
				if((!len) && numofbuffers)
				{
					pdata = va_arg(ap, uint8_t*);
					len = va_arg(ap, int);
					ptr = 0;
					numofbuffers--;
				}
			}
			else b = 0;
			if(len)
			{
				c = pdata[ptr++];
				len--;
				if((!len) && numofbuffers)
				{
					pdata = va_arg(ap, uint8_t*);
					len = va_arg(ap, int);
					ptr = 0;
					numofbuffers--;
				}
			}
			else c = 0;
			g_txd_buffer[pt++] = '=' + (a >> 2);
			g_txd_buffer[pt++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
			g_txd_buffer[pt++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
			g_txd_buffer[pt++] = '=' + ( c & 0x3f);
		} //while
		va_end(ap);
		AddCRC(pt); // add checksum after data block and initates the transmission
	}

	void FlightControl::AddCRC(uint16_t datalen)
	{
		uint16_t tmpCRC = 0, i;
		for(i = 0; i < datalen; i++)
		{
			tmpCRC += g_txd_buffer[i];
		}
		tmpCRC %= 4096;
		g_txd_buffer[datalen++] = '=' + tmpCRC / 64;
		g_txd_buffer[datalen++] = '=' + tmpCRC % 64;
		g_txd_buffer[datalen++] = '\r';
		serialInterface_->output(g_txd_buffer,datalen);
	}

    void FlightControl::mikoCmdCallback(const cyphy_serial_driver::mikoCmd& msg)
    {
                  
		 ExternControl.Pitch = msg.pitch;
		 ExternControl.Roll = msg.roll;
		 ExternControl.Yaw = msg.yaw;		 
		 ExternControl.Throttle = msg.throttle;

		 if(ExternControl.Pitch >=80) ExternControl.Pitch=80;
		 if(ExternControl.Pitch <=-80) ExternControl.Pitch=-80;

		 if(ExternControl.Roll >=80) ExternControl.Roll=80;
		 if(ExternControl.Roll <=-80) ExternControl.Roll=-80;
		 
		 if(ExternControl.Yaw >=60) ExternControl.Yaw=60;
		 if(ExternControl.Yaw <=-60) ExternControl.Yaw=-60;

		 if(ExternControl.Throttle <= 0) ExternControl.Throttle =0;

		
		 //ROS_INFO (">>>> %d, %d, %d, %d",ExternControl.Pitch, ExternControl.Roll, ExternControl.Yaw, ExternControl.Throttle);

		 SendOutData('b', FC_ADDRESS, 1, (uint8_t*)&ExternControl, sizeof(ExternControl));
		 		
	}
} // namespace miko
