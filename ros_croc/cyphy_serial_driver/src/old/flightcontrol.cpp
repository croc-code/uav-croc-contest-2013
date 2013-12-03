/*
 *  Mikrokopter Flight control Serial Interface
 *  Copyright (C) 2010, CYPHY lab
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  http://wiki.qut.edu.au/display/cyphy
 *
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

#include "flightcontrol.h"

ros::Timer timer;
ros::WallTimer walltimer;

unsigned char th_cnt=0;


unsigned char g_buf[300];
unsigned char g_buf_debug[300];
unsigned char g_txd_buffer[TXD_BUFFER_LEN];
unsigned char g_rxd_buffer[RXD_BUFFER_LEN];
unsigned char g_ReceivedBytes;
unsigned char *g_pRxData;
unsigned char g_RxDataLen;
long g_nCounter=0;
long g_nCounter1=0;
double g_pitch_temp;
double g_roll_temp;
double g_pitch_new;
double g_pitch_cmd;
double g_roll_cmd;
double g_roll_new;
double g_yaw_new;
double g_yaw_temp;
double dblDistance;
double g_height;
int g_throttle_cmd;
double g_goal_height;

long counter=0;
long counter1=0;

struct str_Data3D g_Data3D,g_Data3D_Temp;
DebugOut_t g_DebugData,g_DebugData_Temp;
Position_t g_PositionFromAcc,g_PostPosition,g_EstimatePosition,g_CurPosition,g_GoalPosition,g_GoalVel,g_errorPos;
Control_t g_StickControl,g_PostStickControl;
Velocity_t g_W_Vel,g_B_Vel,g_B_ACC,g_B_esti_Vel,g_B_predic_esti_Vel,g_B_goal_Vel;

int main(int argc, char **argv)
{
   int interval=0;
   int nLength;
   ros::init(argc, argv, "flightcontrol");
   miko::FlightControl::FlightControl flightcontrol;
  flightcontrol.last_time = ros::Time::now();
  flightcontrol.last_time_ = ros::Time::now();
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

	mikoCmdSubscriber = nh.subscribe ("mikoCmd", 100, &FlightControl::mikoCmdCallback, this);


    // **** parameter Setting
    freq_ = 50.0;
    port_ = "/dev/ttyUSB0";
    speed_ = 57600;
	//speed_ = 115200;
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

	g_height=0;
	g_throttle_cmd=0;
 	g_goal_height=1.0; // 1m

	myYawAngle=0;
	
    // **** set up intefaces

    serialInterface_ = new miko::SerialInterface::SerialInterface (port_, speed_);
    serialInterface_->serialport_bytes_rx_ = 0;
    serialInterface_->serialport_bytes_tx_ = 0;

// ROS PID TOOL_BOX initilization
    pid_height.initPid( 35.0 , 25.0 , 0.0 , 0 , 0 );

	fd = fopen("accz_mk_filter","w");
	fd_h = fopen("batt_discharge","w");
	
	// To create CSV type log file.
	fprintf(fd,"STICK_GAS,PRESSURE,ACC_Z,index\n");
	fprintf(fd_h,"battAdc,gas,index\n");

	timer= nh.createTimer(ros::Duration(0.050), &FlightControl::spin,this);  //50ms timer 20Hz
    ROS_INFO ("Serial setup finished");
  }
  

  FlightControl::~FlightControl()
  {
    fclose(fd);
    fclose(fd_h);

    ROS_INFO ("Destroying FlightControl Interface");
  }


  void FlightControl::spin(const ros::TimerEvent & e)
  {
	int nLength=0;
	uint8_t interval=5;

	
#ifdef CHECK_DURA
	struct timeval start, end , total_start,total_end,acc_start,acc_end;
	gettimeofday(&start, NULL);
	gettimeofday(&total_start, NULL);
	gettimeofday(&acc_start, NULL);
#endif

	SendOutData('d', FC_ADDRESS, 1,&interval,sizeof(interval)); // Request debug data from FC

#ifdef CHECK_DURA
	gettimeofday(&end, NULL);
    double dur = ((end.tv_sec   * 1000000 + end.tv_usec  ) - 
                (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;
	
    ROS_INFO("Request debug data dur:\t %.3f ms", dur);
#endif


	mikoImu.header.stamp = ros::Time::now();


#ifdef CHECK_DURA
	gettimeofday(&start, NULL);
#endif

	ros::WallDuration(0.01).sleep(); // 10ms delay
	
#ifdef CHECK_DURA
	gettimeofday(&end, NULL);
		dur = ((end.tv_sec   * 1000000 + end.tv_usec  ) - 
                (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;
	
    ROS_INFO("Check delay dur:\t %.3f ms", dur);
#endif



#ifdef CHECK_DURA
    gettimeofday(&start, NULL);
#endif



	nLength=serialInterface_->getdata(g_rxd_buffer, 255);


#ifdef CHECK_DURA
	gettimeofday(&end, NULL);
    dur = ((end.tv_sec   * 1000000 + end.tv_usec  ) - 
                (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;
	ROS_INFO("getdata dur:\t %.3f ms", dur);
#endif


	if(nLength>0)
	{

#ifdef CHECK_DURA
    gettimeofday(&start, NULL);
#endif


	serialInterface_->Decode64();


#ifdef CHECK_DURA
	gettimeofday(&end, NULL);
    dur = ((end.tv_sec   * 1000000 + end.tv_usec  ) - 
                (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;
	ROS_INFO("Decode64 dur:\t %.3f ms", dur);
#endif


#ifdef CHECK_DURA
    gettimeofday(&start, NULL);
#endif

	serialInterface_->ParsingData();


#ifdef CHECK_DURA
	gettimeofday(&end, NULL);
    dur = ((end.tv_sec   * 1000000 + end.tv_usec  ) - 
                (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;
	ROS_INFO("ParsingData dur:\t %.3f ms", dur);
#endif

//=================================================================
// Coordinate and unit conversion from Mikrokopter to Cyphy model.
//=================================================================

// Put negative(minus) all angles to covert all axies into a right hand coordinate.
// Please have a look MK Coordinate system V1.0 documentation in order to understand coordinate systems.
        
		MyAttitude.AnglePitch = -DEG_TO_RAD(g_DebugData.Analog[ANGLE_PITCH]/10.);
		MyAttitude.AngleRoll = -DEG_TO_RAD(g_DebugData.Analog[ANGLE_ROLL]/10.);

		// To make IMU Yaw coordinate same as Laser Yaw coordinate.

		if(g_DebugData.Analog[ANGLE_YAW]>=180) MyAttitude.AngleYaw=(double)(g_DebugData.Analog[ANGLE_YAW]-360.);
		else MyAttitude.AngleYaw= (double)g_DebugData.Analog[ANGLE_YAW];

		MyAttitude.AngleYaw = DEG_TO_RAD(MyAttitude.AngleYaw);

		// Normalize Acceleration data as 1G.
		// Please have a look MK Coordinate system V1.0 documentation in order to understand coordinate systems.
		MyAttitude.ACCX = -(g_DebugData.Analog[ACC_X]/606.)*g;
		MyAttitude.ACCY = (g_DebugData.Analog[ACC_Y]/603.)*g;
		MyAttitude.ACCZ = (g_DebugData.Analog[ACC_Z_FILTER]/15.5); // m/s^2

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

		if(g_DebugData.Analog[BATT]<BATT_MAX) mikoImu.batt = g_DebugData.Analog[BATT];
		
		pub.publish(mikoImu);
		counter++;

	    if(g_DebugData.Analog[BATT] < BATT_MAX) fprintf(fd_h,"%d,%d,%d\n",g_DebugData.Analog[BATT],g_DebugData.Analog[GAS],counter1);
		counter1++;
		
//===================================================
// Mikrokopter transmitter setting (Mode2)
//===================================================

// There are two sticks to control the quadrotor
//             Throttle                Pitch
//             ^                         ^
//             |                         |
//             |                         |
//       <-----|------>Yaw        <------|------> Roll
//             |                         |
//             |                         |
//             |                         |
//             v                         v


//===================================================
// Mikrokoper coordinate and my coordinate system
//===================================================

// Mikrokopter doesn't have concept of any axis. It only has pitch,roll and yaw.
// Red rod is the front and the opposite one is the rear. Pitch(Nick) means forwards and backwrads
// movements. In contrast, Roll means left and right movements.
// When we push all the way up the pitch stick(Right stick on mode2), "StickNick" value(516) is transmitted
// to the FlightControl(FC). This implies that moving forward as fastest speed.
// Actually it means maximum angular velocity of pitch. 
// When we pull all the way down the pitch stick, "StickNick" value(-592) is transmitted
// to the FlightControl. This implies that moving backward as fastest speed.
// On the other hand, When push all the way right the Roll stick, "StickRoll" value(-516) is transmitted
// to the FC. when push all the way left the Roll stick, "StickRoll" value(512) is transmitted to the FC.

// Because Mikrokopter doesn't have clear coordinate systems, we need to define our own coordinates.
// Commonly X-axis is the forwards of the robots and rigt direction is the positive y-axis.
// World coordinate frame is 
//
//              x
//              ^
//              |
//              |
//        ------|------> Y
//              |
//              |                         
//              |
//
// Pitch angle implies rotation around Y axis and Roll angle implies rotion around X axis.
// Because the position data from ROS canonical scan matcher provides a opposite Y position as ours, inverting the Y error.error_k 
	}
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
         //ROS_INFO("mikoCmdCallback");
         
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
		 if(ExternControl.Throttle >=MAX_THROTTLE) ExternControl.Throttle = MAX_THROTTLE;		 
		 SendOutData('b', FC_ADDRESS,1,(uint8_t*)&ExternControl, sizeof(ExternControl));
	}
} // namespace miko
