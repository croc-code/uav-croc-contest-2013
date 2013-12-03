# This is a message to hold data from an Mikrokopter IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational angle should be in rad
#
# by Inkyu

Header header

float64 anglePitch
float64 angleRoll
float64 angleYaw
int32 stick_throttle
int32 barome_height
int32 batt
int32 extern_on

geometry_msgs/Vector3 linear_acceleration

int32[32] debugData

