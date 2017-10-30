
#include <stdio.h>
#include "../inc/ZH_DHA_Param_Area.h"
//////////////////////////////////////////////////////////////////////////

#define WIN32 
//#define EARTH_RADIUS (6378137.0)
//#define EARTH_RADIUS 6371.315
//#define EARTH_RADIUS 6366.707
#define EARTH_RADIUS 6378.388   
#define M_PI       3.14159265358979323846

#define SENSORMODULE_ACCELERATED_FIXED_PARAM  (32768*2*9.8)
#define SENSORMODULE_ANGLES_FIXED_PARAM (32768.0*250)
#define SENSORMODULE_ANGLE_FIXED_PARAM (32768.0*180)


double fabsf(double x){

	if(x>=0) return x;
	else return -1*x;   

} 

 double LantitudeLongitudeDist(double lon1, double lat1, double lon2, double lat2) {
	 double x1,x2,y1,y2,z1,z2,dist,theta,d;
	double radLat1 = M_PI*lat1 / 180.0f;
	double radLat2 = M_PI*lat2 / 180.0f;

	double radLon1 = M_PI*lon1 / 180.0f;
	double radLon2 = M_PI*lon2 / 180.0f;

#ifndef WIN32
	if (radLat1 < 0)
		radLat1 = Math.PI / 2 + Math.abs(radLat1);// south  
	if (radLat1 > 0)
		radLat1 = Math.PI / 2 - Math.abs(radLat1);// north  
	if (radLon1 < 0)
		radLon1 = Math.PI * 2 - Math.abs(radLon1);// west  
	if (radLat2 < 0)
		radLat2 = Math.PI / 2 + Math.abs(radLat2);// south  
	if (radLat2 > 0)
		radLat2 = Math.PI / 2 - Math.abs(radLat2);// north  
	if (radLon2 < 0)
		radLon2 = Math.PI * 2 - Math.abs(radLon2);// west  
	double x1 = EARTH_RADIUS * Math.cos(radLon1) * Math.sin(radLat1);
	double y1 = EARTH_RADIUS * Math.sin(radLon1) * Math.sin(radLat1);
	double z1 = EARTH_RADIUS * Math.cos(radLat1);

	double x2 = EARTH_RADIUS * Math.cos(radLon2) * Math.sin(radLat2);
	double y2 = EARTH_RADIUS * Math.sin(radLon2) * Math.sin(radLat2);
	double z2 = EARTH_RADIUS * Math.cos(radLat2);
	double d = Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
	//ÓàÏÒ¶¨ÀíÇó¼Ð½Ç  
	double theta = Math.acos((EARTH_RADIUS * EARTH_RADIUS + EARTH_RADIUS * EARTH_RADIUS - d * d) / (2 * EARTH_RADIUS * EARTH_RADIUS));
#else
	if (radLat1 < 0)
		radLat1 = M_PI / 2 + fabsf(radLat1);// south  
	if (radLat1 > 0)
		radLat1 = M_PI / 2 - fabsf(radLat1);// north  
	if (radLon1 < 0)
		radLon1 = M_PI * 2 - fabsf(radLon1);// west  
	if (radLat2 < 0)
		radLat2 = M_PI / 2 + fabsf(radLat2);// south  
	if (radLat2 > 0)
		radLat2 = M_PI / 2 - fabsf(radLat2);// north  
	if (radLon2 < 0)
		radLon2 = M_PI * 2 - fabsf(radLon2);// west  

	 x1 = EARTH_RADIUS * cos(radLon1) *sin(radLat1);
	 y1 = EARTH_RADIUS * sin(radLon1) * sin(radLat1);
	 z1 = EARTH_RADIUS * cos(radLat1);

	 x2 = EARTH_RADIUS * cos(radLon2) * sin(radLat2);
	 y2 = EARTH_RADIUS *sin(radLon2) * sin(radLat2);
	 z2 = EARTH_RADIUS * cos(radLat2);
	 d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
	//ÓàÏÒ¶¨ÀíÇó¼Ð½Ç  
	 theta = acos((EARTH_RADIUS * EARTH_RADIUS + EARTH_RADIUS * EARTH_RADIUS - d * d) / (2 * EARTH_RADIUS * EARTH_RADIUS));
#endif


	 dist = theta * EARTH_RADIUS;
	return dist;
}
 

 void ZH_SensorMobile_ACC_Algorithm(short Ax[], short Ay[], short Az[], double Aresult[])
 {

	 double ax = ((Ax[0] << 8) | Ax[1]) / SENSORMODULE_ACCELERATED_FIXED_PARAM;
	 double ay = ((Ay[0] << 8) | Ay[1]) / SENSORMODULE_ACCELERATED_FIXED_PARAM;
	 double az = ((Az[0] << 8) | Az[1]) / SENSORMODULE_ACCELERATED_FIXED_PARAM;
	 Aresult[0] = ax;
	 Aresult[1] = ay;
	 Aresult[2] = az;
 }

 void ZH_SensorMobile_Angle_Speed_Algorithm(short Wx[], short Wy[], short Wz[], double Wresult[])
 {
	 double wx = ((Wx[0] << 8) | Wx[1]) / SENSORMODULE_ANGLES_FIXED_PARAM;
	 double wy = ((Wy[0] << 8) | Wy[1]) / SENSORMODULE_ANGLES_FIXED_PARAM;
	 double wz = ((Wz[0] << 8) | Wz[1]) / SENSORMODULE_ANGLES_FIXED_PARAM;
	 Wresult[0] = wx;
	 Wresult[1] = wy;
	 Wresult[2] = wz;
 }


 void ZH_SensorMobile_Angle_Algorithm(short XRoll[], short YPitch[], short ZYaw[], double Wresult[])
 {
	 double Xr = ((XRoll[0] << 8) | XRoll[1]) / SENSORMODULE_ANGLE_FIXED_PARAM;			//XÖá·­¹ö
	 double Yp = ((YPitch[0] << 8) | YPitch[1]) / SENSORMODULE_ANGLE_FIXED_PARAM;		//YÖá¸©Ñö
	 double Zy = ((ZYaw[0] << 8) | ZYaw[1]) / SENSORMODULE_ANGLE_FIXED_PARAM;			//ZÖáÆ«Àë
	 Wresult[0] = Xr;
	 Wresult[1] = Yp;
	 Wresult[2] = Zy;
 }

