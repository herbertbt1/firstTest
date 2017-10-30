#include <stdio.h>
#include <math.h>
#include "../inc/ZH_DHA_intf.h"


//////////////////////////////////////////////////////////////////////////
static get_current_miliseconds_callback p_current_miliseconds_callback = NULL;
static GET_VEHICLE_MILEAGE	g_get_vehicle_mileage = NULL;

///////////////////////////统计信息存储区 Start//////////////////////////////////////////////
STAT_V_SPEED_INFO				g_VehicleSpeedInfo = { 0 };			//车辆行驶速度信息
STAT_BEYOND_SPEED_INFO			g_BeyondSpeedInfo;					//超速行驶的信息
STAT_BRAKING_INFO				g_BrakingInfo;						//刹车时的信息
STAT_NEUTRAL_COSTING_INFO		g_NeutralCostingInfo;				//空挡滑行信息
STAT_GEARING_RUNNING_INFO		g_GearRunningInfo;					//在档行驶信息
STAT_VEHICLE_STOP_INFO			g_VehicleStopInfo;					//车辆停止信息
STAT_VEHICLE_RUNING_INFO		g_VehicelRunningInfo;				//车辆运行中的信息
STAT_CITY_DRIVING_INFO			g_CityDrivingInfo;
STAT_NO_CITY_DRIVING_INFO		g_NoCityDrivingInfo;				//非城市驾驶信息
//齿轮传动比 GEARING_RATE_INFO	
STAT_ECO_DRIVING_INFO			g_EcoDrivingInfo;					//经济驾驶信息
STAT_IDLE_INFO					g_IdleInfo;							//怠速信息
//怠速扭矩IDLE_TORQUE_INFO				
//坡度信息ROUTE_DEGREE_INFO
STAT_CLOD_ENGIINE_RAPID_DRIVING_INFO g_ClodEngRapidDrvInfo;			//冷引擎激烈驾驶
STAT_ROAD_DRIVING_INFO			g_RoadDrivingInfo;					//行驶的道路信息
STAT_ENGINE_TEMPERATURE_INFO	g_EngineTemperature;				//引擎温度信息
STAT_CRUISE_INFO				g_CruiseInfo;						//定速巡航信息
STAT_CHANGE_GEAR_INFO			g_ChangeGearInfo;					//换挡信息
STAT_INSTANT_DECLERATE_INFO		g_InstantDeceleraterInfo;			//急减速信息
STAT_INSTANT_ACCELERATE_INFO	g_InstantAccelerateInfo;			//急加速信息
STAT_INTANT_THROTTLE_INFO		g_InstantThrottleInfo;				//急油门
STAT_V_STARTSTOP_COUNTER_INFO	g_VehicleStartStopInfo;				//车辆启停急速
STAT_INSTANT_TURN_INFO			g_InstantTurnInfo;					//急转弯信息
STAT_REFUEL_INFO				g_RefuelInfo;						//加油信息
STAT_ENGINE_SPEED_INFO			g_EngineSpeedInfo;					//引擎速度																	
///////////////////////////统计信息存储区 End//////////////////////////////////////////////
CUR_VEHICEL_STATE_STR			g_CurVehicle_Can_Info = { 0 };				//存储当前车辆状态信息
uint32							g_Routing_Time_Counter;						//100ms 一次计数

long long						g_Routing_Start_Time = 0;					//行程开始时间
long long						g_Routing_End_Time = 0;						//行程结束时间
uint16							g_Routing_Speed_Threshold = 100;			//当前道路速度阈值，不同路段速度是不一样的
uint16							g_CityDrving_Speed_threshold = 40;
bool							g_IsCityDrivingSate;						//是否在城市中驾驶



void							Count_Vehicle_Speed(vehicle_param_t* pInfo);

/*为计算角度定义的gps点，gsp 1Hz一次，保存5个就足够了
*/
char*			g_pGps_buf = NULL;			//buffer for store GPS ponts
GPS_POINTS*		g_pGpsPoints[6];			//pointer of structure GPS_POINTS
char			g_validPoint = 0;			//current valid points number

/************************************************************************/
/*    滑行的开始时间和开始时的里程（空滑行和在档滑行都一样）                                      */
uint32			g_Kong_Slide_Start_Mileage = 0;		//Unit:Km
uint32          g_Kong_Slide_Start_Time = 0;			//Unit:ms
uint32			g_Zai_Slide_Start_Mileage = 0;		//Unit:Km
uint32			g_Zai_Slide_Start_Time = 0;			//Unit:ms
/************************************************************************/
/*
* 算法库初始化
*/

int ZH_DBA_library_init(void)
{
	return 0;
}
/*
* 算法库退出
*/
int ZH_DBA_uninit(void)
{
	return 0;
}

/*
* 算法库获取版本
*/
int ZH_DBA_get_version(void)
{
	return 0;
}



/*
@ 行程开始
*/
void ZH_DBA_Routing_Start(long long ltm)
{
	g_Routing_Time_Counter = 0;
	g_Routing_Start_Time = ltm;
}

/*
@ 行程结束
*/
void ZH_DBA_Routing_End(long long ltm)
{
	g_Routing_End_Time = ltm;
}

/*
@ 设置时间回调
*/
void ZH_vehicle_set_current_miliseconds_callback(get_current_miliseconds_callback cb)
{
    p_current_miliseconds_callback = cb;
}

/*
@ 设置获取里程回调
*/

void ZH_Registe_Get_Mileage_Fun(GET_VEHICLE_MILEAGE cb)
{
	g_get_vehicle_mileage = cb;
}

/*
@获得当前时间
*/
 long ZH_vehicle_get_current_miliseconds(void)
{
    if (p_current_miliseconds_callback) {
        return (*p_current_miliseconds_callback)();
    }
    return 0;
}
/*
@ 获得当前里程
*/
 long ZH_Get_Current_Mileage()
{
	if (g_get_vehicle_mileage == NULL)
		return -1;
	return (*g_get_vehicle_mileage)();
}
int ZH_vehicle_set_algorithm_limits(vehicle_param_t *params, int count)
{
    return 0;
}

int ZH_vehicle_put_vehicle_data(vehicle_param_t *params, int count)
{
	int i = 0;
    if (!params || count <= 0) {
        return -1;
    }
	while (count--)
	{
		vehicle_param_t* pVehicle_Inifo = &(params[i++]);
		if (pVehicle_Inifo->valid == 1)
		{
			switch (pVehicle_Inifo->type)
			{
			case VEHICLE_SPEED:																		// 车速计算
			{
				/*最大速度与平均速度计算 开始*/
				if ((uint8)pVehicle_Inifo->dbl_value > g_VehicleSpeedInfo.Max_Speed)				//最大值交换
					g_VehicleSpeedInfo.Max_Speed = (uint8)pVehicle_Inifo->dbl_value;
				//Count_Vehicle_Speed(pVehicle_Inifo);
				
				g_VehicleSpeedInfo.Total_Speed += (uint8)pVehicle_Inifo->dbl_value;
				g_VehicleSpeedInfo.Total_Counter++;

				//count
				if (g_VehicleSpeedInfo.Total_Counter > MAX_SPEEDIINFO_NUM_COUNTER)
				{
					g_VehicleSpeedInfo.Average_Speed = (g_VehicleSpeedInfo.Total_Speed + g_VehicleSpeedInfo.Average_Speed) / (g_VehicleSpeedInfo.Total_Counter + 1);
					g_VehicleSpeedInfo.Total_Speed = 0;
					g_VehicleSpeedInfo.Total_Counter = 0;
				}
				/*最大速度与平均速度计算 结束*/
				/*超速计算开始*/
				if ((uint8)pVehicle_Inifo->dbl_value >= g_Routing_Speed_Threshold)
				{
					g_BeyondSpeedInfo.Run_Distance = 0;	//获取当前时间和里程
					g_BeyondSpeedInfo.Run_Time = 0;
					//超速报警开始
				}
				else
				{	//超速结束
					//超速报警结束
				}
				/*超速计算结束*/
				break;
			}
			case VEHICLE_ENGINE_SPEED:						//引擎速度
			{
				//g_CurVehicle_Can_Info.Can_Engine_Rpm = (uint16)pVehicle_Inifo->dbl_value;
				if ((uint32)pVehicle_Inifo->dbl_value > g_EngineSpeedInfo.Engine_Max_Speed)			//最大值交换
					g_EngineSpeedInfo.Engine_Max_Speed = (uint32)pVehicle_Inifo->dbl_value;
				//Count_Vehicle_Speed(pVehicle_Inifo);
				g_EngineSpeedInfo.Total_Eng_RPM += (uint8)pVehicle_Inifo->dbl_value;
				g_EngineSpeedInfo.Total_Counter++;
				//count
				if (g_EngineSpeedInfo.Total_Counter > MAX_SPEEDIINFO_NUM_COUNTER)
				{
					g_EngineSpeedInfo.Engine_Average_Speed= (g_EngineSpeedInfo.Total_Eng_RPM + g_EngineSpeedInfo.Engine_Average_Speed) / (g_EngineSpeedInfo.Total_Counter + 1);
					g_EngineSpeedInfo.Total_Eng_RPM = 0;
					g_EngineSpeedInfo.Total_Counter = 0;
				}


			}
				break;
			case VEHICLE_GEAR_POS: 
			{
				g_CurVehicle_Can_Info.Can_Gear_Pos = (uint8)pVehicle_Inifo->dbl_value;
				break;
			}
				
			case VEHICLE_BRAKE_PEDAL:
			{
				g_CurVehicle_Can_Info.Can_Braking_Pedal = (uint8)pVehicle_Inifo->dbl_value;		//刹车踏板百分比
				if ((uint8)pVehicle_Inifo->dbl_value > 0)
				{	/*刹车开始*/
					g_BrakingInfo.Braking_Distance = 0;
					g_BrakingInfo.Braking_Time = 0;
					//上报刹车事件开始
				}
				else if ((uint8)pVehicle_Inifo->dbl_value <=0)
				{
					/*刹车结束暂停计数*/
					//上报结束刹车事件
				}

				break;
			}
				
			case VEHICLE_ACCELERATOR_PEDAL:
			{
				g_CurVehicle_Can_Info.Can_Throttle_Pedal = (uint8)pVehicle_Inifo->dbl_value;
				break;
			}
				
			case VEHICLE_CLUTCH_PEDAL:
			{
				g_CurVehicle_Can_Info.Can_Clutch_Pedal = (uint8)pVehicle_Inifo->dbl_value;
				break;
			}
				
			case VEHICLE_CRUISE_STATUS:
			{
				g_CurVehicle_Can_Info.Can_Cruise_State = (uint8)pVehicle_Inifo->dbl_value;
				break;
			}							
			case VEHICLE_ENGINE_TEMPRATURE:
			{
				g_CurVehicle_Can_Info.Can_Engine_Temperature = (uint32)pVehicle_Inifo->dbl_value;
				break;
			}
				
			case VEHICLE_REMAINING_FUEL:
			{
				g_CurVehicle_Can_Info.Can_Remain_Fuel = (uint16)pVehicle_Inifo->dbl_value;
				break;
			}
				
			case VEHICLE_GPS_LONGITUDE:
			{
				g_CurVehicle_Can_Info.Gps_Longitude_Val = (uint32)pVehicle_Inifo->dbl_value;
				break;
			}
				
			case VEHICLE_GPS_LATITUDE:
			{
				g_CurVehicle_Can_Info.Gps_Latitude_Val = (uint32)pVehicle_Inifo->dbl_value;
				break;
			}
				
			case VEHICLE_GPS_HEIGHT:
			{
				g_CurVehicle_Can_Info.Gps_Height_Val = (uint16)pVehicle_Inifo->dbl_value;
				break;
			}
				
			case VEHICLE_WHEEL_RADIUS:
			{
				g_CurVehicle_Can_Info.Wheel_Radius_Val = (uint16)pVehicle_Inifo->dbl_value;
				break;
			}
				
			case VEHICLE_TURNING_RADIUS:
			{
				g_CurVehicle_Can_Info.Vehcile_Turn_Radius_Val = (uint32)pVehicle_Inifo->dbl_value;
				break;
			}
				
			case VEHICLE_ENGINE_TORQUE_IND:
			{
				g_CurVehicle_Can_Info.Engine_Torque_Val = (uint16)pVehicle_Inifo->dbl_value;
				break;

			}
			case ROUTING_SPEED_THRESHOLD:
			{
				g_Routing_Speed_Threshold = (uint16)pVehicle_Inifo->dbl_value;
				break;
			}
			case CITY_DRVING_SPEED_THRESHOLD:
			{
				g_CityDrving_Speed_threshold = (uint16)pVehicle_Inifo->dbl_value;
				break;
			}
				
				
			default:
				break;
			}
		}
	}
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////

/*
@	判断车辆行程中的姿态，如空挡滑行，在档滑行，怠速停车等
*/
void ZH_Vehicle_Monitor_State()
{
	if (g_CurVehicle_Can_Info.Can_V_Speed > 5)			//速度不为0
	{		
		if (g_CurVehicle_Can_Info.Can_Gear_Pos == VEHICLE_GEAR_POS_N)
		{
			//空挡滑行，发动机转速为怠速，
		
			if (g_Zai_Slide_Start_Mileage != 0)  //处理在挡滑行
			{
				g_GearRunningInfo.Gearing_Distance += ZH_Get_Current_Mileage() - g_Zai_Slide_Start_Mileage;
				g_GearRunningInfo.Gearing_Time += ZH_vehicle_get_current_miliseconds() - g_Zai_Slide_Start_Time;

				g_Zai_Slide_Start_Mileage = 0;
				g_Zai_Slide_Start_Time = 0;
			}

			//recode start time and mileage			
			g_Kong_Slide_Start_Mileage = ZH_Get_Current_Mileage();
			g_Kong_Slide_Start_Time = ZH_vehicle_get_current_miliseconds();
		}
		else   //其他档位
		{//在档滑行
			if (g_Kong_Slide_Start_Mileage != 0)  //处理空挡滑行
			{
				g_NeutralCostingInfo.Neu_Costing_Distance += ZH_Get_Current_Mileage() - g_Kong_Slide_Start_Mileage;
				g_NeutralCostingInfo.Neu_Costing_Time += ZH_vehicle_get_current_miliseconds() - g_Kong_Slide_Start_Time;

				g_Kong_Slide_Start_Mileage = 0;
				g_Kong_Slide_Start_Time = 0;
			}
			
		
			//recode start time and mileage			
			g_Zai_Slide_Start_Mileage = ZH_Get_Current_Mileage();
			g_Zai_Slide_Start_Time = ZH_vehicle_get_current_miliseconds();

		}
	}
	else    // 速度为0,车辆静止状态
	{		
// 		if (g_CurVehicle_Can_Info.Can_Engine_Rpm < 100 )			//停车时间算法,行程时间- 有速度的时间就是静止时间
// 		{
// 			g_VehicleStopInfo.Stoping_Time = 0;	
// 		}
	}
	//城市驾驶算法，结合新的技术采用GPS + 电子围栏
	//城市驾驶标志
	if(g_IsCityDrivingSate == true)
	{
		g_CityDrivingInfo.City_Driving_Distance = 0;
		g_CityDrivingInfo.City_Driving_Fule = 0;
		g_CityDrivingInfo.City_Driving_Time = 0;
	}
	else
	{
		g_NoCityDrivingInfo.No_City_Driving_Distance = 0;
		g_NoCityDrivingInfo.No_City_Driving_Fule = 0;
		g_NoCityDrivingInfo.No_City_Driving_Time = 0;
	}

	//经济驾驶模式
	if ((g_CurVehicle_Can_Info.Can_V_Speed > 5) &&	
		(g_CurVehicle_Can_Info.Can_Engine_Rpm < ECO_ENGINE_RPM_MAX) &&
		(g_CurVehicle_Can_Info.Can_Engine_Rpm >= ECO_ENGINE_RPM_MIN)
		)
	{
		/*1,里程从can上取
		  2，从gps 速度 X 1秒 累加来
		  */
		g_EcoDrivingInfo.Running_Distance = 0;  //
		g_EcoDrivingInfo.Running_times =0;
		g_EcoDrivingInfo.Consume_Fuel = 0;
	}

	//停车怠速模式的时间算法
	if ((g_CurVehicle_Can_Info.Can_Throttle_Pedal <= 1) &&
		(g_CurVehicle_Can_Info.Can_V_Speed <= 1) &&
		(g_CurVehicle_Can_Info.Can_Engine_Rpm > 500)
		)
	{
		g_IdleInfo.Idle_Time = 0;
		g_IdleInfo.Idle_Fuel = 0;
	}
}

/**********************************************************************
//
//
//
@GPS 角度计算
//
//
//
***********************************************************************/
double  ZH_Ramp_Angle(double Longti,double Lati,double Height)
{
	double dAngle = 0;
	int i;
	if (g_pGps_buf == NULL)
	{
		g_pGps_buf = (char*)malloc(sizeof(GPS_POINTS) * 6);
		for ( i = 0; i < 6; i++)
		{
			g_pGpsPoints[i] = (GPS_POINTS*)(g_pGps_buf + sizeof(GPS_POINTS) * i);

		}
	}
	g_pGpsPoints[4]->Height = Height;
	g_pGpsPoints[4]->Latitude = Lati;
	g_pGpsPoints[4]->Longitude = Longti;

	g_validPoint++;
	if (g_validPoint >= 5)
	{
		//count horizontal line distance
		double distance = LantitudeLongitudeDist(g_pGpsPoints[0]->Longitude, g_pGpsPoints[0]->Latitude, g_pGpsPoints[4]->Longitude, g_pGpsPoints[4]->Latitude);
		//relative height
		double relateHight = abs(g_pGpsPoints[0]->Height - g_pGpsPoints[4]->Height);
		double dAngle = atanl(relateHight / (abs(distance) * 1000))*ANGLE_PARAM;

		printf("distance:%f,hight:%f,angle:%f\r\n", distance * 1000, relateHight, dAngle);
		g_validPoint -= 1;
	}

	memmove(g_pGps_buf, g_pGps_buf + sizeof(GPS_POINTS), sizeof (GPS_POINTS) * 4);
	return dAngle;
}
///////////////////////////////////////////////////////////////////////////////////
//	
// 车速算法
// input : 两个元素的uint32 数组
//
// output :
//		Param1:平均车速 (Km/h) 
//		Param2:最大车速 (kph)
//
bool ZH_vehicle_get_speed(DWORD *pSpeed)
{
	pSpeed[0] = g_VehicleSpeedInfo.Average_Speed;
	pSpeed[1] = g_VehicleSpeedInfo.Max_Speed;
	return true;
}
///////////////////////////////////////////////////////////////////////////////////
//
// 超速算法
// input : 两个元素的uint32 数组
// output :
//		Param1:超速行驶时间 (S) 
//		Param2:超速行驶距离 (米)
//
bool ZH_vehicle_get_beyond_speed(DWORD *pinfo)
{
	pinfo[0] = g_BeyondSpeedInfo.Run_Time;
	pinfo[1] = g_BeyondSpeedInfo.Run_Distance;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// 刹车算法
// input : 两个元素的uint32 数组
//
// output :
//		Param1:总的刹车时间 (S) 
//		Param2:总的刹车距离 (米)
//
bool ZH_vehicle_get_braking(DWORD *pinfo)
{
	pinfo[0] = g_BrakingInfo.Braking_Time;
	pinfo[1] = g_BrakingInfo.Braking_Distance;
	return true;
}


///////////////////////////////////////////////////////////////////////////////////
//
//	空挡滑行算法
// input : 两个元素的uint32 数组
//
// output :
//		Param1:空挡滑行时间 (S) 
//		Param2:空挡滑行距离 (米)
//
bool ZH_vehicle_get_neutral_coasting_info(DWORD *pinfo)
{
	pinfo[0] = g_NeutralCostingInfo.Neu_Costing_Time;
	pinfo[1] = g_NeutralCostingInfo.Neu_Costing_Distance;
	return true;
}



///////////////////////////////////////////////////////////////////////////////////
//
// input : 两个元素的uint32 数组
//
// output :
//		Param1:在档滑行时间 (S) 
//		Param2:在档滑行距离 (米) 
//
bool ZH_vehicle_get_none_neutral_coasting_info(DWORD *pinfo)
{
	pinfo[0] = g_GearRunningInfo.Gearing_Time;
	pinfo[1] = g_GearRunningInfo.Gearing_Distance;
	return true;
}


///////////////////////////////////////////////////////////////////////////////////
//
//停车时间算法
// input : 两个元素的uint32 数组
//
// output : 停车时间 (S)
//
//
bool ZH_vehicle_stop_time(DWORD *time)
{
	time[0] = g_VehicleStopInfo.Stoping_Time;
	return true;
}


///////////////////////////////////////////////////////////////////////////////////
//行驶时间算法
// input : 两个元素的uint32 数组
//
// output :行驶时间 (S) 
//
//
bool ZH_vehicle_running_time(DWORD *time)
{
	time[0] = g_VehicelRunningInfo.Runing_Time;
	return true;
}


///////////////////////////////////////////////////////////////////////////////////
//
// 城市行驶算法
// input : 两个元素的uint32 数组
//
// output :
//		Param1:行驶距离 (米)  
//		Param2:行驶时间 (S)
//		Param3: 燃油消耗
//
bool ZH_vehicle_get_city_driving_info(DWORD *pinfo)
{
	pinfo[0] = g_CityDrivingInfo.City_Driving_Distance;
	pinfo[1] = g_CityDrivingInfo.City_Driving_Time;
	pinfo[2] = g_CityDrivingInfo.City_Driving_Fule;

	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// 非城市行驶算法（高速公路)
// input : 两个元素的uint32 数组
//
// output :
//		Param1:行驶距离 (米)  
//		Param2:行驶时间 (S)
//		Param3: 燃油消耗
//
bool ZH_vehicle_get_none_city_driving_info(DWORD *pinfo)
{
	pinfo[0] = g_NoCityDrivingInfo.No_City_Driving_Distance;
	pinfo[1] = g_NoCityDrivingInfo.No_City_Driving_Time;
	pinfo[2] = g_NoCityDrivingInfo.No_City_Driving_Fule;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// 齿轮转动比
// input : 两个元素的uint32 数组
//
// output : 
//		传动比 (百分比)
//		
// Note:内部算法需要
double ZH_vehicle_get_gear_rate(unsigned char *v)
{
	double wheel_rpm = g_CurVehicle_Can_Info.Can_V_Speed /8.33 * M_PI * g_CurVehicle_Can_Info.Wheel_Radius_Val;
	double transm_rate = wheel_rpm / g_CurVehicle_Can_Info.Can_Engine_Rpm;
	return transm_rate;
}

///////////////////////////////////////////////////////////////////////////////////
//
// ECO（经济）驾驶算法
// input : 两个元素的uint32 数组
//
// output :
//		Param1:行驶距离(米)
//		Param2:燃油消耗(升)
//
bool ZH_vehicle_get_ECO_info(DWORD *pinfo)
{
	pinfo[0] = g_EcoDrivingInfo.Running_Distance;
	pinfo[1] = g_EcoDrivingInfo.Consume_Fuel;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//	1.12.怠速算法
// input : 两个元素的uint32 数组
//
// output :
//		Param1:怠速时间 (S)
//		Param2:怠速油耗 (升) 
//
bool ZH_vehicle_get_idle_info(DWORD *pinfo)
{
	pinfo[0] = g_IdleInfo.Idle_Time;
	pinfo[1] = g_IdleInfo.Idle_Fuel;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//	怠速扭矩算法
// input : 两个元素的uint32 数组
//
// output : 均怠速扭矩 (N.m)
//
//
bool ZH_vehicle_get_idle_torque(DWORD *pinfo)
{
	
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//1.14.道路坡度算法
// input : 两个元素的uint32 数组
//
// output : 道路坡度 (度)
//
//
bool ZH_vehicle_get_route_degree(float *pinfo)
{
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//1.15.坡度驾驶算法
// input : 两个元素的uint32 数组
//
// output :
//		Param1:上坡的行驶距离 (米)
//		Param2:下坡的行驶距离 (米)
//		Param3:平路的行驶距离 (米)
bool ZH_vehicle_get_total_driving_distance_info(DWORD *pinfo)
{
	pinfo[0] = g_RoadDrivingInfo.Uphill_Slope;
	pinfo[1] = g_RoadDrivingInfo.Downhill_Slope;
	pinfo[2] = g_RoadDrivingInfo.Flat_Road;

	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// input : 两个元素的uint32 数组
//
// output :
//		Param1:冷引擎激烈驾驶时间 (S)
//		Param2:冷引擎激烈驾驶距离 (米)
//
bool ZH_vehicle_get_cool_engine_driving_info(DWORD *pinfo)
{
	pinfo[0] = g_ClodEngRapidDrvInfo.Instant_Driving_Time;
	pinfo[1] = g_ClodEngRapidDrvInfo.Instant_Driving_Distance;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// 引擎温度算法
// input : 两个元素的uint32 数组
//
// output :
//		Param1:引擎平均温度 (度)
//		Param2:引擎最高温度 (度)
//
bool ZH_vehicle_get_engine_temperature_info(float *pinfo)
{
	pinfo[0] = g_EngineTemperature.Engine_Average_Temper;
	pinfo[1] = g_EngineTemperature.Engine_Max_Temper;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// 计算定速巡航时间和距离
// input : 两个元素的uint32 数组
//
// output :
//		Param1:定速巡航时间 (S) 
//		Param2:定速巡航距离 (米)
//
bool ZH_vehicle_get_cruise_info(DWORD *pinfo)
{
	pinfo[0] = g_CruiseInfo.Cruise_Time;
	pinfo[1] = g_CruiseInfo.Cruise_Distance;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// 计算换挡次数
// 计算当引擎发生超速的时候车辆行驶的时间和距离
// input : 两个元素的uint32 数组
//
// output :
//		Param1:换挡次数
//		Param2:平均每小时换挡次数
//
bool ZH_vehicle_get_change_gear_info(DWORD *pinfo)
{

	pinfo[0] = g_ChangeGearInfo.Change_Gear;
	pinfo[1] = g_ChangeGearInfo.Change_Gear_Per_Hour;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//当激烈减速发生的时候计算车辆行驶的时间和距离
// input : 两个元素的uint32 数组
//
// output :
//		Param1:急减速时间 (S)
//		Param2:急减速距离 (米)
//		Param2:急减速次数
//
bool ZH_vehicle_get_intenseslow_info(DWORD *pinfo)
{

	pinfo[0] = g_InstantDeceleraterInfo.Decelerate_Time;
	pinfo[1] = g_InstantDeceleraterInfo.Decelerate_Distance;
	pinfo[2] = g_InstantDeceleraterInfo.Decelerate_Times;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//当激烈加速发生的时候计算车辆行驶的时间和距离
// input : 两个元素的uint32 数组
//
// output :
//		Param1:急加速时间 (S)
//		Param2:急加速距离 (米)
//		Param3:急加速次数
//
bool ZH_vehicle_get_intense_acc_info(DWORD *pinfo)
{
	pinfo[0] = g_InstantAccelerateInfo.Accelerate_Time;
	pinfo[1] = g_InstantAccelerateInfo.Accelerate_Distance;
	pinfo[2] = g_InstantAccelerateInfo.Accelerate_Times;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//当激踩油门时计算车辆行驶的时间和距离
// input : 两个元素的uint32 数组
//
// output :
//		Param1:急油门时间 (S) 
//		Param2:急油门距离 (米)
//
bool ZH_vehicle_get_intense_throttle_info(DWORD *pinfo)
{
	pinfo[0] = g_InstantThrottleInfo.Instant_Thro_Time;
	pinfo[0] = g_InstantThrottleInfo.Instant_Thro_Distance;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//记录车外环境温度
// input : 两个元素的uint32 数组
//
// output : 车外环境温度（单位:degC）
//
//
bool ZH_vehicle_get_out_truck_temperature_info(float *pinfo)
{
	 //pinfo[0] = ;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//记录车辆启动停止次数
// input : 两个元素的uint32 数组
//
// output :
//		Param1:冷启动起步次数
//		Param2:起步次数
//
bool ZH_vehicle_get_truck_start_stop_counter(DWORD *pinfo)
{
	pinfo[0] = g_VehicleStartStopInfo.Cold_Engine_Start_Times;
	pinfo[1] = g_VehicleStartStopInfo.Total_Start_Times;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//记录车辆的急转向次数
// input : 两个元素的uint32 数组
//
// output : 急转向次数
//
//
bool ZH_vehicle_get_intense_turning_info(DWORD *pinfo)
{
	pinfo[0] = g_InstantTurnInfo.Instant_Turn_Tims;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// 当车辆发生加油的时候记录车辆的加油事件
// input : 两个元素的uint32 数组
//
// output :
//		Param1:开始加油百分比
//		Param2:结束加油百分比
//		Param3:加油量 (升)
//
bool ZH_vehicle_get_refule_info(DWORD *pinfo)
{
	pinfo[0] = g_RefuelInfo.Start_Refuel_Percent;
	pinfo[1] = g_RefuelInfo.Stop_Refuel_Percent;
	pinfo[2] = g_RefuelInfo.Refuel_Litre;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//记录引擎速度的平均值，最大值
// input : 两个元素的uint32 数组
//
// output :
//		Param1:引擎转速平均值
//		Param2:引擎转速最大值
//
bool ZH_vehicle_get_engine_speed_info(DWORD *pinfo)
{
	pinfo[0] = g_EngineSpeedInfo.Engine_Average_Speed;
	pinfo[1] = g_EngineSpeedInfo.Engine_Max_Speed;
	return true;
}
////////////////////////////////////////////////////////////////////////////////////





/*设置实时报警事件回调函数*/
void ZH_vehicle_set_realtime_alarm_event_callback(ZH_vehicle_realtime_alarm_event_callback cb)
{

}



//void Count_Vehicle_Speed(vehicle_param_t* pInfo)
//{
//	g_VehicleSpeedInfo.Total_Speed += g_CurVehicle_Can_Info.Can_V_Speed;
//	g_VehicleSpeedInfo.Total_Counter++;
//	if (g_VehicleSpeedInfo.Total_Counter > MAX_SPEEDIINFO_NUM_COUNTER)
//	{
//		g_VehicleSpeedInfo.Average_Speed = (g_VehicleSpeedInfo.Total_Speed + g_VehicleSpeedInfo.Average_Speed) / (g_VehicleSpeedInfo.Total_Counter + 1);
//		g_VehicleSpeedInfo.Total_Speed = 0;
//		g_VehicleSpeedInfo.Total_Counter = 0;
//	}
//
//	//g_VehicleSpeedInfo.Max_Speed = (g_CurVehicle_Can_Info.Can_V_Speed > g_VehicleSpeedInfo.Max_Speed) ? g_CurVehicle_Can_Info.Can_V_Speed : g_VehicleSpeedInfo.Max_Speed;
//}
#if 0
///////////////////////////////////////////////////////////////////////////////////
//
// input : 两个元素的uint32 数组
//
// output :
//		Param1:超速行驶时间 (S) 
//		Param2:超速行驶距离 (米)
//
bool ZH_vehicle_get_beyond_speed(DWORD *pinfo)
{
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// input : 两个元素的uint32 数组
//
// output :
//		Param1:超速行驶时间 (S) 
//		Param2:超速行驶距离 (米)
//
bool ZH_vehicle_get_beyond_speed(DWORD *pinfo)
{
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// input : 两个元素的uint32 数组
//
// output :
//		Param1:超速行驶时间 (S) 
//		Param2:超速行驶距离 (米)
//
bool ZH_vehicle_get_beyond_speed(DWORD *pinfo)
{
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// input : 两个元素的uint32 数组
//
// output :
//		Param1:超速行驶时间 (S) 
//		Param2:超速行驶距离 (米)
//
bool ZH_vehicle_get_beyond_speed(DWORD *pinfo)
{
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// input : 两个元素的uint32 数组
//
// output :
//		Param1:超速行驶时间 (S) 
//		Param2:超速行驶距离 (米)
//
bool ZH_vehicle_get_beyond_speed(DWORD *pinfo)
{
	return true;
}
#endif