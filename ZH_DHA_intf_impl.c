#include <stdio.h>
#include <math.h>
#include "../inc/ZH_DHA_intf.h"


//////////////////////////////////////////////////////////////////////////
static get_current_miliseconds_callback p_current_miliseconds_callback = NULL;
static GET_VEHICLE_MILEAGE	g_get_vehicle_mileage = NULL;

///////////////////////////ͳ����Ϣ�洢�� Start//////////////////////////////////////////////
STAT_V_SPEED_INFO				g_VehicleSpeedInfo = { 0 };			//������ʻ�ٶ���Ϣ
STAT_BEYOND_SPEED_INFO			g_BeyondSpeedInfo;					//������ʻ����Ϣ
STAT_BRAKING_INFO				g_BrakingInfo;						//ɲ��ʱ����Ϣ
STAT_NEUTRAL_COSTING_INFO		g_NeutralCostingInfo;				//�յ�������Ϣ
STAT_GEARING_RUNNING_INFO		g_GearRunningInfo;					//�ڵ���ʻ��Ϣ
STAT_VEHICLE_STOP_INFO			g_VehicleStopInfo;					//����ֹͣ��Ϣ
STAT_VEHICLE_RUNING_INFO		g_VehicelRunningInfo;				//���������е���Ϣ
STAT_CITY_DRIVING_INFO			g_CityDrivingInfo;
STAT_NO_CITY_DRIVING_INFO		g_NoCityDrivingInfo;				//�ǳ��м�ʻ��Ϣ
//���ִ����� GEARING_RATE_INFO	
STAT_ECO_DRIVING_INFO			g_EcoDrivingInfo;					//���ü�ʻ��Ϣ
STAT_IDLE_INFO					g_IdleInfo;							//������Ϣ
//����Ť��IDLE_TORQUE_INFO				
//�¶���ϢROUTE_DEGREE_INFO
STAT_CLOD_ENGIINE_RAPID_DRIVING_INFO g_ClodEngRapidDrvInfo;			//�����漤�Ҽ�ʻ
STAT_ROAD_DRIVING_INFO			g_RoadDrivingInfo;					//��ʻ�ĵ�·��Ϣ
STAT_ENGINE_TEMPERATURE_INFO	g_EngineTemperature;				//�����¶���Ϣ
STAT_CRUISE_INFO				g_CruiseInfo;						//����Ѳ����Ϣ
STAT_CHANGE_GEAR_INFO			g_ChangeGearInfo;					//������Ϣ
STAT_INSTANT_DECLERATE_INFO		g_InstantDeceleraterInfo;			//��������Ϣ
STAT_INSTANT_ACCELERATE_INFO	g_InstantAccelerateInfo;			//��������Ϣ
STAT_INTANT_THROTTLE_INFO		g_InstantThrottleInfo;				//������
STAT_V_STARTSTOP_COUNTER_INFO	g_VehicleStartStopInfo;				//������ͣ����
STAT_INSTANT_TURN_INFO			g_InstantTurnInfo;					//��ת����Ϣ
STAT_REFUEL_INFO				g_RefuelInfo;						//������Ϣ
STAT_ENGINE_SPEED_INFO			g_EngineSpeedInfo;					//�����ٶ�																	
///////////////////////////ͳ����Ϣ�洢�� End//////////////////////////////////////////////
CUR_VEHICEL_STATE_STR			g_CurVehicle_Can_Info = { 0 };				//�洢��ǰ����״̬��Ϣ
uint32							g_Routing_Time_Counter;						//100ms һ�μ���

long long						g_Routing_Start_Time = 0;					//�г̿�ʼʱ��
long long						g_Routing_End_Time = 0;						//�г̽���ʱ��
uint16							g_Routing_Speed_Threshold = 100;			//��ǰ��·�ٶ���ֵ����ͬ·���ٶ��ǲ�һ����
uint16							g_CityDrving_Speed_threshold = 40;
bool							g_IsCityDrivingSate;						//�Ƿ��ڳ����м�ʻ



void							Count_Vehicle_Speed(vehicle_param_t* pInfo);

/*Ϊ����Ƕȶ����gps�㣬gsp 1Hzһ�Σ�����5�����㹻��
*/
char*			g_pGps_buf = NULL;			//buffer for store GPS ponts
GPS_POINTS*		g_pGpsPoints[6];			//pointer of structure GPS_POINTS
char			g_validPoint = 0;			//current valid points number

/************************************************************************/
/*    ���еĿ�ʼʱ��Ϳ�ʼʱ����̣��ջ��к��ڵ����ж�һ����                                      */
uint32			g_Kong_Slide_Start_Mileage = 0;		//Unit:Km
uint32          g_Kong_Slide_Start_Time = 0;			//Unit:ms
uint32			g_Zai_Slide_Start_Mileage = 0;		//Unit:Km
uint32			g_Zai_Slide_Start_Time = 0;			//Unit:ms
/************************************************************************/
/*
* �㷨���ʼ��
*/

int ZH_DBA_library_init(void)
{
	return 0;
}
/*
* �㷨���˳�
*/
int ZH_DBA_uninit(void)
{
	return 0;
}

/*
* �㷨���ȡ�汾
*/
int ZH_DBA_get_version(void)
{
	return 0;
}



/*
@ �г̿�ʼ
*/
void ZH_DBA_Routing_Start(long long ltm)
{
	g_Routing_Time_Counter = 0;
	g_Routing_Start_Time = ltm;
}

/*
@ �г̽���
*/
void ZH_DBA_Routing_End(long long ltm)
{
	g_Routing_End_Time = ltm;
}

/*
@ ����ʱ��ص�
*/
void ZH_vehicle_set_current_miliseconds_callback(get_current_miliseconds_callback cb)
{
    p_current_miliseconds_callback = cb;
}

/*
@ ���û�ȡ��̻ص�
*/

void ZH_Registe_Get_Mileage_Fun(GET_VEHICLE_MILEAGE cb)
{
	g_get_vehicle_mileage = cb;
}

/*
@��õ�ǰʱ��
*/
 long ZH_vehicle_get_current_miliseconds(void)
{
    if (p_current_miliseconds_callback) {
        return (*p_current_miliseconds_callback)();
    }
    return 0;
}
/*
@ ��õ�ǰ���
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
			case VEHICLE_SPEED:																		// ���ټ���
			{
				/*����ٶ���ƽ���ٶȼ��� ��ʼ*/
				if ((uint8)pVehicle_Inifo->dbl_value > g_VehicleSpeedInfo.Max_Speed)				//���ֵ����
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
				/*����ٶ���ƽ���ٶȼ��� ����*/
				/*���ټ��㿪ʼ*/
				if ((uint8)pVehicle_Inifo->dbl_value >= g_Routing_Speed_Threshold)
				{
					g_BeyondSpeedInfo.Run_Distance = 0;	//��ȡ��ǰʱ������
					g_BeyondSpeedInfo.Run_Time = 0;
					//���ٱ�����ʼ
				}
				else
				{	//���ٽ���
					//���ٱ�������
				}
				/*���ټ������*/
				break;
			}
			case VEHICLE_ENGINE_SPEED:						//�����ٶ�
			{
				//g_CurVehicle_Can_Info.Can_Engine_Rpm = (uint16)pVehicle_Inifo->dbl_value;
				if ((uint32)pVehicle_Inifo->dbl_value > g_EngineSpeedInfo.Engine_Max_Speed)			//���ֵ����
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
				g_CurVehicle_Can_Info.Can_Braking_Pedal = (uint8)pVehicle_Inifo->dbl_value;		//ɲ��̤��ٷֱ�
				if ((uint8)pVehicle_Inifo->dbl_value > 0)
				{	/*ɲ����ʼ*/
					g_BrakingInfo.Braking_Distance = 0;
					g_BrakingInfo.Braking_Time = 0;
					//�ϱ�ɲ���¼���ʼ
				}
				else if ((uint8)pVehicle_Inifo->dbl_value <=0)
				{
					/*ɲ��������ͣ����*/
					//�ϱ�����ɲ���¼�
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
@	�жϳ����г��е���̬����յ����У��ڵ����У�����ͣ����
*/
void ZH_Vehicle_Monitor_State()
{
	if (g_CurVehicle_Can_Info.Can_V_Speed > 5)			//�ٶȲ�Ϊ0
	{		
		if (g_CurVehicle_Can_Info.Can_Gear_Pos == VEHICLE_GEAR_POS_N)
		{
			//�յ����У�������ת��Ϊ���٣�
		
			if (g_Zai_Slide_Start_Mileage != 0)  //�����ڵ�����
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
		else   //������λ
		{//�ڵ�����
			if (g_Kong_Slide_Start_Mileage != 0)  //����յ�����
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
	else    // �ٶ�Ϊ0,������ֹ״̬
	{		
// 		if (g_CurVehicle_Can_Info.Can_Engine_Rpm < 100 )			//ͣ��ʱ���㷨,�г�ʱ��- ���ٶȵ�ʱ����Ǿ�ֹʱ��
// 		{
// 			g_VehicleStopInfo.Stoping_Time = 0;	
// 		}
	}
	//���м�ʻ�㷨������µļ�������GPS + ����Χ��
	//���м�ʻ��־
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

	//���ü�ʻģʽ
	if ((g_CurVehicle_Can_Info.Can_V_Speed > 5) &&	
		(g_CurVehicle_Can_Info.Can_Engine_Rpm < ECO_ENGINE_RPM_MAX) &&
		(g_CurVehicle_Can_Info.Can_Engine_Rpm >= ECO_ENGINE_RPM_MIN)
		)
	{
		/*1,��̴�can��ȡ
		  2����gps �ٶ� X 1�� �ۼ���
		  */
		g_EcoDrivingInfo.Running_Distance = 0;  //
		g_EcoDrivingInfo.Running_times =0;
		g_EcoDrivingInfo.Consume_Fuel = 0;
	}

	//ͣ������ģʽ��ʱ���㷨
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
@GPS �Ƕȼ���
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
// �����㷨
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:ƽ������ (Km/h) 
//		Param2:����� (kph)
//
bool ZH_vehicle_get_speed(DWORD *pSpeed)
{
	pSpeed[0] = g_VehicleSpeedInfo.Average_Speed;
	pSpeed[1] = g_VehicleSpeedInfo.Max_Speed;
	return true;
}
///////////////////////////////////////////////////////////////////////////////////
//
// �����㷨
// input : ����Ԫ�ص�uint32 ����
// output :
//		Param1:������ʻʱ�� (S) 
//		Param2:������ʻ���� (��)
//
bool ZH_vehicle_get_beyond_speed(DWORD *pinfo)
{
	pinfo[0] = g_BeyondSpeedInfo.Run_Time;
	pinfo[1] = g_BeyondSpeedInfo.Run_Distance;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// ɲ���㷨
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:�ܵ�ɲ��ʱ�� (S) 
//		Param2:�ܵ�ɲ������ (��)
//
bool ZH_vehicle_get_braking(DWORD *pinfo)
{
	pinfo[0] = g_BrakingInfo.Braking_Time;
	pinfo[1] = g_BrakingInfo.Braking_Distance;
	return true;
}


///////////////////////////////////////////////////////////////////////////////////
//
//	�յ������㷨
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:�յ�����ʱ�� (S) 
//		Param2:�յ����о��� (��)
//
bool ZH_vehicle_get_neutral_coasting_info(DWORD *pinfo)
{
	pinfo[0] = g_NeutralCostingInfo.Neu_Costing_Time;
	pinfo[1] = g_NeutralCostingInfo.Neu_Costing_Distance;
	return true;
}



///////////////////////////////////////////////////////////////////////////////////
//
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:�ڵ�����ʱ�� (S) 
//		Param2:�ڵ����о��� (��) 
//
bool ZH_vehicle_get_none_neutral_coasting_info(DWORD *pinfo)
{
	pinfo[0] = g_GearRunningInfo.Gearing_Time;
	pinfo[1] = g_GearRunningInfo.Gearing_Distance;
	return true;
}


///////////////////////////////////////////////////////////////////////////////////
//
//ͣ��ʱ���㷨
// input : ����Ԫ�ص�uint32 ����
//
// output : ͣ��ʱ�� (S)
//
//
bool ZH_vehicle_stop_time(DWORD *time)
{
	time[0] = g_VehicleStopInfo.Stoping_Time;
	return true;
}


///////////////////////////////////////////////////////////////////////////////////
//��ʻʱ���㷨
// input : ����Ԫ�ص�uint32 ����
//
// output :��ʻʱ�� (S) 
//
//
bool ZH_vehicle_running_time(DWORD *time)
{
	time[0] = g_VehicelRunningInfo.Runing_Time;
	return true;
}


///////////////////////////////////////////////////////////////////////////////////
//
// ������ʻ�㷨
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:��ʻ���� (��)  
//		Param2:��ʻʱ�� (S)
//		Param3: ȼ������
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
// �ǳ�����ʻ�㷨�����ٹ�·)
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:��ʻ���� (��)  
//		Param2:��ʻʱ�� (S)
//		Param3: ȼ������
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
// ����ת����
// input : ����Ԫ�ص�uint32 ����
//
// output : 
//		������ (�ٷֱ�)
//		
// Note:�ڲ��㷨��Ҫ
double ZH_vehicle_get_gear_rate(unsigned char *v)
{
	double wheel_rpm = g_CurVehicle_Can_Info.Can_V_Speed /8.33 * M_PI * g_CurVehicle_Can_Info.Wheel_Radius_Val;
	double transm_rate = wheel_rpm / g_CurVehicle_Can_Info.Can_Engine_Rpm;
	return transm_rate;
}

///////////////////////////////////////////////////////////////////////////////////
//
// ECO�����ã���ʻ�㷨
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:��ʻ����(��)
//		Param2:ȼ������(��)
//
bool ZH_vehicle_get_ECO_info(DWORD *pinfo)
{
	pinfo[0] = g_EcoDrivingInfo.Running_Distance;
	pinfo[1] = g_EcoDrivingInfo.Consume_Fuel;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//	1.12.�����㷨
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:����ʱ�� (S)
//		Param2:�����ͺ� (��) 
//
bool ZH_vehicle_get_idle_info(DWORD *pinfo)
{
	pinfo[0] = g_IdleInfo.Idle_Time;
	pinfo[1] = g_IdleInfo.Idle_Fuel;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//	����Ť���㷨
// input : ����Ԫ�ص�uint32 ����
//
// output : ������Ť�� (N.m)
//
//
bool ZH_vehicle_get_idle_torque(DWORD *pinfo)
{
	
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//1.14.��·�¶��㷨
// input : ����Ԫ�ص�uint32 ����
//
// output : ��·�¶� (��)
//
//
bool ZH_vehicle_get_route_degree(float *pinfo)
{
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//1.15.�¶ȼ�ʻ�㷨
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:���µ���ʻ���� (��)
//		Param2:���µ���ʻ���� (��)
//		Param3:ƽ·����ʻ���� (��)
bool ZH_vehicle_get_total_driving_distance_info(DWORD *pinfo)
{
	pinfo[0] = g_RoadDrivingInfo.Uphill_Slope;
	pinfo[1] = g_RoadDrivingInfo.Downhill_Slope;
	pinfo[2] = g_RoadDrivingInfo.Flat_Road;

	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:�����漤�Ҽ�ʻʱ�� (S)
//		Param2:�����漤�Ҽ�ʻ���� (��)
//
bool ZH_vehicle_get_cool_engine_driving_info(DWORD *pinfo)
{
	pinfo[0] = g_ClodEngRapidDrvInfo.Instant_Driving_Time;
	pinfo[1] = g_ClodEngRapidDrvInfo.Instant_Driving_Distance;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// �����¶��㷨
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:����ƽ���¶� (��)
//		Param2:��������¶� (��)
//
bool ZH_vehicle_get_engine_temperature_info(float *pinfo)
{
	pinfo[0] = g_EngineTemperature.Engine_Average_Temper;
	pinfo[1] = g_EngineTemperature.Engine_Max_Temper;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// ���㶨��Ѳ��ʱ��;���
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:����Ѳ��ʱ�� (S) 
//		Param2:����Ѳ������ (��)
//
bool ZH_vehicle_get_cruise_info(DWORD *pinfo)
{
	pinfo[0] = g_CruiseInfo.Cruise_Time;
	pinfo[1] = g_CruiseInfo.Cruise_Distance;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// ���㻻������
// ���㵱���淢�����ٵ�ʱ������ʻ��ʱ��;���
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:��������
//		Param2:ƽ��ÿСʱ��������
//
bool ZH_vehicle_get_change_gear_info(DWORD *pinfo)
{

	pinfo[0] = g_ChangeGearInfo.Change_Gear;
	pinfo[1] = g_ChangeGearInfo.Change_Gear_Per_Hour;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//�����Ҽ��ٷ�����ʱ����㳵����ʻ��ʱ��;���
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:������ʱ�� (S)
//		Param2:�����پ��� (��)
//		Param2:�����ٴ���
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
//�����Ҽ��ٷ�����ʱ����㳵����ʻ��ʱ��;���
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:������ʱ�� (S)
//		Param2:�����پ��� (��)
//		Param3:�����ٴ���
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
//����������ʱ���㳵����ʻ��ʱ��;���
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:������ʱ�� (S) 
//		Param2:�����ž��� (��)
//
bool ZH_vehicle_get_intense_throttle_info(DWORD *pinfo)
{
	pinfo[0] = g_InstantThrottleInfo.Instant_Thro_Time;
	pinfo[0] = g_InstantThrottleInfo.Instant_Thro_Distance;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//��¼���⻷���¶�
// input : ����Ԫ�ص�uint32 ����
//
// output : ���⻷���¶ȣ���λ:degC��
//
//
bool ZH_vehicle_get_out_truck_temperature_info(float *pinfo)
{
	 //pinfo[0] = ;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//��¼��������ֹͣ����
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:�������𲽴���
//		Param2:�𲽴���
//
bool ZH_vehicle_get_truck_start_stop_counter(DWORD *pinfo)
{
	pinfo[0] = g_VehicleStartStopInfo.Cold_Engine_Start_Times;
	pinfo[1] = g_VehicleStartStopInfo.Total_Start_Times;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
//��¼�����ļ�ת�����
// input : ����Ԫ�ص�uint32 ����
//
// output : ��ת�����
//
//
bool ZH_vehicle_get_intense_turning_info(DWORD *pinfo)
{
	pinfo[0] = g_InstantTurnInfo.Instant_Turn_Tims;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// �������������͵�ʱ���¼�����ļ����¼�
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:��ʼ���Ͱٷֱ�
//		Param2:�������Ͱٷֱ�
//		Param3:������ (��)
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
//��¼�����ٶȵ�ƽ��ֵ�����ֵ
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:����ת��ƽ��ֵ
//		Param2:����ת�����ֵ
//
bool ZH_vehicle_get_engine_speed_info(DWORD *pinfo)
{
	pinfo[0] = g_EngineSpeedInfo.Engine_Average_Speed;
	pinfo[1] = g_EngineSpeedInfo.Engine_Max_Speed;
	return true;
}
////////////////////////////////////////////////////////////////////////////////////





/*����ʵʱ�����¼��ص�����*/
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
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:������ʻʱ�� (S) 
//		Param2:������ʻ���� (��)
//
bool ZH_vehicle_get_beyond_speed(DWORD *pinfo)
{
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:������ʻʱ�� (S) 
//		Param2:������ʻ���� (��)
//
bool ZH_vehicle_get_beyond_speed(DWORD *pinfo)
{
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:������ʻʱ�� (S) 
//		Param2:������ʻ���� (��)
//
bool ZH_vehicle_get_beyond_speed(DWORD *pinfo)
{
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:������ʻʱ�� (S) 
//		Param2:������ʻ���� (��)
//
bool ZH_vehicle_get_beyond_speed(DWORD *pinfo)
{
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
//
// input : ����Ԫ�ص�uint32 ����
//
// output :
//		Param1:������ʻʱ�� (S) 
//		Param2:������ʻ���� (��)
//
bool ZH_vehicle_get_beyond_speed(DWORD *pinfo)
{
	return true;
}
#endif