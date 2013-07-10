#ifdef _MAIN_
	#ifndef _GLOBAL_
		#define _GLOBAL_
	#endif
#else
	#ifndef _GLOBAL_
		#define _GLOBAL_	extern
	#endif
#endif	


#include	"commonDef.h"



#define TIME 0.053

typedef struct R_state
{
	double x;
	double y;
	double th;

	double com_th;
	double pre_com_th;
		
	double v;
	double w;

	double D;
	double W;

	double v_left;
	double v_right;

	double target_v_left;
	double target_v_right;

	double temp_v_left;
	double temp_v_right;
	double MR_P, MR_I, MR_D, MR_pre_P;
	double ML_P, ML_I, ML_D,ML_pre_P;

	double v_target_v, v_target_w;

	double target_v, target_w;
	double target_v_E, target_v_S;
	double target_w_E, target_w_S;

	double laser_data[690];

	

	int vfh_mode;
	int Run_mode;

}t_ROBOT_STATE;

typedef struct state
{
	double Goal_Distance;
	double Goal_B_angle;
	double Virtual_Goal_B_angle;
	
	
	double Distance;
	double Bearing_Ang;
	double Beta;

	double Ex,Ey;

	double Laser_data[3];


	double x,y,th;

	double Dis_P, Dis_pre_P, Dis_Integ, Dis_D;
	double Bea_P, Bea_pre_P, Bea_Integ, Bea_D;

	//double Cr=0;
	//double CR=0;
	//double al=1;

	double VFH_max_speed;
	double VFH_target_pos[3];

} t_RELATIVE_STATE;


typedef struct meas
{
	double d;
	double b;
	double d_sound;
	double b_sound;
	double beta;
	double b_laser;
	double b_vision;

	double laserData[2];
	
}t_MEASUREMENT;

typedef struct matrix
{
	double	jF[3][3];
	double	jD[2][3];
		
}t_MATRIX;


typedef double(*t_Matrix2)[2];
typedef double(*t_Matrix3)[3];
typedef double(*t_Matrix4)[4];

/****** Robot Structures From Player Client Library***********************/
/*
_GLOBAL_ playerc_client_t 			*Leader_client;
_GLOBAL_ playerc_position2d_t 		*Leader_position2d;
_GLOBAL_ playerc_sonar_t			*Leader_sonar;
_GLOBAL_ playerc_gps_t				*Leader_gps;
_GLOBAL_ playerc_position2d_t		*Leader_compass;
_GLOBAL_ playerc_position2d_t		*Leader_rawodom;
_GLOBAL_ playerc_laser_t			*Leader_laser;
_GLOBAL_ playerc_laser_t			*Leader_laser_org;
_GLOBAL_ playerc_position2d_t 		*Leader_vision;

_GLOBAL_ playerc_client_t 			*Follower_client[FOLLOWER_NUMBER];
_GLOBAL_ playerc_position2d_t 		*Follower_position2d[FOLLOWER_NUMBER];
_GLOBAL_ playerc_sonar_t			*Follower_sonar[FOLLOWER_NUMBER];
_GLOBAL_ playerc_gps_t			*Follower_gps[FOLLOWER_NUMBER];
_GLOBAL_ playerc_position2d_t		*Follower_compass[FOLLOWER_NUMBER];
_GLOBAL_ playerc_position2d_t		*Follower_rawodom[FOLLOWER_NUMBER];
_GLOBAL_ playerc_laser_t			*Follower_laser[FOLLOWER_NUMBER];
_GLOBAL_ playerc_laser_t			*Follower_laser_org[FOLLOWER_NUMBER];
_GLOBAL_ playerc_position2d_t 		*Follower_vision[FOLLOWER_NUMBER];

_GLOBAL_ playerc_position2d_t 		*Follower_VFH[FOLLOWER_NUMBER];
*/



_GLOBAL_  const char *Robot_Host[ROBOT_NUMBER];
_GLOBAL_  int Robot_Port[ROBOT_NUMBER];


_GLOBAL_ volatile double Laser_Ranges[ROBOT_NUMBER][700];



/******************************************************************/




_GLOBAL_	unsigned char g_operationFlag;	// Set flag to 1 to force program to g_operationFlag

/*****************************************************************************
					LEADER ROBOT
*****************************************************************************/
/*
//Connection IP, PORT
_GLOBAL_  const char *Leader_Robot_Host;
_GLOBAL_  int Leader_Robot_Port;

//odometry
_GLOBAL_  double	Leader_Position_VectorbyOdom[vector_count];
_GLOBAL_  double Leader_Pre_HeadingAnglebyOdom;
_GLOBAL_  double Leader_Cur_HeadingAnglebyOdom;

_GLOBAL_  double	Leader_Position_VectorbyOdomUpdatedEKF[vector_count];
_GLOBAL_  double Leader_Pre_HeadingAnglebyOdomUpdatedEKF;
_GLOBAL_  double Leader_Cur_HeadingAnglebyOdomUpdatedEKF;


//GPS,COMPASS For measurement
_GLOBAL_  double	Leader_Position_VectorbyGPS[vector_count];
_GLOBAL_  double Leader_Cur_HeadingAnglebyCompass;


//Laser ranges
_GLOBAL_ volatile double Leader_Laser_Ranges[700];

// Fusion(���� ��ǥ)
_GLOBAL_  double	Leader_Position_Vector[vector_count];
_GLOBAL_  double Leader_Pre_HeadingAngle;
_GLOBAL_  double Leader_Cur_HeadingAngle;

//for Formation Control
_GLOBAL_  double Leader_Normalized_Velocity_Vector[vector_count];
_GLOBAL_  double Leader_Orthognal_Vector_to_NormalizedVector[vector_count];
_GLOBAL_  double Leader_Curvature_K;

// LEADER Linear Velocity, Angular Velocity
_GLOBAL_  double Leader_MAX_Linear_Velocity;
_GLOBAL_  double Leader_MIN_Linear_Velocity;
_GLOBAL_  double Leader_CUR_Linear_Velocity;
_GLOBAL_  double Leader_MAX_Angle_Velocity;
_GLOBAL_  double Leader_MIN_Angle_Velocity;
_GLOBAL_  double Leader_CUR_Angle_Velocity;
*/

/*****************************************************************************
					FOLLOWER  ROBOT
*****************************************************************************/
/*//Connection IP, PORT
_GLOBAL_  const char *Follower_Robot_Host[FOLLOWER_NUMBER];
_GLOBAL_  int Follower_Robot_Port[FOLLOWER_NUMBER];

//Odometry
_GLOBAL_  double Follower_Position_VectorbyOdom[FOLLOWER_NUMBER][vector_count];
_GLOBAL_  double Follower_Cur_HeadingAnglebyOdom[FOLLOWER_NUMBER];
_GLOBAL_  double Follower_Pre_HeadingAnglebyOdom[FOLLOWER_NUMBER];

//RAWOdometry
_GLOBAL_  double Follower_Position_VectorbyOdomUpdatedEKF[FOLLOWER_NUMBER][vector_count];
_GLOBAL_  double Follower_Cur_HeadingAnglebyOdomUpdatedEKF[FOLLOWER_NUMBER];
_GLOBAL_  double Follower_Pre_HeadingAnglebyOdomUpdatedEKF[FOLLOWER_NUMBER];


//GPS, COMPASS
_GLOBAL_  double Follower_Position_VectorbyGPS[FOLLOWER_NUMBER][vector_count];
_GLOBAL_  double Follower_Cur_HeadingAnglebyCompass[FOLLOWER_NUMBER];

//Laser ranges
_GLOBAL_ volatile double Follower_Laser_Ranges[FOLLOWER_NUMBER][700];



// Fusion(���� ��ǥ)
_GLOBAL_  double Follower_Position_Vector[FOLLOWER_NUMBER][vector_count];
_GLOBAL_  double Follower_Cur_HeadingAngle[FOLLOWER_NUMBER];
_GLOBAL_  double Follower_Pre_HeadingAngle[FOLLOWER_NUMBER];

//for Formation Control
_GLOBAL_  double Follower_Normalized_Velocity_Vector[FOLLOWER_NUMBER][vector_count];
_GLOBAL_  double Follower_Orthognal_Vector_to_NormalizedVector[FOLLOWER_NUMBER][vector_count];
_GLOBAL_  double Follower_Curvature_K[FOLLOWER_NUMBER];

// FOLLOWER Linear Velocity, Angular Velocity
_GLOBAL_  double Follower_MAX_Linear_Velocity[FOLLOWER_NUMBER];
_GLOBAL_  double Follower_MIN_Linear_Velocity[FOLLOWER_NUMBER];
_GLOBAL_  double Follower_CUR_Linear_Velocity[FOLLOWER_NUMBER];
_GLOBAL_  double Follower_REQ_Linear_Velocity[FOLLOWER_NUMBER];

_GLOBAL_  double Follower_MAX_Angle_Velocity[FOLLOWER_NUMBER];
_GLOBAL_  double Follower_MIN_Angle_Velocity[FOLLOWER_NUMBER];
_GLOBAL_  double Follower_CUR_Angle_Velocity[FOLLOWER_NUMBER];
_GLOBAL_  double Follower_REQ_Angle_Velocity[FOLLOWER_NUMBER];

*/

/*****************************************************************************
			LEADER �� FOLLOWER ROBOT ��踦 ���� ����  for Formation Control
*****************************************************************************/

// Eta_Gain_Value, Target_Relatvie_Angle, Distance, Diff_Heading_Angle ���� Total Robot������ 1�� ���.
// (�κ��� �ش��ϴ� ������ �ƴ϶� �����(�� �κ��ΰ��� 1����)���� �ش�.) �����Ұ�.
// Eta_Gain_Value�� Follower�κ��� �ӵ��� ���ϴ� ���ΰ��̹Ƿ� �������� 1������.

//Leader Robot�� Follower�κ��� ������� �񱳷� ���� ����

_GLOBAL_  double Target_Relative_Angle[FOLLOWER_NUMBER];
_GLOBAL_  double Target_Relative_Distance[FOLLOWER_NUMBER];
_GLOBAL_  double Diff_Heading_Angle_Beta[FOLLOWER_NUMBER];
_GLOBAL_  double Cone_Angle_alpha[FOLLOWER_NUMBER];

_GLOBAL_  double Normalized_Velocity_Vector[FOLLOWER_NUMBER][vector_count];
_GLOBAL_  double Orthognal_Vector_to_NormalizedVector[FOLLOWER_NUMBER][vector_count];

_GLOBAL_  double Error_Vector[FOLLOWER_NUMBER][vector_count];
_GLOBAL_  double Cal_Time_Varying_Gain_Eta[FOLLOWER_NUMBER][5];
_GLOBAL_  double Minimum_Eta_gainVal[FOLLOWER_NUMBER];
_GLOBAL_  double Sutiable_Eta_GainVal[FOLLOWER_NUMBER];


/*****************************************************************************/


/*****************************************************************************
LOCALIZATION( EXTENDED KALMAN FILTER : Odometry(prediction) + GPS,COMPASS(Update)
*****************************************************************************/
_GLOBAL_  double Ts_CurMovingAngle[ROBOT_NUMBER];
_GLOBAL_  double Ts_PreMovingAngle[ROBOT_NUMBER];



// Extended Kalman Filter
/* Q :Process Noise , Odometry(Encoder)
|  covariance         0                  0          | : x
|       0 		  covariance         0          | : y 
|       0			0		covariance  |: tehta
*/
_GLOBAL_  double 	Error_cov_Qx[ROBOT_NUMBER][3];
_GLOBAL_  double 	Error_cov_Qy[ROBOT_NUMBER][3];
_GLOBAL_  double 	Error_cov_Qtheta[ROBOT_NUMBER][3];
/* Q :Measurement Noise , GPS , Compass
|  covariance         0                  0          | : x
|       0 		  covariance         0          | : y 
|       0			0		covariance  |: tehta
*/
_GLOBAL_  double 	Error_cov_Rx[ROBOT_NUMBER][3];
_GLOBAL_  double 	Error_cov_Ry[ROBOT_NUMBER][3];
_GLOBAL_  double 	Error_cov_Rtheta[ROBOT_NUMBER][3];

_GLOBAL_  double tmp_state_x[ROBOT_NUMBER];
_GLOBAL_  double tmp_state_y[ROBOT_NUMBER];
_GLOBAL_  double tmp_state_theta[ROBOT_NUMBER];

_GLOBAL_  double tmp_cov_x[ROBOT_NUMBER][3];
_GLOBAL_  double tmp_cov_y[ROBOT_NUMBER][3];
_GLOBAL_  double tmp_cov_theta[ROBOT_NUMBER][3];

_GLOBAL_  double pre_tmp_cov_x[ROBOT_NUMBER][3];
_GLOBAL_  double pre_tmp_cov_y[ROBOT_NUMBER][3];
_GLOBAL_  double pre_tmp_cov_theta[ROBOT_NUMBER][3];

_GLOBAL_  double Kalman_Gain_x[ROBOT_NUMBER][3];
_GLOBAL_  double Kalman_Gain_y[ROBOT_NUMBER][3];
_GLOBAL_  double Kalman_Gain_theta[ROBOT_NUMBER][3];

_GLOBAL_  double final_state_x[ROBOT_NUMBER];
_GLOBAL_  double final_state_y[ROBOT_NUMBER];
_GLOBAL_  double final_state_theta[ROBOT_NUMBER];

_GLOBAL_  double tmp_tx_state_x[ROBOT_NUMBER];
_GLOBAL_  double tmp_tx_state_y[ROBOT_NUMBER];
_GLOBAL_  double tmp_tx_state_theta[ROBOT_NUMBER];


_GLOBAL_  double final_cov_x[ROBOT_NUMBER][3];
_GLOBAL_  double final_cov_y[ROBOT_NUMBER][3];
_GLOBAL_  double final_cov_theta[ROBOT_NUMBER][3];

_GLOBAL_  double mea_x_GPS[ROBOT_NUMBER]; // mea : measurement
_GLOBAL_  double mea_y_GPS[ROBOT_NUMBER];
_GLOBAL_  double mea_theta_Com[ROBOT_NUMBER]; //com : compass

_GLOBAL_  double TX_state_x[ROBOT_NUMBER];
_GLOBAL_  double TX_state_y[ROBOT_NUMBER];
_GLOBAL_  double TX_state_theta[ROBOT_NUMBER];



_GLOBAL_ unsigned char Formation_Control_StartFlag;

_GLOBAL_ unsigned char FollowerTotalNum;

