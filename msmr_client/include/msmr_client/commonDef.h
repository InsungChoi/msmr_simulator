
#ifndef _COMMON_H_
#define _COMMON_H_


//#define REAL_ROBOT_PLATFORM
#define SIMULATION

#ifdef SIMULATION
 	#define DYNAMICS
#endif

/****************************************************************************************
what sensor used for measurement?


*****************************************************************************************/
// measurement �����͸� ��뿩�� 
// ���谣 GPS����, COMPASS ������ Ȯ���ϰ��� ��쿡 ��� ODOMETRY�θ� EKF����. 

/****************************************************************************************
Make LogFIle for Debugging
*****************************************************************************************/

#define	CREAT_LOG_FILE

/****************************************************************************************
Monitoring For Debugging
*****************************************************************************************/
//#define	PRINT_EKF_DebugMode_Detail
//#define	PRINT_EKF_DebugMode_Detail_ForFINALDATA
//#define	DEBUG_FOR_FORMATION_DATA

//#define GPS_ON

/****************************************************************************************
Localization Algotrithms ��� ���� 
*****************************************************************************************/

//#define	EKF_ON

/****************************************************************************************
Formation Control Algotrithms ��� ���� 
*****************************************************************************************/

#define   FormationControl_Start


/****************************************************************************************
���� ����ϴ� ��, ������ ���Ͽ� 
*****************************************************************************************/


#define   ON				1
#define   OFF				0

#define	vector_count 	2
#define	X                          0
#define	Y 				1
#define   TH                        2

//#define   x                          0
//#define   y                          1
//#define   the                       2



/****************************************************************************************
�κ��� ���� 
*****************************************************************************************/

#define	FOLLOWER_NUMBER		1
#define	ROBOT_NUMBER		FOLLOWER_NUMBER+1

/****************************************************************************************
�κ��� �Ҵ�� IP �� port 
*****************************************************************************************/
#ifdef REAL_ROBOT_PLATFORM
	#define	LEADER_IP			"192.168.10.11"
	#define Follower_2_IP		"192.168.10.13"
	#define Follower_1_IP		"192.168.10.12"

	#define	Leader_ConnectingPortNum			6665
	#define	Follower_1_ConnectingPortNum		6665
	#define	Follower_2_ConnectingPortNum		6665
#endif

#ifdef SIMULATION
	#define	LEADER_IP			"localhost"
	#define Follower_1_IP		"localhost"
	#define Follower_2_IP		"localhost"

	#define	Leader_ConnectingPortNum			6665
	#define	Follower_1_ConnectingPortNum		6666
	#define	Follower_2_ConnectingPortNum		6667
#endif


/****************************************************************************************
����Ʈ���� �κ� ID, �� ����̽� ID: �ҽ��ڵ�  ���ϰ� �ϱ� ���Ͽ� 
*****************************************************************************************/

#define	LEADER_ID	   	0x1000
#define	FOLLOWER_ID 	0x1001
#define	FOLLOWER_1 		0x1002
#define	FOLLOWER_2 		0x1003


#define GPS_ID			0x53
#define COMPASS_ID		0x54
#define	SONAR_ID		0x55
#define	RAWODOMETRY_ID	0x56
#define LASER_ID        0x57
#define VISION_ID       0x58

#define L	 			       0
#define F1 				1
#define F2 				2

#define F1_L			       0
#define F2_L	     		       1

/****************************************************************************************
Formation Control PID Gains
*****************************************************************************************/
#ifdef REAL_ROBOT_PLATFORM
     #define VP_gain 1
     #define VI_gain 0.01
     #define VD_gain 0
     
     #define WP_gain 2
     #define WI_gain 0.01
     #define WD_gain 0
#endif

#ifdef  SIMULATION

     #define VP_gain   3
     #define VI_gain   0.01
     #define VD_gain  0
     
     #define WP_gain   2
     #define WI_gain   0.3
     #define WD_gain  0

/*
     #define VP_gain   3
     #define VI_gain   0.01
     #define VD_gain  0
     
     #define WP_gain   2
     #define WI_gain   0.3
     #define WD_gain  0
*/


     #define MP_gain  2
     #define MI_gain  0.2
     #define MD_gain 0
#endif



/*******************************************************************************************
Localization �� ���Ͽ� ����    Extended Kalman Filter ���� 
Process Noise  		odometry
Measurement Noise	gps,compass
****************
******************************************************************************/
#ifdef SIMULATION
	#define 	Error_std_odom_x				0.1
	#define 	Error_std_odom_y				0.1
	#define 	Error_std_odom_theta			DToR(10)

	#define     Error_std_odom_v                0.1
	#define     Error_std_odom_w                DTOR(5)
	
	#define     Error_std_wheel                 0.05
	#define		ERR_STD_DISTANCE				0.01
	#define		ERR_STD_BEARING					DToR(15)
	#define		ERR_STD_ORIENTATION				DToR(3)
	#define		ERR_STD_SOUND					DToR(5)
#endif

#ifdef REAL_ROBOT_PLATFORM
	#define     Error_std_wheel                 0.05
	#define		ERR_STD_DISTANCE				0.2
	#define		ERR_STD_BEARING					DToR(15)
	#define		ERR_STD_ORIENTATION				DToR(15)
	#define		ERR_STD_SOUND					DToR(15)
#endif //REAL_ROBOT_PLATFORM

/*******************************************************************************************
 Formation Control���� Relative Localization���� ����ϴ� d�� ���� �� ����
 d: �κ��� �κ��� �Ÿ� ��
 ���� : �κ��� �κ��� ���̰� Follower�� Leader�� ���հ�( ���������� ������ ��ġ�� - �������� Heading��)
**********************************************************************************************/

// Formation Control���� Relative Localization���� ����ϴ� d�� ���� �� ����.

//  K * d (radius)  < 1 �̾���ϹǷ�  d �ִ� 10�̶�� ��, 0.1��.
#define  Leader_MAXIMUM_Curvature_K		0.1
#define  MAX_DistancefromLeadertoFollower	10.0

/*************************************************************
                         *		Follower 1
				*     Leader
                         *		Follower 2	 	
**************************************************************/


/*********************************************************************************************
	�����κ� �ʱ� ��ġ �� �ӵ� ������ ���� ���� �κ�.
	�ùķ��̼� �� ���  ������ msmr.world�̹Ƿ�  ���
	Robot Configuration For Leader_Robot
	RL means Robot Leader	
**********************************************************************************************/



// player�� ��� �ٷ� ������ ��ġ�� 0�̹Ƿ� ��� Setpose �Լ��� �غ������� �ȵǴ� ���� ó����ġ�� ��ǥ��ŭ �����ش�
//0<RL_MIN_Linear_Velocity<= (RL_V) <= RL_MAX_Liner_Velocity
//Linear Velocity : m/s , Angle Velocity : rad/s


/*******************************************************************************************
	Follower �κ� �ʱ� ��ġ �� �ӵ� ������ ���� ���� �κ�.
	�ùķ��̼� �� ���  ������ msmr.world�̹Ƿ�  ���
	Robot Configuration For Leader_Robot
	RF means Robot Follower
**********************************************************************************************/
// Robot Configuration For Follower_Robot1
// RF means Robot Follower
// ��������� �����ϱ� ���Ͽ� �׳� RF�� ��.

#define D2R(degree) (degree * PI / 180.0)
#define R2D(radian) (radian * 180.0 / PI)

#define PI  3.14159265358979   

/*******************************************************************************************
	GAIN�� ��� �� �� ����ϴ� �Ķ���͵�.
	PAPER : " A Geometric Characterization of Leader-Follower Formation Control" ���� ���� �̸����� �����.
**********************************************************************************************/
#define   M 								1.0
#define   X1								0.85
#define   X2								0.9


/*********************************************************************************************
	DEBUG 
*********************************************************************************************/
//#define DEBUG_ROBOT_STATE		//robot State[x,y,theta]
//#define DEBUG_ROBOT_CONTROL
//#define DEBUG_MEASUREMENT_DATA
//#define DEBUG_MATRIX_TRANSPOSE
//#define DEBUG_MATRIX_CONTROLUPDATE
//#define DEBUG_MATRIX_TRANSPOSE_34
//#define DEBUG_MATRIX_R
//#define DEBUG_COVARIANCE		
//#define DEBUG_MEASUREMENTUPDATE
//#define DEBUG_EXTERNAL_SENSOR_DATA

/*********************************************************************************************
	Robot Control Limitation 
*********************************************************************************************/
#define MAX_V_SPEED			1.0
#define	T					0.05
#define	ALIGNMENT			0.3

/*********************************************************************************************
	FILE 
*********************************************************************************************/
#define LOG_FILE

#endif


