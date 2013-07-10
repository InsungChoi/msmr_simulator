#ifdef	_PLAYER_
	#ifndef _PLAYER_GLOBAL_
		#define _PLAYER_GLOBAL_	
	#endif
#else
	#ifndef _PLAYER_GLOBAL_
		#define _PLAYER_GLOBAL_	extern
	#endif
#endif


#include <unistd.h>
#include <string.h>
#include <netinet/in.h>						// 컴파일 해야하는 모든 파일에는 이것이 include가 되어 있어야함. 아님 make error 발생. 
#include <error.h>
#include <stdio.h>
#include <tgmath.h>						//삼각함수 -  theta 값은 radian값임.

#include <libplayerc/playerc.h>
#include "commonDef.h"
#include "globalvariable.h"



/**************** define Functions***********************/
_PLAYER_GLOBAL_ void Init_Create_PlayercClient(void);
_PLAYER_GLOBAL_ void Position2d_Subscribe_Interfaces(int robot_id);
_PLAYER_GLOBAL_ void Sensor_Subscribe_Interfaces(int robot_id,int sensor_id);
_PLAYER_GLOBAL_ void PrintRobotInformations(int robot_id);
_PLAYER_GLOBAL_ void Move_Straight_Robot(int robot_id, double linear_velocity);
_PLAYER_GLOBAL_ void Move_Turn_Robot(int robot_id, double linear_velocity, double angle_velocity);
_PLAYER_GLOBAL_ void Shutdown(int robot_id);
_PLAYER_GLOBAL_ void ReadRobotState(void);
/*****************************************************/





