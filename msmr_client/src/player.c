/************** MsMr. 군집로봇 제어 시뮬레이션 프로그램***********************
 Starting it on 2009. 8.11  by LEE JAEMOON

 Modified on 8.13
 player.c

  
**********************************************************************/

#define _PLAYER_
#include "player.h"

void PrintRobotInformations(int robot_id)
{
	int cnt=0;
	if(robot_id == LEADER_ID){
		printf("*****************************************************\n");
		playerc_position2d_print(Robot_position2d[L],"Leader Position_Value\n");
		printf("*****************************************************\n");
	}
	else if(robot_id == FOLLOWER_ID){
		for(cnt=0;cnt<FOLLOWER_NUMBER;cnt++)
		{
			printf("*****************************************************\n");
			playerc_position2d_print(Robot_position2d[cnt+1],"Follower Position_Value\n");
			printf("*****************************************************\n");
		}
	}
}


void Move_Straight_Robot(int robot_id, double linear_velocity)
{
	int cnt;
	if(robot_id == LEADER_ID){
		playerc_position2d_set_cmd_vel(Robot_position2d[L], linear_velocity, 0, 0, 0);
	}
	else if(robot_id == FOLLOWER_ID){
		for(cnt=0;cnt<FOLLOWER_NUMBER;cnt++)
		{
			playerc_position2d_set_cmd_vel(Robot_position2d[cnt+1], linear_velocity, 0, 0, 0);
		}
	}
}

void Move_Turn_Robot(int robot_id, double linear_velocity, double angle_velocity)
{
	int cnt;
	if(robot_id == LEADER_ID){
		playerc_position2d_set_cmd_vel(Robot_position2d[L], linear_velocity, 0, angle_velocity, 0);
	}
	else if(robot_id == FOLLOWER_ID){

		for(cnt=0;cnt<FOLLOWER_NUMBER;cnt++)
		{
			playerc_position2d_set_cmd_vel(Robot_position2d[cnt+1], linear_velocity, 0, angle_velocity, 0);
		}
	}
}

void ReadRobotState(void)
{
	int cnt;
	for(cnt=L;cnt<ROBOT_NUMBER;cnt++)
		playerc_client_read(Robot_client[cnt]);		
		
}

void Init_Create_PlayercClient(void)
{
	int cnt;
	for(cnt=L;cnt<ROBOT_NUMBER;cnt++)
	{
		
		Robot_client[cnt] = playerc_client_create(NULL, Robot_Host[cnt], Robot_Port[cnt]);
			
		if (playerc_client_connect(Robot_client[cnt]) != 0){
					
			fprintf(stderr, "error: %s\n", playerc_error_str());
			print_usage();
			
		}
	}

		
		


	

}


void	Position2d_Subscribe_Interfaces(int robot_id)
{

	// Create a position2d proxy (device id "position2d:0") and susbscribe
	// in read/write mode

	int cnt;
	
	for(cnt=L;cnt<ROBOT_NUMBER;cnt++){
		Robot_position2d[cnt] = playerc_position2d_create(Robot_client[cnt], 0);
		if (playerc_position2d_subscribe(Robot_position2d[cnt], PLAYERC_OPEN_MODE) != 0){
					
			fprintf(stderr, "error: %s\n", playerc_error_str());
			
		}
		if(cnt!=L)
		{
			Robot_VFH[cnt] = playerc_position2d_create(Robot_client[cnt], 3);
			if (playerc_position2d_subscribe(Robot_VFH[cnt], PLAYERC_OPEN_MODE) != 0){
						
			fprintf(stderr, "error: %s\n", playerc_error_str());
			
			}
			
		}
		
	}
}

void	Sensor_Subscribe_Interfaces(int robot_id,int sensor_id)
{

	int cnt,i;
	double tmp[700], *tmp2;

	// create 시 *.cfg에서 interface index가 두번째 인자임. 
	// ex)만약 configuration 파일에서 "localhsot:sonar:1" 인경우 아래의 두번째 인자는 1로 적어준다.
	if(robot_id == LEADER_ID){

		
		if(sensor_id == LASER_ID){
			Robot_laser[L] = playerc_laser_create(Robot_client[L], 1);

			if (playerc_laser_subscribe(Robot_laser[L], PLAYERC_OPEN_MODE) != 0){
				printf("leader laser error!!\n");
				fprintf(stderr, "error: %s\n", playerc_error_str());
				
			}
		}
		

		
	}
	else if(robot_id == FOLLOWER_ID){		


		
		for(cnt=F1;cnt<ROBOT_NUMBER;cnt++)
		{
		
			
			if(sensor_id == COMPASS_ID){
				//position2d:1 //compass
				Robot_compass[cnt] = playerc_position2d_create(Robot_client[cnt], 1);

				
				if (playerc_position2d_subscribe(Robot_compass[cnt], PLAYERC_OPEN_MODE) != 0){
					printf("follower compass error!!\n");
					fprintf(stderr, "error: %s\n", playerc_error_str());
					
				}


			}
			else if(sensor_id == LASER_ID){
				
				Robot_laser[cnt] = playerc_laser_create(Robot_client[cnt], 1);
				if (playerc_laser_subscribe(Robot_laser[cnt], PLAYERC_OPEN_MODE) != 0){
					printf("follower laser error!!\n");
					fprintf(stderr, "error: %s\n", playerc_error_str());
					
				}

				
			}
			

		}
	}
}






void Shutdown(int robot_id)
{
	// Shutdown and tidy up
	int cnt;
	if(robot_id == LEADER_ID){
		playerc_position2d_unsubscribe(Robot_position2d[L]);
		playerc_position2d_destroy(Robot_position2d[L]);
		
		playerc_client_disconnect(Robot_client[L]);
		playerc_client_destroy(Robot_client[L]);


		
		
	}	
	else if(robot_id == FOLLOWER_ID){

		for(cnt=0;cnt<FOLLOWER_NUMBER;cnt++)
		{
			playerc_position2d_unsubscribe(Robot_VFH[cnt+1]);
      		       playerc_position2d_destroy(Robot_VFH[cnt+1]);

			playerc_position2d_unsubscribe(Robot_position2d[cnt+1]);
			playerc_position2d_destroy(Robot_position2d[cnt+1]);

			playerc_client_disconnect(Robot_client[cnt+1]);
			playerc_client_destroy(Robot_client[cnt+1]);
		}
	}
}


