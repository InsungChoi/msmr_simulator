#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <netinet/in.h>						// ������ �ؾ��ϴ� ��� ���Ͽ��� �̰��� include�� �Ǿ� �־����. �ƴ� make error �߻�. 
#include <error.h>
#include <stdio.h>
//#include <tgmath.h>						//�ﰢ�Լ� -  theta ���� radian����.
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <cmath>
//#include <ctgmath>

#include <msmr_client/commonDef.h>
#include <msmr_client/globalvariable.h>

//#include <boost/thread.hpp>
//#include <boost/bind.hpp>






FILE *fp_leader_logfile;
FILE *fp_follower_1_logfile;
FILE *fp_follower_2_logfile;

void print_usage();
void sig_quit(int signum);
void Init_Val(void);
void SettingIP_PORTforRobot(void);
void InitSystem(void);
void gauss_rand(double *y1, double *y2);
void GetOdometryData();
void RobotTrajectory(t_ROBOT_STATE* Robot);
void Laser_process(double *Laser_ranges, double angle, double* Laser_data, int index);
void Laser_filltering(double *Laser_ranges, double* f_Laser_data);
double DToR(double degree);
double RToD(double radian);

void Robot1_Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg);
void Robot2_Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg);

void Robot1_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
void Robot2_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);




