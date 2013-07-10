#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <msmr_client/main.h>
//#include <boost/bind.hpp>






t_ROBOT_STATE 	  s_Robot[ROBOT_NUMBER], ts_Robot[ROBOT_NUMBER];
t_RELATIVE_STATE  s_Formation[FOLLOWER_NUMBER];

double odom_v[ROBOT_NUMBER],odom_w[ROBOT_NUMBER];


double scan_range[ROBOT_NUMBER][690];

FILE *f_F_object[2];

//boost::mutex lock;









/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void Robot1_Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//ROS_INFO("I heard: [%s]",msg->header.frame_id.c_str());
	odom_v[0] = msg->twist.twist.linear.x;
	odom_w[0] = msg->twist.twist.angular.z;
}

void Robot2_Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//ROS_INFO("I heard: [%s]",msg->child_frame_id.c_str());
	odom_v[1] = msg->twist.twist.linear.x;
	odom_w[1] = msg->twist.twist.angular.z;
}

void Robot1_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int i;
	for(i=0;i<690;i++)
		scan_range[0][i] = msg->ranges[i];
}
void Robot2_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int i;
	for(i=0;i<690;i++)
		scan_range[1][i] = msg->ranges[i];
}










int main(int argc, char **argv)
{
	ros::init(argc, argv, "main");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	FILE *f_Leader;
	FILE *f_F1;
	FILE *f_F2;
	FILE *f_L_m;
	FILE *f_F1_m;
	FILE *f_F2_m;
	FILE *f_F1_m_EKF;
	FILE *f_F2_m_EKF;
	FILE *f_L_encoder;
	FILE *f_F1_encoder;
	FILE *f_F2_encoder;
	FILE *f_L_odom;
	FILE *f_F1_odom;
	FILE *f_F2_odom;
	FILE *f_L_com;
	FILE *f_F1_com;
	FILE *f_F2_com;
	FILE *f_time;
	FILE *f_F1_laser;
	FILE *f_F2_laser;
	FILE *f_F1_req_command;
	FILE *f_F2_req_command;
	FILE *f_F1_v_PID;
	FILE *f_F1_w_PID;

	FILE *f_F1_laser_data;
	FILE *f_F1_laser_f_data;

	FILE *f_F2_laser_data;
	FILE *f_F2_laser_f_data;

	FILE *f_F1_mode;
	FILE *f_F2_mode;

	FILE *f_F1_formation;
	FILE *f_F2_formation;


	f_time = fopen("Sim_Log_file/time.txt","w");

	f_Leader = fopen("Sim_Log_file/log_leader.txt","w");
	f_L_odom = fopen("Sim_Log_file/log_L_odom.txt","w");
	f_L_com = fopen("Sim_Log_file/log_L_com.txt","w");

	f_F1_odom = fopen("Sim_Log_file/log_F1_odom.txt","w");
	f_F1_com = fopen("Sim_Log_file/log_F1_com.txt","w");
	f_F1_laser = fopen("Sim_Log_file/log_F1_laser.txt","w");
	f_F1_req_command = fopen("Sim_Log_file/log_F1_command.txt","w");

	f_F2_odom = fopen("Sim_Log_file/log_F2_odom.txt","w");
	f_F2_com = fopen("Sim_Log_file/log_F2_com.txt","w");
	f_F2_laser = fopen("Sim_Log_file/log_F2_laser.txt","w");
	f_F2_req_command = fopen("Sim_Log_file/log_F2_command.txt","w");


	f_F1_v_PID=fopen("Sim_Log_file/log_F1_v_PID.txt","w");
	f_F1_w_PID=fopen("Sim_Log_file/log_F1_w_PID.txt","w");

	f_F1_laser_data = fopen("Sim_Log_file/log_F1_laser_data.txt","w");
	f_F1_laser_f_data = fopen("Sim_Log_file/log_F1_laser_f_data.txt","w");
	f_F_object[0]= fopen("Sim_Log_file/log_F1_object.txt","w");

	f_F2_laser_data = fopen("Sim_Log_file/log_F2_laser_data.txt","w");
	f_F2_laser_f_data = fopen("Sim_Log_file/log_F2_laser_f_data.txt","w");
	f_F_object[1]= fopen("Sim_Log_file/log_F2_object.txt","w");

	f_F1_mode = fopen("Sim_Log_file/log_F1_mode.txt","w");
	f_F2_mode = fopen("Sim_Log_file/log_F2_mode.txt","w");

	f_F1_formation = fopen("Sim_Log_file/log_F1_formation.txt","w");
	f_F2_formation = fopen("Sim_Log_file/log_F2_formation.txt","w");





	geometry_msgs::Twist robot1_cmdvel,robot2_cmdvel;

	ros::Subscriber robot1_odom_sub = n.subscribe("robot1/odom", 1, Robot1_Odom_Callback);
	ros::Subscriber robot2_odom_sub = n.subscribe("robot2/odom", 1, Robot2_Odom_Callback);

	ros::Subscriber robot1_scan_sub = n.subscribe("/robot1/base_scan/scan", 1, Robot1_Scan_Callback);
	ros::Subscriber robot2_scan_sub = n.subscribe("/robot2/base_scan/scan", 1, Robot2_Scan_Callback);

	ros::Publisher robot1_cmd_pub = n.advertise<geometry_msgs::Twist>("robot1/cmd_vel", 1);
	ros::Publisher robot2_cmd_pub = n.advertise<geometry_msgs::Twist>("robot2/cmd_vel", 1);


	Init_Val();

	robot1_cmdvel.linear.x = 0.0;
	robot1_cmdvel.angular.z = 0.0;

	robot2_cmdvel.linear.x = 0.0;
	robot2_cmdvel.angular.z = 0.0;



	int i,j;
	int ccc=0;
	int init_count = 0;



	double Cr=0;
	double CR=0;
	double al=1;
	double leader_w_temp=0;

	double Bearing_smoothing_thr=DToR(10);
	double Bearing_smoothing_time=DToR(0);
	double alpha = 0.01;
	double Diff_bearing_goalbearing;

	double evr,evl;

	unsigned char g_operationFlag=0;
	/*
  while(ros::ok())
  {
	  	robot1_cmdvel.linear.x = 0.2;
	    robot1_cmdvel.angular.z = 0.1;

	    robot2_cmdvel.linear.x = 0.5;
	    robot2_cmdvel.angular.z = 0.1;

	    robot1_cmd_pub.publish(robot1_cmdvel);
	    robot2_cmd_pub.publish(robot2_cmdvel);

	    ros::spinOnce();
	    loop_rate.sleep();

  }
	 */
	while(1)
	{
		ros::spinOnce();
		loop_rate.sleep();

		init_count++;
		if(init_count>10)
			break;


	}
	while(ros::ok())// || !g_operationFlag)
	{


		GetOdometryData();



		//robot1_cmd_pub.publish(robot1_cmdvel);
		//robot2_cmd_pub.publish(robot2_cmdvel);

		for(i=F1_L; i<FOLLOWER_NUMBER;i++)
		{


			Laser_process(s_Robot[i+1].laser_data,s_Formation[i].Bearing_Ang, s_Formation[i].Laser_data, i);

			//s_Robot[i+1].vfh_mode = Robot_VFH[i+1]->stall;
			//printf("vfh mode : %d\n",s_Robot[i+1].vfh_mode);

			if(s_Formation[i].Laser_data[0])
			{
				s_Formation[i].Distance = s_Formation[i].Laser_data[0];
				s_Formation[i].Bearing_Ang = s_Formation[i].Laser_data[1];
				s_Formation[i].x = s_Formation[i].Distance*cos(s_Formation[i].Bearing_Ang)+0.25;
				s_Formation[i].y = s_Formation[i].Distance*sin(s_Formation[i].Bearing_Ang);
				s_Formation[i].Distance = sqrt(s_Formation[i].x*s_Formation[i].x + s_Formation[i].y*s_Formation[i].y);
				s_Formation[i].Bearing_Ang = atan2(s_Formation[i].y,s_Formation[i].x);
			}

			printf("bering : %f    Distance : %f",RToD(s_Formation[i].Bearing_Ang),s_Formation[i].Distance);

			s_Formation[i].Beta = s_Robot[L].com_th - s_Robot[i+1].com_th;

			if(s_Formation[i].Beta > PI)
				s_Formation[i].Beta-= 2*PI;
			else if(s_Formation[i].Beta < -PI)
				s_Formation[i].Beta += 2*PI;



		}
		for(i=F1_L; i<FOLLOWER_NUMBER;i++)
		{
			//if(ccc>50);
			//s_Formation[i].Goal_B_angle = DToR(45);
			Diff_bearing_goalbearing = fabs(s_Formation[i].Bearing_Ang-s_Formation[i].Goal_B_angle);
			//if(Diff_bearing_goalbearing> Bearing_smoothing_thr)
			/*
			if(Diff_bearing_goalbearing> Bearing_smoothing_thr+Bearing_smoothing_time/2)
			{
				s_Formation[i].Virtual_Goal_B_angle = s_Formation[i].Goal_B_angle*alpha+s_Formation[i].Bearing_Ang*(1-alpha);
				printf("[1]\n");
			}
			else if((Diff_bearing_goalbearing < Bearing_smoothing_thr+Bearing_smoothing_time/2) && (Diff_bearing_goalbearing > Bearing_smoothing_thr-Bearing_smoothing_time/2))
			{
				s_Formation[i].Virtual_Goal_B_angle = (s_Formation[i].Goal_B_angle*alpha+s_Formation[i].Bearing_Ang*(1-alpha))*(sin(PI/Bearing_smoothing_time*(Diff_bearing_goalbearing-(Bearing_smoothing_thr)))+1)/2+s_Formation[i].Goal_B_angle*(1-(sin(PI/Bearing_smoothing_time*(Diff_bearing_goalbearing-(Bearing_smoothing_thr)))+1)/2);
				printf("[2]\n");
			}
			else
			{
				s_Formation[i].Virtual_Goal_B_angle = s_Formation[i].Goal_B_angle;
				printf("[3]\n");
			}*/
			s_Formation[i].Virtual_Goal_B_angle = s_Formation[i].Goal_B_angle;

			s_Formation[i].Ex = (s_Formation[i].Distance * cos(s_Formation[i].Bearing_Ang))-(s_Formation[i].Goal_Distance * cos(s_Formation[i].Virtual_Goal_B_angle));
			s_Formation[i].Ey = (s_Formation[i].Distance * sin(s_Formation[i].Bearing_Ang))-(s_Formation[i].Goal_Distance * sin(s_Formation[i].Virtual_Goal_B_angle));

			s_Formation[i].Dis_pre_P = s_Formation[i].Dis_P;
			s_Formation[i].Dis_P = s_Formation[i].Ex;
			s_Formation[i].Dis_Integ += s_Formation[i].Dis_P;
			s_Formation[i].Dis_D = s_Formation[i].Dis_pre_P - s_Formation[i].Dis_P;

			s_Formation[i].Bea_pre_P = s_Formation[i].Bea_P;
			s_Formation[i].Bea_P = s_Formation[i].Ey;
			s_Formation[i].Bea_Integ += s_Formation[i].Bea_P;
			s_Formation[i].Bea_D = s_Formation[i].Bea_pre_P - s_Formation[i].Bea_P;



			s_Robot[L].w = s_Robot[L].target_w;
			if(s_Robot[L].w==0)
			{
				Cr=0;
				CR = 0;//Goal_Distance[i]*sin(Goal_B_angle[i]);
			}
			else
			{
				al = s_Robot[L].w/fabs(s_Robot[L].w);
				Cr = s_Robot[L].v/s_Robot[L].w;
				CR = sqrt((s_Formation[i].Goal_Distance*sin(s_Formation[i].Goal_B_angle)*s_Formation[i].Goal_Distance*sin(s_Formation[i].Goal_B_angle))+(Cr*Cr)-(s_Formation[i].Goal_Distance*s_Formation[i].Goal_Distance));
				CR += s_Formation[i].Goal_Distance*sin(s_Formation[i].Goal_B_angle);
			}




			s_Formation[i].Beta=0;
			//target_v_E[i] = s_Robot[L].v+((CR-Cr)*s_Robot[L].w);
			s_Robot[i+1].target_v_E = s_Robot[L].v*(cos(s_Formation[i].Beta-s_Formation[i].Goal_B_angle)/cos(s_Formation[i].Goal_B_angle));
			s_Robot[i+1].target_v_S = (s_Formation[i].Dis_P*VP_gain)+(s_Formation[i].Dis_Integ*VI_gain)+(s_Formation[i].Dis_D*VD_gain);

			s_Robot[i+1].target_w_E = 0;//s_Robot[L].v*(sin(s_Formation[i].Beta)/(s_Formation[i].Goal_Distance*cos(s_Formation[i].Goal_B_angle)));
			s_Robot[i+1].target_w_S = (s_Formation[i].Bea_P*WP_gain)+(s_Formation[i].Bea_Integ*WI_gain)+(s_Formation[i].Bea_D*WD_gain);

			if(s_Formation[i].Dis_Integ > 0.5)
				s_Formation[i].Dis_Integ = 0.5;
			if(s_Formation[i].Bea_Integ > 0.2)
				s_Formation[i].Bea_Integ = 0.2;
			else if(s_Formation[i].Bea_Integ < -0.2)
				s_Formation[i].Bea_Integ = -0.2;

			s_Formation[i].VFH_max_speed = s_Robot[L].v+s_Formation[i].Distance-s_Formation[i].Goal_Distance;
			if(s_Formation[i].VFH_max_speed > 0.7)
				s_Formation[i].VFH_max_speed=0.7;
			//VFH_target_pos[i][X] = s_Robot[i].x;//-(Goal_Distance[i]*cos(Goal_B_angle[i]));
			s_Formation[i].VFH_target_pos[X] = s_Formation[i].x;
			s_Formation[i].VFH_target_pos[Y] = s_Formation[i].y;//-(s_Formation[i].Goal_Distance*sin(s_Formation[i].Goal_B_angle));
			s_Formation[i].VFH_target_pos[TH] = 0;




		}

		for(i=L; i<ROBOT_NUMBER;i++)
		{
			if(i==L)
			{


#ifdef REAL_ROBOT_PLATFORM
				if(ccc <300)
				{robot1_cmdvel.linear.x = 0.5;	robot1_cmdvel.angular.z = 0.0}
				else if(ccc < 600)
				{robot1_cmdvel.linear.x = 0.5;	robot1_cmdvel.angular.z = 0.2}
				else if(ccc < 800)
				{robot1_cmdvel.linear.x = 0.5;	robot1_cmdvel.angular.z = 0.0}
				else if(ccc < 1100)
				{robot1_cmdvel.linear.x = 0.5;	robot1_cmdvel.angular.z = -0.2}
				else if(ccc < 1300)
				{robot1_cmdvel.linear.x = 0.5;	robot1_cmdvel.angular.z = 0.0}
				else if(ccc < 1350)
				{robot1_cmdvel.linear.x = 0.5;	robot1_cmdvel.angular.z = 0.0}

#endif

#ifdef SIMULATION

				if(ccc<200)//ccc <80)
				{robot1_cmdvel.linear.x=0.3;  robot1_cmdvel.angular.z=0;}
				else if(ccc < 440)
				{robot1_cmdvel.linear.x=0.3;  robot1_cmdvel.angular.z=-0.13;}//-0.098125;}
				else if(ccc < 500)
				{robot1_cmdvel.linear.x=0.3;  robot1_cmdvel.angular.z=0;}
				else if(ccc < 820)
				{robot1_cmdvel.linear.x=0.3;  robot1_cmdvel.angular.z=0.098125;}
				else if(ccc < 1000)
				{robot1_cmdvel.linear.x=0.3;  robot1_cmdvel.angular.z=0;}
				else if(ccc < 1050)
				{robot1_cmdvel.linear.x=0.0;  robot1_cmdvel.angular.z=0;}


#endif
				else
					g_operationFlag=1;


			}
			else
			{
				s_Robot[i].target_v = s_Robot[i].target_v_E + s_Robot[i].target_v_S;
				s_Robot[i].target_w = s_Robot[i].target_w_E + s_Robot[i].target_w_S;

				if(s_Robot[i].target_v<0)
					s_Robot[i].target_v=0;
				else if(s_Robot[i].target_v > 1)
					s_Robot[i].target_v=1;

			}



			s_Robot[i].v_target_v = s_Robot[i].target_v;
			s_Robot[i].v_target_w = s_Robot[i].target_w;

			robot2_cmdvel.linear.x = s_Robot[i].v_target_v;
			robot2_cmdvel.angular.z = s_Robot[i].v_target_w;
			printf("Followerv : %f\n",robot2_cmdvel.linear.x);


			if(i!=L)
			{
				if(g_operationFlag)
				{
					robot2_cmdvel.linear.x = 0;
					robot2_cmdvel.angular.z = 0;

					robot2_cmd_pub.publish(robot2_cmdvel);
				}

				else if(s_Robot[L].v ==0)
				{
					robot2_cmdvel.linear.x = 0;
					robot2_cmdvel.angular.z = 0;

					//playerc_position2d_set_cmd_vel(Robot_position2d[i], 0,0,0,0);
					robot2_cmd_pub.publish(robot2_cmdvel);
					s_Robot[i].Run_mode=0;
					printf("Leader_stop\n");
				}
				else
				{
					//playerc_position2d_set_cmd_vel(Robot_VFH[i], s_Formation[i-1].VFH_max_speed,0,s_Formation[i-1].Bearing_Ang,0);// Leader position
					if(0)//s_Robot[i].vfh_mode)
					{
						// playerc_position2d_set_cmd_pose(Robot_VFH[i], s_Formation[i-1].VFH_target_pos[X], s_Formation[i-1].VFH_target_pos[Y],s_Formation[i-1].VFH_target_pos[TH], 1);

						printf("VFH_1\n\n");
						s_Formation[i-1].Dis_Integ=0;  s_Formation[i-1].Bea_Integ=0;
						s_Robot[i].Run_mode=1;
					}
					else
					{
						//playerc_position2d_set_cmd_pose(Robot_VFH[i], s_Formation[i-1].VFH_target_pos[X], s_Formation[i-1].VFH_target_pos[Y],s_Formation[i-1].VFH_target_pos[TH], 0);
						//playerc_position2d_set_cmd_vel(Robot_position2d[i], s_Robot[i].v_target_v,0,s_Robot[i].v_target_w,0);
						robot2_cmd_pub.publish(robot2_cmdvel);

						printf("(d,phi)-control\n\n");
						s_Robot[i].Run_mode=3;
					}

				}

			}
			else
				robot1_cmd_pub.publish(robot1_cmdvel);//playerc_position2d_set_cmd_vel(Robot_position2d[i], s_Robot[i].v_target_v,0,s_Robot[i].v_target_w,0);


		}

		ccc++;
		printf("count : %d\n",ccc);
		if(g_operationFlag)
			break;




		fprintf(f_Leader,"%f\t%f\n",s_Robot[L].x,s_Robot[L].y);
		fprintf(f_L_odom,"%f\t%f\t%f\t%f\n",s_Robot[L].target_v,s_Robot[L].target_w,s_Robot[L].v,s_Robot[L].w);
		//fprintf(f_L_com,"%f\t%f\n",Leader_compass->pa,RToD(s_Robot[L].com_th));




		fprintf(f_F1_laser,"%f\t%f\t%f\t%f\n",s_Formation[F1_L].Distance,RToD(s_Formation[F1_L].Bearing_Ang),s_Formation[F1_L].Ex,s_Formation[F1_L].Ey);
		fprintf(f_F1_odom,"%f\t%f\t%f\t%f\n",s_Robot[F1].target_v,s_Robot[F1].v,s_Robot[F1].target_w,s_Robot[F1].w);
		//fprintf(f_F1_com,"%f\t%f\n",Follower_compass[0]->pa,RToD(s_Robot[F1].com_th));
		fprintf(f_F1_req_command,"%f\t%f\n",s_Robot[F1].target_v,s_Robot[F1].target_w);
		fprintf(f_F1_mode,"%d      %d\n",ccc,s_Robot[F1].Run_mode);
		fprintf(f_F1_formation,"%f      %f\n",s_Formation[F1_L].Goal_Distance,RToD(s_Formation[F1_L].Virtual_Goal_B_angle));





		fprintf(f_F_object[0],"count : %d\n",ccc);







		ros::spinOnce();
		loop_rate.sleep();
	}
	init_count=0;
	while(1)
	{
		init_count++;
		printf("end\n");


		robot1_cmdvel.linear.x = 0.0;
		robot1_cmdvel.angular.z = 0.0;

		robot2_cmdvel.linear.x = 0.0;
		robot2_cmdvel.angular.z = 0.0;

		robot1_cmd_pub.publish(robot1_cmdvel);
		robot2_cmd_pub.publish(robot2_cmdvel);

		ros::spinOnce();
		loop_rate.sleep();

		if(init_count>50)
			break;
	}


	return 0;
}


//void GetOdometryData(t_ROBOT_STATE *Robot)
void GetOdometryData()
{
	int cnt;
	int i;
//	s_Robot[0].v = 0.1;//odom_v[cnt];
//	s_Robot[0].w = 0;//odom_w[cnt];

	for(cnt=L;cnt<ROBOT_NUMBER;cnt++)
	{
		s_Robot[cnt].v = odom_v[cnt];
		s_Robot[cnt].w = odom_w[cnt];

		//Robot[cnt].v_right = Robot[cnt].v+(0.3*Robot[cnt].w/2);
		//Robot[cnt].v_left = Robot[cnt].v-(0.3*Robot[cnt].w/2);

		for(i=0;i<690;i++)
			s_Robot[cnt].laser_data[i] = scan_range[cnt][i];


		/////
		//Robot[cnt].v_left = Robot[cnt].v-(0.3*Robot[cnt].w/2);
		//Robot[cnt].v_right = Robot[cnt].v+(0.3*Robot[cnt].w/2);
	}

	//	printf("%f\t%f\t\t%f\t%f\n",Robot[0].v,Robot[0].w,Robot[1].v,Robot[1].w);

#ifdef REAL_ROBOT_PLATFORM
	for(cnt=L;cnt<ROBOT_NUMBER;cnt++)
		Robot[cnt].com_th = 0;//DToR(Follower_compass[F1]->pa);
#else if SIMULATION
	for(cnt=L;cnt<ROBOT_NUMBER;cnt++)
		s_Robot[cnt].com_th = 0;

#endif


}

void Laser_process(double *Laser_ranges, double angle, double* Laser_data, int index)
{
	int i;
	int label = 0;
	int find_obj=0;
	int Laser_label[681];

	double Label_angle[300];
	double Label_distance[300];
	double Label_count[300];
	double Label_size[300];
	int Label_num;
	double temp_angle=3.14/8;
	int temp=0;
	//struct laser_data Laser_data;

	//angle = -DToR(angle);
	////printf("angle = %f\n",RToD(angle));
	//angle = -angle;
	//printf("angle laser : %f\n",RToD(angle));
	label=0;
	for(i = 0 ; i<680 ; i++)
	{
		//if(!i && Laser_ranges[i]<4)
		//label=1;
		if(Laser_ranges[i] < 4)
		{
			if((fabs(Laser_ranges[i]-Laser_ranges[i-1])>0.2 && (i-1)>-1/*(Laser_ranges[i-1])*/  ) )
				label++;
			else if(Laser_label[i-1]==0)
				label++;

			Laser_label[i] = label;

		}
		else
		{
			Laser_label[i]=0;
		}

	}

	find_obj=label+1;

	for(i=1 ; i<find_obj ; i++)
	{
		Label_count[i] = 0;
		Label_angle[i] = 0;
		Label_distance[i] = 0;
	}


	for(i=0 ; i<680 ; i++)
	{
		if(Laser_label[i])
		{	//printf("%d    %d\n",i,Laser_label[i]);
			Label_count[Laser_label[i]]++;
			Label_distance[Laser_label[i]] += Laser_ranges[i];
			Label_angle[Laser_label[i]] = i;
		}
	}


	for(i=1 ; i<find_obj ; i++)
	{
		Label_distance[i] =Label_distance[i]/ Label_count[i];
		Label_size[i] = (pow((Laser_ranges[(int)Label_angle[i]]*cos(DToR((-340+Label_angle[i])*0.35)))-(Laser_ranges[(int)(Label_angle[i]-Label_count[i]+1)]*cos(DToR((-340+Label_angle[i]-Label_count[i])*0.35))),2.0)
				+ pow((Laser_ranges[(int)Label_angle[i]]*sin(DToR((-340+Label_angle[i])*0.35)))-(Laser_ranges[(int)(Label_angle[i]-Label_count[i]+1)]*sin(DToR((-340+Label_angle[i]-Label_count[i])*0.35))),2.0));
		Label_size[i] = sqrt(Label_size[i]);
		Label_angle[i] = DToR((-340+Label_angle[i]-Label_count[i]/2)*0.35);// laser 정방향
		//Label_angle[i] = DToR((-340+Label_angle[i]-Label_count[i]/2)*-0.35);// laser 역방향

		//printf("[%d] angle : %f    distance : %f    size : %f  \n",i,RToD(Label_angle[i]),Label_distance[i],Label_size[i]);
		//fprintf(f_F_object[index],"[%d] angle : %f    distance : %f    size : %f  \n",i,RToD(Label_angle[i]),Label_distance[i],Label_size[i]);
	}

	Label_num = find_obj-1;
	//angle = (int)(340+RToD(angle)*34/12);
	//printf("angle : %f",RToD(angle));
	for(i=1 ; i<find_obj ; i++)
	{
		//printf("%f \n",fabs(angle-Label_angle[i]));
		if( fabs(angle-Label_angle[i]) < temp_angle)
		{
			if(Label_size[i] < 0.4 && Label_size[i] > 0.15 && Label_distance[i] < 3.0)
			{
				temp_angle = fabs(angle-Label_angle[i]);
				temp = i;
			}
		}
	}
	//fprintf(f_F_object[index],"temp : %d  ",temp);

	if(Laser_data[2])
	{
		Laser_data[2]=0;
		for(i=85; i<595 ; i++)
		{
			if(Laser_ranges[i]<1.5)
				if(Laser_label[i]!=temp && Label_size[Laser_label[i]] > 0.1)
				{
					Laser_data[2]=1;
					//printf("aa\n");
					break;
				}

		}
	}
	else
	{
		for(i=85; i<595 ; i++)//for(i=212; i<468 ; i++)
		{
			if(Laser_ranges[i]<1.5 && Label_size[Laser_label[i]] > 0.1)
				if(Laser_label[i]!=temp)
				{
					Laser_data[2]=1;
					//printf("bb\n");
					break;
				}

		}
	}

	if(/*temp_angle > 3.14/4 ||*/ temp==0)
	{
		Laser_data[0] = 0;
		Laser_data[1] = 0;
		//	printf("temp = 0\n");
	}
	else
	{
		Laser_data[0] = Label_distance[temp];
		Laser_data[1] = Label_angle[temp];
		//	printf("temp = %d\n",temp);
	}

}

void Init_Val(void)
{
	//FollowerTotalNum = FOLLOWER_NUMBER;

	s_Formation[F1_L].Goal_B_angle = DToR(-45);
	s_Formation[F1_L].Goal_Distance = 1.5;

	s_Formation[F1_L].Bearing_Ang = DToR(-45);
	s_Formation[F1_L].Distance = 1.5;


}



double DToR(double degree)
{
	return degree*3.14159265358979 /180.0;

}
double RToD(double radian)
{
	return radian*180.0 /3.14159265358979;
}
