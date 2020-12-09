#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Imu.h"
#include "px4_control_cfg.h"
#include "geometry_msgs/Twist.h" 
#include "nlink_parser/LinktrackAnchorframe0.h"
#include "nlink_parser/LinktrackTagframe0.h"
#include "tello_driver/TelloStatus.h"

using namespace std;
//using namespace Eigen;

float tello_pos[3] = {0,0,0};
float yaw_angle;

float expectPos[3] = {0,0,0};
float expectYawAngle;
float initialYawAngle;

float zero_pos[3] = {0,0,0};

float vel_P;
float vel_D;
float vel_I;
float w_P;

int Battery_percentage;

geometry_msgs::Twist cmd_vel;

S_PID_ITEM PidItemX;
S_PID_ITEM PidItemY;
S_PID_ITEM PidItemZ;

int command;


geometry_msgs::Twist VelPIDController(float selfPos[3],float leaderPos[3],float expectPos[3],float expectyaw)
{
	geometry_msgs::Twist cmd_vel;

	//cmd.linear和cmd.angular范围都是-1到1，模拟的是摇杆最上和最下。tellopy自带限幅
	//但是tello的速度还有两档，fastmode?

	// tello飞机的平动机体坐标系是右前上坐标系,yaw也是朝上为正
	PidItemX.difference = leaderPos[0]+expectPos[0]-selfPos[0];
	PidItemX.differential = PidItemX.difference - PidItemX.tempDiffer;
	PidItemX.integral += PidItemX.difference;
	PidItemX.tempDiffer = PidItemX.difference;
	cmd_vel.linear.x = vel_P*PidItemX.difference + vel_D*PidItemX.differential + vel_I*PidItemX.integral;	

	// vy 
	PidItemY.difference = leaderPos[1]+expectPos[1]-selfPos[1];
	PidItemY.differential = PidItemY.difference - PidItemY.tempDiffer;
	PidItemY.integral += PidItemY.difference;
	PidItemY.tempDiffer = PidItemY.difference;
	cmd_vel.linear.y = vel_P*PidItemY.difference + vel_D*PidItemY.differential + vel_I*PidItemY.integral;

	// vz
	PidItemZ.difference = leaderPos[2]+expectPos[2]-selfPos[2];
	PidItemZ.differential = PidItemZ.difference - PidItemZ.tempDiffer;
	PidItemZ.integral += PidItemZ.difference;
	PidItemZ.tempDiffer = PidItemZ.difference;
	cmd_vel.linear.z = vel_P*PidItemZ.difference + vel_D*PidItemZ.differential + vel_I*PidItemZ.integral;

	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = 0;
	cmd_vel.angular.z = w_P*(expectyaw-yaw_angle);
	//注意！tello航向角朝下为正

	return cmd_vel;
}


void cb_readPos(const nlink_parser::LinktrackAnchorframe0::ConstPtr& msg)
{
	for(auto &item : msg->nodes)
		{
			if(item.id == 0)
				{
				tello_pos[0] =  item.pos_3d[0];
				tello_pos[1] =  item.pos_3d[1];
				//tello_pos[2] =  item.pos_3d[2]; //uwb的高度数据不准确
				}
		}

}

void cb_status(const tello_driver::TelloStatus::ConstPtr& msg)
{
	tello_driver::TelloStatus Tellodata;
	Tellodata = *msg;
	Battery_percentage = Tellodata.battery_percentage;
	tello_pos[2] = Tellodata.height_m;	
	//cout << "Battery_percentage: "<< Battery_percentage << endl;
	//cout << "tello_height: "<< tello_pos[2] << endl;
}


void cb_command(const std_msgs::Int32::ConstPtr& msg)
{
	std_msgs::Int32 cmd_data;
	cmd_data = *msg;
	command = cmd_data.data;
	cout << "receive command: " << command << endl;
}


void cb_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
	double Roll,Pitch,Yaw;
	sensor_msgs::Imu IMUdata;
	IMUdata = *msg;
	tf::Quaternion quat;
      	tf::quaternionMsgToTF(IMUdata.orientation,quat);
      	tf::Matrix3x3(quat).getRPY(Roll,Pitch,Yaw);

	Roll = Roll*180/3.14159;
	Pitch = Pitch*180/3.14159;
	Yaw = Yaw*180/3.14159;

	//cout << "IMU angle RPY: "<< Roll << " " << Pitch <<" " << Yaw << endl;
	//tello的yaw是以开机时的角度为0的

	yaw_angle = Yaw; 
}

void cb_expectPos(const geometry_msgs::Twist::ConstPtr& msg)
{
	geometry_msgs::Twist posdata;
	posdata = *msg;
	expectPos[0] = posdata.linear.x; //meter
	expectPos[1] = posdata.linear.y; //meter 
	expectPos[2] = posdata.linear.z; //meter
	expectYawAngle = posdata.angular.z + initialYawAngle; //degree 这个是在起飞航向角的基础上增加的值
	cout << "expectPos_x: "<< expectPos[0] << endl;
	cout << "expectPos_y: "<< expectPos[1] << endl;
	cout << "expectPos_z: "<< expectPos[2] << endl;
	cout << "expectYawAngle: "<< expectYawAngle << endl;
}




int main(int argc,char **argv)
{
	ros::init(argc,argv,"tello0");
	ros::NodeHandle nh("~");

	ros::Subscriber cmd_sub = nh.subscribe("/command",10,cb_command); 
	ros::Subscriber pos_sub = nh.subscribe("/nlink_linktrack_anchorframe0",10,cb_readPos); //cb means Callback
	ros::Subscriber expectPos_sub = nh.subscribe("/tello0/expectPos",10,cb_expectPos);
	ros::Subscriber status_sub = nh.subscribe("/tello0/status",10,cb_status); 
	ros::Subscriber imu_sub = nh.subscribe("/tello0/imu",10,cb_imu); 

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/tello0/cmd_vel",10); 
	ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>("/tello0/takeoff",10); 
	ros::Publisher land_pub = nh.advertise<std_msgs::Empty>("/tello0/land",10); 

	nh.param<float>("vel_P",vel_P,1.0);
	nh.param<float>("vel_D",vel_D,0);
	nh.param<float>("vel_I",vel_I,0);
	nh.param<float>("w_P",w_P,1.0);

	cout << "vel_P: "<< vel_P << endl;
	cout << "vel_D: "<< vel_D << endl;
	cout << "vel_I: "<< vel_I << endl;
	cout << "w_P: "<< w_P << endl;


	yaw_angle = 0;
	expectYawAngle = 0;
	initialYawAngle = 0;
	Battery_percentage = 0;
	cmd_vel.linear.x = 0;
	cmd_vel.linear.y = 0;
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = 0;
	PidItemX.tempDiffer = 0;
	PidItemX.integral = 0;
	PidItemY.tempDiffer = 0;
	PidItemY.integral = 0;
	PidItemZ.tempDiffer = 0;
	PidItemZ.integral = 0;
	command = 0;

	ros::Rate loop_rate(30);

	while(ros::ok())
	{
		if(command == 0) //确定期望航向角为此时的角度，且指令速度给0
			{
				initialYawAngle = yaw_angle;	
				expectYawAngle = yaw_angle;			
				//cout << "expectYawAngle: " << expectYawAngle << endl;
				PidItemX.tempDiffer = 0;
				PidItemX.integral = 0;
				PidItemY.tempDiffer = 0;
				PidItemY.integral = 0;	
				PidItemZ.tempDiffer = 0;
				PidItemZ.integral = 0;	
				cmd_vel.linear.x = 0;
				cmd_vel.linear.y = 0;
				cmd_vel.linear.z = 0;
				cmd_vel.angular.x = 0;
				cmd_vel.angular.y = 0;
				cmd_vel.angular.z = 0;
				vel_pub.publish(cmd_vel);
			}			
		if(command == 1)  //起飞
			{	
				std_msgs::Empty takeoff_cmd;
				takeoff_pub.publish(takeoff_cmd);							
			}	
		if(command == 2)  //飞到指令点
			{	
				if(expectPos[0]!=0 && expectPos[1]!=0 && expectPos[2]!=0 ) //没发期望位置就不飞
				{
					cmd_vel = VelPIDController(tello_pos,zero_pos,expectPos,expectYawAngle); 
					vel_pub.publish(cmd_vel);	
				}	
			}	
	if (command == 3)  // 指令速度给0,但期望航向角不变
			{
				PidItemX.tempDiffer = 0;
				PidItemX.integral = 0;
				PidItemY.tempDiffer = 0;
				PidItemY.integral = 0;	
				PidItemZ.tempDiffer = 0;
				PidItemZ.integral = 0;	
				cmd_vel.linear.x = 0;
				cmd_vel.linear.y = 0;
				cmd_vel.linear.z = 0;
				cmd_vel.angular.x = 0;
				cmd_vel.angular.y = 0;
				cmd_vel.angular.z = 0;
				vel_pub.publish(cmd_vel);
			}
		if(command == 9)  //降落
			{	
				std_msgs::Empty land_cmd;
				land_pub.publish(land_cmd);							
			}	
		ros::spinOnce();
		loop_rate.sleep();
	}  //无法在回调函数里发布话题，报错函数里没有定义vel_pub!只能在main里面发布了

	return 0;
}
