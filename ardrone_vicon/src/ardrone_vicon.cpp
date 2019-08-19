#include "ros/ros.h"
#include "geometry_msgs/Twist.h"			//for command velocity
#include "geometry_msgs/Vector3.h"			//for command velocity
#include "std_msgs/Empty.h"	 				//For take off and landing
#include "ardrone_autonomy/CamSelect.h"    	// For toggling camera
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include "ardrone_autonomy/vector31.h"
#include "ardrone_autonomy/vector21.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "ardrone_autonomy/navdata_gps.h"    //Accessing ardrone's published data
#include <sensor_msgs/Joy.h>
#include <stdio.h>
#include <math.h>
#include <termios.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ardrone_vicon/Drone_odo.h"
#include "ardrone_vicon/est_co.h"
#include "nav_msgs/Odometry.h"
// including vicons message
#include "geometry_msgs/TransformStamped.h"

// class variables
std_msgs::Empty emp_msg;				// variable in the take off and landing class
geometry_msgs::Twist vel_msg;			// variable in the command velocity class

// Variables for Publishing
ros::Publisher T_pub_empty;				//take off publisher
ros::Publisher L_pub_empty;				//landing publisher
ros::Publisher velocity_publisher;		// velocity publisher

// Variables for Subscribing
ros::Subscriber pose_subscriber;		// ardrone navdata subsciber
ros::Subscriber gps_subscriber;		// ardrone navdata subsciber
ros::Subscriber imu_subscriber;		// ardrone navdata subsciber
ros::Subscriber joy_sub_;          
ros::Subscriber vicon_sub;		// vicon data topic subscriber for ardrone
ros::Subscriber vicon_sub_2;    // vicon data topic subscriber for target 
ros::Subscriber vicon_sub_3;    // vicom data topic subscriber for obstacle
// Variables for Service
ros::ServiceClient client1;		// ardrone camera service

using namespace std;

float lx, ly, lz, ax, ay, az,foc = 685, angx,angy,angz, est_x, est_y, est_z,K_est_x, K_est_y, K_est_z,test_x,test_y,test_z;
int to, la, f = 200 , redX, redY, res =0, H=0,I=0,J=0;
double k = 0.51, z, tf, ang_time,hem; // k value for tau
double lat, lon, ele;
double xD=7,yD=4,zD=1;
//gps variables///
double new_lat, new_lon;
bool gps_cntr = 0;
//static float pos_x = 0, pos_y = 0, pos_z = 0;
const double PI = 3.14159265359;

//class instances
ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata
sensor_msgs::Imu ang;
nav_msgs::Odometry odo;
geometry_msgs::TransformStamped vicon_statedata;  	// Using this class variable for storing state data for ardron1 from Vicon 
geometry_msgs::TransformStamped vicon_statedata_2;  // Using this class variable for storing state data for target from Vicon
geometry_msgs::TransformStamped vicon_statedata_3;	// Using this class variable for storing state data for obstacle from Vicon 

//callback function declarations
void NumCallback(const ardrone_vicon::Drone_odo::ConstPtr& val);
void EstCallback(const ardrone_vicon::est_co::ConstPtr& est);
void INS_K_Callback(const ardrone_vicon::est_co::ConstPtr& esti);
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message);		// drone actual data
void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message);		// drone actual data
void new_gpsCallback(const std_msgs::String::ConstPtr& gps1);
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message);		// drone actual data
void joyCallback(const sensor_msgs::Joy::ConstPtr & joy);		// Joy data
int getch();
void testdataCallback(const ardrone_autonomy::vector31::ConstPtr & test );
void ViconStateCallback(const geometry_msgs::TransformStamped::ConstPtr & vicon_state_message); //callback function for vicon data of ardrone1
void ViconStateCallback_2(const geometry_msgs::TransformStamped::ConstPtr & vicon_state_message); //callback function for vicon data of target
void ViconStateCallback_3(const geometry_msgs::TransformStamped::ConstPtr & vicon_state_message); //callback function for vicon data of obstcale
void hover(int timee);		// hovering
void takeoff();		// take off
void land();		//landing
void move(float lx, float ly, float lz, float ax, float ay, float az ); // publishing command velocity
void keyboard_control();
void pose_measurement();
void calibration();
void trajectory();
void wait(double x);
void movement(double goal_x, double goal_y);
void target_follow();
void potential_field_track();
void potential_obstacle();
void potential_track_obstacle();

int main(int argc, char **argv)
{
	//Initiate the ROS

	ros::init(argc, argv, "ardrone_vicon");
	ros::NodeHandle n; 			// Nodehandle is like a class where you are operating

	// Publishing the data
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100); // initialize to send the Control Input

	T_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);  //initialize to send the take off command  /* Message queue length is just 1 */

	L_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/land", 1); //initialize to send the land command /* Message queue length is just 1 */

	// Subscribing the data
	pose_subscriber = n.subscribe("/ardrone/navdata", 200, poseCallback);	//initialize to receive processed sensor data
	imu_subscriber = n.subscribe("/ardrone/imu", 200, imuCallback); //initialize to receive raw sensor data
	gps_subscriber = n.subscribe("/ardrone/navdata_gps", 10, gpsCallback); //initialize to receive gps data
        //joy sub
	joy_sub_ = n.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);
	//vicon data subscriber
	vicon_sub = n.subscribe("/vicon/ardrone1/ardrone1",200,ViconStateCallback);
	vicon_sub_2 = n.subscribe("/vicon/target/target",200,ViconStateCallback_2);
	vicon_sub_3 = n.subscribe("/vicon/obstacle/obstacle",200,ViconStateCallback_3);
while(ros::ok())
{

cout<<endl<<"function menu"<<endl;
cout<<"y : takeoff        h : land           k : keyboard control"<<endl;
cout<<"a : calibration    b : trajectory     c : pose measurement"<<endl;
cout<<"f : target follow  p : potential field track"<<endl;
cout<<"l : potential field repulse"<<endl;
cout<<"m : potential field track with obstacle"<<endl;
    int timee;
    double x, y, z;
    int c = getch();   // call your non-blocking input function
			
    switch (c)
    {
    case 'y':
    			takeoff();
    			break;
    case 'h':
                        land();
                        break;

    case 'k':
			cout<<"\n keyboard control menu"<<endl;
			keyboard_control();
    case 'x':
			cout<<"\n enter hover time"<<endl;
			cin>>timee;
    			cout<<"\n Hovering in a place"<<endl;
    			hover(timee);
    			break;
	
	case 'f':
                target_follow();
				break;

    case 'a':
                        calibration();
                        break;
    case 'c':
                        pose_measurement();
                        break;

    case 'p':
	                    potential_field_track();
						break;
   
    case 'l':
	           potential_obstacle();
			   break;

	case 'm':
	          potential_track_obstacle();
			  break;

    case 'b':
                        trajectory();                        
                        break;
    default:
    			cout<<endl<<"*********** wrong key! Please choose again! *************"<<endl;
                        cout<<endl;
    			break;
    }
}
land();
cout<<"landed";
}


void calibration(){
ros::Rate loop_rate(10);
double x0,y0,z0,xr0,yr0,zr0,w0;
int ctr = 0;
while(ctr<3){
	x0 = vicon_statedata.transform.translation.x, y0 = vicon_statedata.transform.translation.y, z0 =     vicon_statedata.transform.translation.z;
    xr0 = vicon_statedata.transform.rotation.x, yr0 = vicon_statedata.transform.rotation.y, zr0 = vicon_statedata.transform.rotation.z, w0 = vicon_statedata.transform.rotation.w;
	ctr = ctr+1;
	ros::spinOnce(); 
	loop_rate.sleep();
}

double x,y,z,xr,yr,zr,w;
double px=0, py=0;
double ix=0, iy=0;
double dx=0, dy=0;
double ex=0, ey=0;
double out_x, out_y;
int i=0;
while(ros::ok()){
	x = vicon_statedata.transform.translation.x, y = vicon_statedata.transform.translation.y, z = vicon_statedata.transform.translation.z;
    xr = vicon_statedata.transform.rotation.x, yr = vicon_statedata.transform.rotation.y, zr = vicon_statedata.transform.rotation.z, w = vicon_statedata.transform.rotation.w;
	double pitch, yaw, roll;
    roll = asin(2*(w*yr-zr*xr)); yaw = atan2(2*(w*zr+xr*yr),1-2*(yr*yr+zr*zr)); pitch = atan2(2*(w*xr+yr*zr),1-2*(xr*xr+yr*yr));
	cout<<endl<<" angle : "<<yaw;	
	px=x, py=y;
	ix+=x, iy+=y;
	dx=px-ex, dy=py-ey;
	ex=x, ey=y;
	out_x=0.1*py+0.00004*iy+0.004*dy;//+0.1*dy+0.00001*iy;//+0.0097*iy-0.2037*dy;//+iy/3000;
    out_y=0.1*px+0.00004*ix+0.004*dy;//+0.1*dx+0.00001*ix;//+0.0097*ix+0.2037*dx;//+ix/3000;
    //wait(0.01);
    cout<<endl<<"     "<<"x : "<<-y<<"         "<<"y : "<<-x<<"           "<<"z : "<<z<<endl;
	if(abs(yaw)<0.15)
	{
	move(out_x,-out_y,0,0,0,-yaw);
	i=1;
	}
	if(i!=1)
	{
		move(0,0,0,0,0,-yaw);
	}
	ros::spinOnce();
	loop_rate.sleep();
	}
}
	
void trajectory()
{
	movement(0,0);
	cout<<endl<<"00"<<endl;
	movement(1,1);
	cout<<endl<<"11"<<endl;
	movement(1,-1);
	cout<<endl<<"1-1"<<endl;
	movement(-1,-1);
	cout<<endl<<"-1-1"<<endl;
	movement(-1,1);
	cout<<endl<<"-11"<<endl;
	movement(0,0);
    cout<<endl<<"position done"<<endl;
	land();
	
}

void target_follow(){
ros::Rate loop_rate(10);
double x0,y0,z0,xr0,yr0,zr0,w0;
int ctr = 0;
while(ctr<3){
	x0 = vicon_statedata.transform.translation.x, y0 = vicon_statedata.transform.translation.y, z0 =     vicon_statedata.transform.translation.z;
    xr0 = vicon_statedata.transform.rotation.x, yr0 = vicon_statedata.transform.rotation.y, zr0 = vicon_statedata.transform.rotation.z, w0 = vicon_statedata.transform.rotation.w;
	ctr = ctr+1;
	ros::spinOnce(); 
	loop_rate.sleep();
}

double x,y,z,xr,yr,zr,w,x1,y1,z1;

while(ros::ok()){
	x = vicon_statedata.transform.translation.x, y = vicon_statedata.transform.translation.y, z = vicon_statedata.transform.translation.z;
    xr = vicon_statedata.transform.rotation.x, yr = vicon_statedata.transform.rotation.y, zr = vicon_statedata.transform.rotation.z, w = vicon_statedata.transform.rotation.w;
	double pitch, yaw, roll;
    roll = asin(2*(w*yr-zr*xr)); yaw = atan2(2*(w*zr+xr*yr),1-2*(yr*yr+zr*zr)); pitch = atan2(2*(w*xr+yr*zr),1-2*(xr*xr+yr*yr));
	x1 = vicon_statedata_2.transform.translation.x, y1 = vicon_statedata_2.transform.translation.y, z1 = vicon_statedata_2.transform.translation.z;
    cout<<endl<<"      "<<" object "<<"       "<<"   x   "<<"        "<<"  y  "<<"         "<<"  z  "<<"        "<<" angle "<<endl;
	cout<<"      "<<"ardrone "<<"      "<<-y<<"      "<<-x<<"      "<<z<<"      "<<yaw<<endl;
	cout<<"      "<<"target  "<<"      "<<-y1<<"        "<<-x1<<endl;
	double distance_difference = sqrt((y1-y)*(y1-y)+(x1-x)*(x1-x));
	double destination = distance_difference-2;
	double target_angle;
	target_angle = asin((x1-x)/distance_difference);
	double angle_difference = target_angle-yaw;

		move(0.07*destination,0,1.2*(z1-z),0,0,angle_difference);
	    wait(0.01);		

	ros::spinOnce();
	loop_rate.sleep();
	}
}

void potential_field_track()
{
	ros::Rate loop_rate(10);
    double x0,y0,z0,xr0,yr0,zr0,w0;
    int ctr = 0;
    while(ctr<3){
	x0 = vicon_statedata.transform.translation.x, y0 = vicon_statedata.transform.translation.y, z0 =     vicon_statedata.transform.translation.z;
    xr0 = vicon_statedata.transform.rotation.x, yr0 = vicon_statedata.transform.rotation.y, zr0 = vicon_statedata.transform.rotation.z, w0 = vicon_statedata.transform.rotation.w;
	ctr = ctr+1;
	ros::spinOnce(); 
	loop_rate.sleep();
}

double x,y,z,xr,yr,zr,w,x1,y1,z1;
int i = 0;
while(ros::ok()){
    i = i+1;
	x = vicon_statedata.transform.translation.x, y = vicon_statedata.transform.translation.y, z = vicon_statedata.transform.translation.z;
    xr = vicon_statedata.transform.rotation.x, yr = vicon_statedata.transform.rotation.y, zr = vicon_statedata.transform.rotation.z, w = vicon_statedata.transform.rotation.w;
	double pitch, yaw, roll;
    roll = asin(2*(w*yr-zr*xr)); yaw = atan2(2*(w*zr+xr*yr),1-2*(yr*yr+zr*zr)); pitch = atan2(2*(w*xr+yr*zr),1-2*(xr*xr+yr*yr));
	x1 = vicon_statedata_2.transform.translation.x, y1 = vicon_statedata_2.transform.translation.y, z1 = vicon_statedata_2.transform.translation.z;
	
    cout<<endl<<"      "<<" object "<<"       "<<"   x   "<<"        "<<"  y  "<<"         "<<"  z  "<<"        "<<" angle "<<endl;
	cout<<"      "<<"ardrone "<<"       "<<-y<<"      "<<-x<<"      "<<z<<"      "<<yaw<<endl;
	cout<<"      "<<"target  "<<"       "<<-y1<<"        "<<-x1<<endl;
	
	double speed_difference,speed_target;
	double previous_x,previous_x1,previous_y,previous_y1;
	if(i==1)
	{
		speed_difference=0;
		speed_target=0;
	}
	else{
	    speed_difference = 100*(sqrt((x1-previous_x1)*(x1-previous_x1)+(y1-previous_y1)*(y1-previous_y1))*(previous_y1-y1)/(abs(previous_y1-y1))-sqrt((x-previous_x)*(x-previous_x)+(y-previous_y)*(y-previous_y))*(previous_y1-y1)/(abs(previous_y1-y1)));
		speed_target = 100*(sqrt((x1-previous_x1)*(x1-previous_x1)+(y1-previous_y1)*(y1-previous_y1)))*(previous_y1-y1)/(abs(previous_y1-y1));
	}
    //cout<<endl<<speed_target<<"          "<<speed_difference<<endl;
	 previous_x = x, previous_x1=x1;
	 previous_y = y, previous_y1=y1;
	double distance_difference = sqrt((y1-y)*(y1-y)+(x1-x)*(x1-x));
	double destination = distance_difference-1;
	double p1=0.05;
	double p2=0.003;
	double p3=0.003;
	double desired_speed=p1*destination+p2*speed_target+p3*speed_difference;
	double target_angle = asin((x1-x)/distance_difference);
	if(y1>y)
	{
		target_angle = 3.14-asin((x1-x)/distance_difference);
	}
	if(x1<x&&y1>y)
	{
        target_angle = -3.14-asin((x1-x)/distance_difference);
	}
	double angle_difference = target_angle-yaw;
	
    if(angle_difference>3.14)
	{
        angle_difference=angle_difference-7.28;
	}
	if(angle_difference<-3.14)
	{
		angle_difference=angle_difference+7.28;
	}
	
		move(desired_speed,0,1.2*(z1-z),0,0,angle_difference);
	    wait(0.01);		

	ros::spinOnce();
	loop_rate.sleep();
	}
}

void potential_obstacle()
{
	ros::Rate loop_rate(10);
    double x0,y0,z0,xr0,yr0,zr0,w0;
    int ctr = 0;
    while(ctr<3){
	x0 = vicon_statedata.transform.translation.x, y0 = vicon_statedata.transform.translation.y, z0 =     vicon_statedata.transform.translation.z;
    xr0 = vicon_statedata.transform.rotation.x, yr0 = vicon_statedata.transform.rotation.y, zr0 = vicon_statedata.transform.rotation.z, w0 = vicon_statedata.transform.rotation.w;
	ctr = ctr+1;
	ros::spinOnce(); 
	loop_rate.sleep();
}

double x,y,z,xr,yr,zr,w,x1,y1,z1;
int i = 0;
while(ros::ok()){
    i = i+1;
	x = vicon_statedata.transform.translation.x, y = vicon_statedata.transform.translation.y, z = vicon_statedata.transform.translation.z;
    xr = vicon_statedata.transform.rotation.x, yr = vicon_statedata.transform.rotation.y, zr = vicon_statedata.transform.rotation.z, w = vicon_statedata.transform.rotation.w;
	double pitch, yaw, roll;
    roll = asin(2*(w*yr-zr*xr)); yaw = atan2(2*(w*zr+xr*yr),1-2*(yr*yr+zr*zr)); pitch = atan2(2*(w*xr+yr*zr),1-2*(xr*xr+yr*yr));
	x1 = vicon_statedata_3.transform.translation.x, y1 = vicon_statedata_3.transform.translation.y, z1 = vicon_statedata_3.transform.translation.z;

    cout<<endl<<"      "<<" object "<<"       "<<"   x   "<<"        "<<"  y  "<<"         "<<"  z  "<<"        "<<" angle "<<endl;
	cout<<"      "<<"ardrone "<<"      "<<-y<<"      "<<-x<<"      "<<z<<"      "<<yaw<<endl;
	cout<<"      "<<"obstacle"<<"      "<<-y1<<"        "<<-x1<<endl;
	
	double speed_difference,speed_obstacle;
	double previous_x,previous_x1,previous_y,previous_y1;
	if(i==1)
	{
		speed_difference=0;
		speed_obstacle=0;
	}
	else{
	    speed_difference = 100*(sqrt((x1-previous_x1)*(x1-previous_x1)+(y1-previous_y1)*(y1-previous_y1))*(previous_y1-y1)/(abs(previous_y1-y1))-sqrt((x-previous_x)*(x-previous_x)+(y-previous_y)*(y-previous_y))*(previous_y1-y1)/(abs(previous_y1-y1)));
		speed_obstacle = 100*(sqrt((x1-previous_x1)*(x1-previous_x1)+(y1-previous_y1)*(y1-previous_y1)))*(previous_y1-y1)/(abs(previous_y1-y1));
	}
    
	 previous_x = x, previous_x1=x1;
	 previous_y = y, previous_y1=y1;

	double distance_difference = sqrt((y1-y)*(y1-y)+(x1-x)*(x1-x));
	double target_angle = asin((x1-x)/distance_difference);
    double desired_speed=0.2*(1.5-distance_difference);
	if(distance_difference>1.5)
	{
		desired_speed=0;
	}

	if(y1>y)
	{
		target_angle = 3.14-asin((x1-x)/distance_difference);
	}
	if(x1<x&&y1>y)
	{
        target_angle = -3.14-asin((x1-x)/distance_difference);
	}
	double angle_difference = target_angle-yaw;
	double speed_x=-desired_speed*cos(angle_difference);
	double speed_y=-desired_speed*sin(angle_difference);

		move(speed_x,speed_y,0,0,0,0);
	    wait(0.01);		
	ros::spinOnce();
	loop_rate.sleep();
	}
}

void potential_track_obstacle()
{
	ros::Rate loop_rate(10);
    double x0,y0,z0,xr0,yr0,zr0,w0;
    int ctr = 0;
    while(ctr<3){
	x0 = vicon_statedata.transform.translation.x, y0 = vicon_statedata.transform.translation.y, z0 =     vicon_statedata.transform.translation.z;
    xr0 = vicon_statedata.transform.rotation.x, yr0 = vicon_statedata.transform.rotation.y, zr0 = vicon_statedata.transform.rotation.z, w0 = vicon_statedata.transform.rotation.w;
	ctr = ctr+1;
	ros::spinOnce(); 
	loop_rate.sleep();
}

double x,y,z,xr,yr,zr,w,x1,y1,z1,x2,y2,z2;std_msgs::Empty emp_msg;				// variable in the take off and landing class
geometry_msgs::Twist vel_msg;			// variable in the command velocity class
int i = 0;
while(ros::ok()){
    i = i+1;
	x = vicon_statedata.transform.translation.x, y = vicon_statedata.transform.translation.y, z = vicon_statedata.transform.translation.z;
    xr = vicon_statedata.transform.rotation.x, yr = vicon_statedata.transform.rotation.y, zr = vicon_statedata.transform.rotation.z, w = vicon_statedata.transform.rotation.w;
	double pitch, yaw, roll;
    roll = asin(2*(w*yr-zr*xr)); yaw = atan2(2*(w*zr+xr*yr),1-2*(yr*yr+zr*zr)); pitch = atan2(2*(w*xr+yr*zr),1-2*(xr*xr+yr*yr));
	x1 = vicon_statedata_2.transform.translation.x, y1 = vicon_statedata_2.transform.translation.y, z1 = vicon_statedata_2.transform.translation.z;
	x2 = vicon_statedata_3.transform.translation.x, y2 = vicon_statedata_3.transform.translation.y, z2 = vicon_statedata_3.transform.translation.z;
   
    cout<<endl<<"      "<<" object "<<"       "<<"   x   "<<"        "<<"  y  "<<"         "<<"  z  "<<"        "<<" angle "<<endl;
	cout<<"      "<<"ardrone "<<"      "<<-y<<"      "<<-x<<"      "<<z<<"      "<<yaw<<endl;
	cout<<"      "<<"target  "<<"      "<<-y1<<"       "<<-x1<<endl;
	cout<<"      "<<"obstacle"<<"      "<<-y2<<"      "<<-x2<<endl;
	
	double speed_difference,speed_target;
	double previous_x,previous_x1,previous_y,previous_y1;
	if(i==1)
	{
		speed_difference=0;
		speed_target=0;
	}
	else{
	    speed_difference = 100*(sqrt((x1-previous_x1)*(x1-previous_x1)+(y1-previous_y1)*(y1-previous_y1))*(previous_y1-y1)/(abs(previous_y1-y1))-sqrt((x-previous_x)*(x-previous_x)+(y-previous_y)*(y-previous_y))*(previous_y1-y1)/(abs(previous_y1-y1)));
		speed_target = 100*(sqrt((x1-previous_x1)*(x1-previous_x1)+(y1-previous_y1)*(y1-previous_y1)))*(previous_y1-y1)/(abs(previous_y1-y1));
	}
    
	 previous_x = x, previous_x1=x1;
	 previous_y = y, previous_y1=y1;

	double distance_difference_target = sqrt((y1-y)*(y1-y)+(x1-x)*(x1-x));
	double distance_difference_obstacle = sqrt((y2-y)*(y2-y)+(x2-x)*(x2-x));
	double target_angle = asin((x1-x)/distance_difference_target);
	double obstacle_angle = asin((x2-x)/distance_difference_obstacle);
    double desired_speed_obstacle=0.2*(1.5-distance_difference_obstacle);
	if(distance_difference_obstacle>1.5)
	{
		desired_speed_obstacle=0;
	}

    double destination = distance_difference_target-1;
	double p1=0.05;
	double p2=0.003;
	double p3=0.003;
	double desired_speed_target=p1*destination+p2*speed_target+p3*speed_difference;
    
	if(y1>y)
	{
		target_angle = 3.14-asin((x1-x)/distance_difference_target);
	}
	if(x1<x&&y1>y)
	{
        target_angle = -3.14-asin((x1-x)/distance_difference_target);
	}
	if(y2>y)
	{
		obstacle_angle = 3.14-asin((x2-x)/distance_difference_obstacle);
	}
	if(x2<x&&y2>y)
	{
        obstacle_angle = -3.14-asin((x2-x)/distance_difference_obstacle);
	}

	double angle_difference_target = target_angle-yaw;
	double angle_difference_obstacle = obstacle_angle-yaw;
	double speed_x=-desired_speed_obstacle*cos(angle_difference_obstacle);
	double speed_y=-desired_speed_obstacle*sin(angle_difference_obstacle);
    double desired_speed = desired_speed_target+speed_x;
	if(angle_difference_target>3.14)
	{
        angle_difference_target=angle_difference_target-7.28;
	}
	if(angle_difference_target<-3.14)
	{
		angle_difference_target=angle_difference_target+7.28;
	}
		move(desired_speed,speed_y,1.2*(z1-z),0,0,angle_difference_target);
	    wait(0.01);		
	ros::spinOnce();
	loop_rate.sleep();
	}
}


void pose_measurement(){                       // the current position and angle
ros::Rate loop_rate(10);
double x0,y0,z0,xr,yr,zr,w;
int ctr = 0;
while(ctr<3){
x0 = vicon_statedata.transform.translation.x, y0 = vicon_statedata.transform.translation.y, z0 =     vicon_statedata.transform.translation.z;
xr = vicon_statedata.transform.rotation.x, yr = vicon_statedata.transform.rotation.y, zr = vicon_statedata.transform.rotation.z, w = vicon_statedata.transform.rotation.w;
	ctr = ctr+1;
 ros::spinOnce();
 loop_rate.sleep();
}

double pitch, yaw, roll;
roll = asin(2*(w*yr-zr*xr)); yaw = atan2(2*(w*zr+xr*yr),1-2*(yr*yr+zr*zr)); pitch = atan2(2*(w*xr+yr*zr),1-2*(xr*xr+yr*yr));
cout<<endl<<"x : "<<-y0<<"      "<<"y : "<<-x0<<"      "<<"z : "<<z0<<endl;
cout<<"pitch : "<<pitch<<"       "<<"roll : "<<roll<<"       "<<"yaw : "<<yaw<<endl;
cout<<endl;
}


void keyboard_control(){
ros::Rate loop_rate(10);
int exit_val=0;
cout<<"Press a key"<<endl;
cout<<"y : takeoff           h : land             z : up            c : down         "<<endl;
cout<<"w : forward           s : backward         a : left          d : right        "<<endl;
cout<<"q : yaw(counterClockwise)                  e : yaw(Clockwise)"<<endl;
cout<<"x : exit to the previous menu"<<endl;
cout<<"The drone will move about 1 meter after one key pressed"<<endl;

while(exit_val==0){
int l = getch(); 
int sval=0.5;
double t0 = 0,t1 = 0;
if(l=='z'){
	move(0,0,1,0,0,0);
	wait(1);
    move(0,0,0,0,0,0);}
else if(l=='c'){
	move(0,0,-1,0,0,0);
	wait(1);
    move(0,0,0,0,0,0);}
else if(l=='w'){
	move(1,0,0,0,0,0);
	wait(1);
    move(0,0,0,0,0,0);}
else if(l=='s'){
	move(-1,0,0,0,0,0);
	wait(1);
    move(0,0,0,0,0,0);}
else if(l=='a'){
	move(0,1,0,0,0,0);
	wait(1);
    move(0,0,0,0,0,0);}
else if(l=='d'){
	move(0,-1,0,0,0,0);
	wait(1);
    move(0,0,0,0,0,0);}
else if(l=='q'){
	move(0,0,0,0,0,1);
	wait(1);
    move(0,0,0,0,0,0);}
else if(l=='e'){
	move(0,0,0,0,0,-1);
	wait(1);
    move(0,0,0,0,0,0);}		
else if(l=='x')
exit_val = 1;
else if(l=='y')
takeoff();
else if(l=='h')
land();
else
hover(2);

ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
loop_rate.sleep();
}
}


void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message )
{
	drone_navdata.vx	= 	pose_message->vx;
	drone_navdata.vy 	= 	pose_message->vy;
	drone_navdata.vz 	= 	pose_message->vz;
	drone_navdata.ax	= 	pose_message->ax;
	drone_navdata.ay 	= 	pose_message->ay;
	drone_navdata.az 	= 	pose_message->az;
	drone_navdata.rotX	= 	pose_message->rotX;
	drone_navdata.rotY	= 	pose_message->rotY;
	drone_navdata.rotZ	= 	pose_message->rotZ;
	drone_navdata.magX	= 	pose_message->magX;
	drone_navdata.magY	= 	pose_message->magY;
	drone_navdata.magZ	= 	pose_message->magZ;
	drone_navdata.altd 	= 	pose_message->altd;
	drone_navdata.tm	= 	pose_message->tm;
	drone_navdata.header	= 	pose_message->header;
	drone_navdata.batteryPercent= pose_message->batteryPercent;
}

void ViconStateCallback(const geometry_msgs::TransformStamped::ConstPtr & vicon_state_message)
{
	vicon_statedata.transform.translation.x	= 	vicon_state_message->transform.translation.x;
	vicon_statedata.transform.translation.y	= 	vicon_state_message->transform.translation.y;
	vicon_statedata.transform.translation.z	= 	vicon_state_message->transform.translation.z;
	
	vicon_statedata.transform.rotation.x	= 	vicon_state_message->transform.rotation.x;//quaternion orientation x,y,z,w
	vicon_statedata.transform.rotation.y	= 	vicon_state_message->transform.rotation.y;
	vicon_statedata.transform.rotation.z	= 	vicon_state_message->transform.rotation.z;
	vicon_statedata.transform.rotation.w	= 	vicon_state_message->transform.rotation.w;
	
}

void ViconStateCallback_2(const geometry_msgs::TransformStamped::ConstPtr & vicon_state_message) // for target
{
	vicon_statedata_2.transform.translation.x	= 	vicon_state_message->transform.translation.x;
	vicon_statedata_2.transform.translation.y	= 	vicon_state_message->transform.translation.y;
	vicon_statedata_2.transform.translation.z	= 	vicon_state_message->transform.translation.z;
	
}

void ViconStateCallback_3(const geometry_msgs::TransformStamped::ConstPtr & vicon_state_message) // for obstacle
{
	vicon_statedata_3.transform.translation.x	= 	vicon_state_message->transform.translation.x;
	vicon_statedata_3.transform.translation.y	= 	vicon_state_message->transform.translation.y;
	vicon_statedata_3.transform.translation.z	= 	vicon_state_message->transform.translation.z;
	
}

void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message)
{
	ang.header	= 	imu_message->header;
	ang.angular_velocity	= 	imu_message->angular_velocity;
	ang.linear_acceleration =  imu_message->linear_acceleration;
}

void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message)
{
	lat	= 	gps_message->lat_fused;
	lon 	= 	gps_message->long_fused;
	ele 	= 	gps_message->elevation;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr & joy )
{
	lx = joy->axes[0];
	ly = joy->axes[1];
	az = joy->axes[2];
	lz = joy->axes[3];
	to = joy->buttons[8];
	la = joy->buttons[9];
}

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

double deg2rad(double angle_in_degrees)
{
	return angle_in_degrees*PI/180.0;
}

double getDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
	return sqrt(pow(x1-x2, 2)+pow(y1-y2, 2)+pow(z1-z2, 2));
}

void takeoff()
{
	ros::Rate loop_rate(10);
	while (ros::ok())
		{
			double init_time=ros::Time::now().toSec();  // epoch time
			double time;
			while (time < (init_time+5.0)) /* Send command for five seconds*/
			{
				T_pub_empty.publish(emp_msg); /* launches the drone */
				ros::spinOnce(); // feedback
				loop_rate.sleep();
				time = ros::Time::now().toSec();
			}//time loop
		exit(0);
		}//ros::ok loop
}

void land()
{
	ros::Rate loop_rate(10);
	while (ros::ok())
		{
			double init_time=ros::Time::now().toSec();
			double time;
			while (time < (init_time+5.0)) /* Send command for five seconds*/
			{
				L_pub_empty.publish(emp_msg); /* lands the drone */
				ros::spinOnce();
				loop_rate.sleep();
				time = ros::Time::now().toSec();
			}//time loop
		exit(0);
		}//ros::ok loop

}

void hover(int timee)
{

	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'
	double t1;

	ros::Rate loop_rate(200);

	do{

		t1 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

		vel_msg.linear.x = 0;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 0;

	velocity_publisher.publish(vel_msg);

	ros::spinOnce();
	loop_rate.sleep();

	}while(t1 <= (t0+timee));

	//ros::spinOnce();
}

void wait(double x)
{
   double t0,t1;
   t0 = ros::Time::now().toSec();
   do{
         t1 = ros::Time::now().toSec();
     }while(t1<(t0+x));
}

void move(float lx, float ly, float lz, float ax, float ay, float az )
{

	//defining the linear velocity
	vel_msg.linear.x = lx;
	vel_msg.linear.y = ly;
	vel_msg.linear.z = lz;

	//defining the linear velocity
	vel_msg.angular.x = ax;
	vel_msg.angular.y = ay;
	vel_msg.angular.z = az;

	velocity_publisher.publish(vel_msg);

}

void movement(double goal_x, double goal_y)
{
	ros::Rate loop_rate(10);
    double x0,y0,z0,xr0,yr0,zr0,w0;
    int ctr = 0;
    while(ctr<3){
	x0 = vicon_statedata.transform.translation.x, y0 = vicon_statedata.transform.translation.y, z0 =     vicon_statedata.transform.translation.z;
    xr0 = vicon_statedata.transform.rotation.x, yr0 = vicon_statedata.transform.rotation.y, zr0 = vicon_statedata.transform.rotation.z, w0 = vicon_statedata.transform.rotation.w;
	ctr = ctr+1;
	ros::spinOnce(); 
	loop_rate.sleep();
}

double x,y,z,xr,yr,zr,w;
double px=0, py=0;
double ix=0, iy=0;
double dx=0, dy=0;
double ex=0, ey=0;
double out_x, out_y;
int i=0;
while(ros::ok()){
	x = vicon_statedata.transform.translation.x, y = vicon_statedata.transform.translation.y, z = vicon_statedata.transform.translation.z;
    xr = vicon_statedata.transform.rotation.x, yr = vicon_statedata.transform.rotation.y, zr = vicon_statedata.transform.rotation.z, w = vicon_statedata.transform.rotation.w;
	double pitch, yaw, roll;
    roll = asin(2*(w*yr-zr*xr)); yaw = atan2(2*(w*zr+xr*yr),1-2*(yr*yr+zr*zr)); pitch = atan2(2*(w*xr+yr*zr),1-2*(xr*xr+yr*yr));
	cout<<endl<<" angle : "<<yaw;	
	cout<<endl<<"     "<<"x : "<<-y<<"         "<<"y : "<<-x<<"           "<<"z : "<<z<<endl;
	px=x-goal_x, py=y-goal_y;
	ix+=px, iy+=py;
	dx=px-ex, dy=py-ey;
	ex=x-goal_x, ey=y-goal_y;
	out_x=0.1*py;//+0.004*dy+0.00004*iy;//+0.0097*iy-0.2037*dy;//+iy/3000;
    out_y=0.1*px;//+0.004*dx+0.00004*ix;//+0.0097*ix+0.2037*dx;//+ix/3000;
	if(abs(yaw)<0.15)
	{
		move(out_x,-out_y,0,0,0,-yaw);
		i=1;
		/*
		if(abs(px)<0.2&&abs(py)<0.2)
		{
			move(0,0,0,0,0,0);
			break;
		}
		*/
		
	}
	if(i!=1){
		move(0,0,0,0,0,-yaw);
	}
    if(abs(yaw)<0.15&&abs(px)<0.2&&abs(py)<0.2)
	{
		move(0,0,0,0,0,0);
		cout<<endl<<"finished"<<endl;
		break;
	}
	ros::spinOnce();
	loop_rate.sleep();
	}
}





