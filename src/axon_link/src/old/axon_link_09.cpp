/*
//Title: i-Building for AXON Robot (Include Speed, Encoder load) (IoT Lab.)
//Author: Chen, Chun-Lin
//Data: 2015/12/03
//Update: 2016/01/11
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <cereal_port/CerealPort.h>
#include <sensor_msgs/Imu.h>
using namespace std;

//ROS Parameter
#define MAIN_FREQ           120.0       //Controller main frequency
#define TF_REV_DELAY_T      150         //TF of MAP_ODOM frame receive start delay units * ts
#define USE_REV_ENCODE      1

#define REPLY_SIZE 1
#define TIMEOUT 10
#define TX_ONE_BYTE_DELAY 0.5
#define TX_LEN 3

#define PI           3.14159
#define RTD          180.0/PI           //Radian to Degree
#define DTR          PI/180.0           //Degree to Radian
#define TS 0.1                          //unit:s

#define AXON_ROBOT_R 0.1075             //unit:m
#define AXON_ROBOT_L 0.2680             //unit:m
#define AXON_WHEEL_MINSPEED 0.0065      //(rad/s)
#define AXON_WHEEL_CMD_LIMIT 45

#define ONE_R_ENCODER_PLUES     51200.0                    //unit:pluse
#define RAD_PLUSE               PI/ONE_R_ENCODER_PLUES     //unit:rad/pluse

#define GETSTR_PORT "/axon_parameter/SerialPort"
#define GETSTR_RATE "/axon_parameter/BaudRate"

//------------------------------------------
double main_freq_=MAIN_FREQ;
double tf_rev_delay_t_=TF_REV_DELAY_T;
int use_rev_encode_=USE_REV_ENCODE;

typedef struct
{
    double roll;
    double pitch;
    double yaw;
} Pose;

typedef struct
{
    double L;
    double r;
    double h;
} rbtmdl;

typedef struct
{
    double ts;

    double w_L;
    double w_R;

    double w_Ls;
    double w_Rs;

    double avg_w_Ls;
    double avg_w_Rs;

    double v_L;
    double v_R;

    unsigned char v_Ls;
    unsigned char v_Rs;

    double v_t;
    double w_t;

    double v_cmd;
    double w_cmd;

    unsigned char w_La;     //to AXON Left wheel 1 byte command
    unsigned char w_Ra;     //to AXON Right wheel 1 byte command
} rbtctrl;
rbtctrl rc;
rbtmdl rm;

//----------------------------------------------------------
ros::Subscriber axon_info_sub_;
//Publish the AXON info. over ROS
ros::Time current_time, last_time, odom_current_time, odom_last_time;
ros::Publisher axon_info_pub_;
ros::Subscriber IMU_sub_;
ros::Subscriber cmd_vel_sub_;
ros::Publisher odom_pub_;

int baud_rate=38400;
std::string serial_port;
cereal::CerealPort device;
char reply[REPLY_SIZE];
int cmd_type=0;
unsigned char rx_buff[8];
int rx_count=0;
char rx_timeout_flag=0;
int rx_timeout_cnt=0;
int AXON_cmd_send_flag=0;
int cmd_vel_hold_cnt=0;

unsigned long int left_ec=0;
unsigned long int right_ec=0;

double total_time=0;
int rx_stop_flag=0;
int spin_cnt=0;
int tx_cmd_send_cnt=0;
int rx_ec_flag=0;

double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

sensor_msgs::Imu imu_val;
int imu_rev_flag=0;
Pose imu_pose;

int init_yaw_rev_flag=0;
double init_yaw=0.0;

//Quaternion to All Euler angle
Pose Quat_to_AllEuler(geometry_msgs::Quaternion msg_quat)
{
    tf::Quaternion tf_quat;
    Pose pose;
    tf::quaternionMsgToTF(msg_quat, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(pose.roll, pose.pitch, pose.yaw);
    return pose;
}

void Sat_Value_uc(unsigned char *In, unsigned char Upp, unsigned char Low)
{
    unsigned char data;
    data=*In;
    if (data>Upp) data=Upp;
    if (data<Low) data=Low;
    *In=data;
}

void Sat_Value_ui(unsigned int *In, unsigned int Upp, unsigned int Low)
{
    unsigned int data;
    data=*In;
    if (data>Upp) data=Upp;
    if (data<Low) data=Low;
    *In=data;
}

unsigned char AXON_Wheel_Motor_Speed(double w_X)
{
    int neg_flag=0;
    unsigned int w_ui;
    unsigned char w_Xa;

    if (w_X==0)
        w_Xa=0;
    else
    {
        if (w_X<0)
        {
            neg_flag=1;
            w_X=-w_X;
        }

        if (w_X>=AXON_WHEEL_MINSPEED) //min Speed (rad/s)
            //w_ui=(unsigned int)(12.519* pow(w_X, 0.5014) );
            w_ui=(unsigned int)(16.622*w_X + 0.1538 );
        else
            w_Xa=0;

        Sat_Value_ui(&w_ui, AXON_WHEEL_CMD_LIMIT, 0);

        if (neg_flag==1)
            w_Xa=(unsigned char)(w_ui+127);
        else
            w_Xa=(unsigned char)w_ui;
    }

    return w_Xa;
}

void AXON_Command_Send(unsigned char cmd, unsigned char param1, unsigned char param2)
{
    int i;
    unsigned char a[TX_LEN];
    char b[2];

    a[0]=cmd;
    a[1]=param1;
    a[2]=param2;

    //Protection Mechanisms
    //Sat_Value_uc(&a[1], AXON_WHEEL_CMD_LIMIT, 0);
    //Sat_Value_uc(&a[2], AXON_WHEEL_CMD_LIMIT, 0);

    if ( (cmd>0) && (cmd<4) )
    {
        if (cmd==1)//3 bytes
        {
            for (i=0;i<TX_LEN;i++)
            {
                b[0]=(char)a[i];
                b[1]='\0';
                device.write(b,1);  //Send one byte over the serial port
                //ros::Duration d= ros::Duration(TX_ONE_BYTE_DELAY); //ROS delay function (ICL must delay between byte by byte)
            }
        }
        if (cmd==3)//1 bytes
        {
            b[0]=(char)a[0];
            b[1]='\0';
            device.write(b,1);  //Send one byte over the serial port
        }
        AXON_cmd_send_flag=1;
    }
}

void Print_HEXString(char s)
{
    unsigned int num;
    unsigned int num1,num2;
    char* str;
    char str1[3];
    if (s<0)
        num=s+256;
    else
        num=s;
    num1=num/16;
    num2=num%16;

    if (num1>=10)
        num1=num1+'A'-10;
    else
        num1=num1+'0';
    if (num2>=10)
        num2=num2+'A'-10;
    else
        num2=num2+'0';

    str1[0]=num1;
    str1[1]=num2;
    str1[2]='\0';
    str=str1;
    printf("0x%s ", str);
}

void AXON_LRWheel_Setting(void)
{
    rc.w_La=AXON_Wheel_Motor_Speed(rc.w_L);
    rc.w_Ra=AXON_Wheel_Motor_Speed(rc.w_R);
    //printf("[w_La=%d w_Ra=%d]\n", rc.w_La, rc.w_Ra);
    AXON_Command_Send(1, rc.w_La, rc.w_Ra);
}

void AXON_LRWheel_Encode_Setting(void)
{
    rc.w_La=AXON_Wheel_Motor_Speed(rc.w_L);
    rc.w_Ra=AXON_Wheel_Motor_Speed(rc.w_R);
    //printf("[w_La=%d w_Ra=%d]\n", rc.w_La, rc.w_Ra);
    AXON_Command_Send(1, rc.w_La, rc.w_Ra);
    AXON_Command_Send(3, 0, 0);
}

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    rc.v_cmd = cmd_vel->linear.x;
    rc.w_cmd = cmd_vel->angular.z;
    //set VW to set WLR (unit: rad/s)
    rc.w_L= (1/rm.r)*rc.v_cmd - (rm.L/rm.r)*rc.w_cmd;
    rc.w_R= (1/rm.r)*rc.v_cmd + (rm.L/rm.r)*rc.w_cmd;

    if (use_rev_encode_==0)
        AXON_LRWheel_Setting();
    cmd_vel_hold_cnt=0;
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_data)
{
    imu_val.orientation=imu_data->orientation;
    imu_pose=Quat_to_AllEuler(imu_val.orientation);
    imu_val.angular_velocity=imu_data->angular_velocity;
    imu_val.linear_acceleration=imu_data->linear_acceleration;
    imu_rev_flag=1;
    if (init_yaw_rev_flag==0)
    {
        init_yaw=imu_pose.yaw;
        printf("Get Yaw value from IMU~~\n");
    }
    init_yaw_rev_flag=1;
}

void Send_Speed_Encode_Command(void)
{
    //if (init_yaw_rev_flag==0)
        //printf("Wait to get first time Yaw value from IMU~~\n");

    //if ( (rx_stop_flag==0) && (init_yaw_rev_flag==1) )
    if (rx_stop_flag==0)
    {
        tx_cmd_send_cnt++;
        if (tx_cmd_send_cnt>=10)
        {
            //set WLR to AXON motor speed command
            AXON_LRWheel_Encode_Setting();

            cmd_vel_hold_cnt++;
            if (cmd_vel_hold_cnt>=240)
            {
                rc.w_L=0;
                rc.w_R=0;
                cmd_vel_hold_cnt=0;
            }
            tx_cmd_send_cnt=0;
        }
    }
    else
    {
        if (rx_timeout_cnt>=1)
        {
            rx_timeout_cnt--;
            if (rx_timeout_cnt==0)
            {
                rx_stop_flag=0;
                rx_timeout_cnt=0;
                rx_count=0;
                device.flush(); //clear device rx buff
                device.close();
                try{ device.open(serial_port.c_str(), baud_rate); }
                catch(cereal::Exception& e)
                {
                    ROS_FATAL("Failed to open the AXON serial port!!!");
                    ROS_BREAK();
                }

            }
        }
    }
}

void Packet_Decode(void)
{
    int i,j;
    if (rx_stop_flag==0)
    {
        rx_timeout_flag=0;
        // Get the reply, the last value is the timeout in ms
        try{ device.read(reply, REPLY_SIZE, TIMEOUT);}
        catch(cereal::TimeoutException& e)
        {
            //ROS_ERROR("Timeout!");
            rx_timeout_flag=1;
        }

        //=================================================================
        //Packet decode
        if (rx_timeout_flag==0)
        {
            unsigned int num;
            unsigned char reg_srt;
            if (reply[0]<0)
                num=reply[0]+256;
            else
                num=reply[0];
            rx_buff[rx_count]=(unsigned char)num;
            rx_count++;
            if (rx_count>=11)//CMD1 reponse (2bytes) + CMD3 reponse (9 bytes)
            {
                //--Shift rx_buff to HEX: 0x01 0x01 0x03-------------------
                for(i=0;i<(11-1);i++)
                {
                    if ( (rx_buff[0]==0x01) && (rx_buff[1]==0x01) && (rx_buff[2]==0x03) )
                    {
                        break;
                    }
                    else
                    {
                        reg_srt=rx_buff[0];
                        for(j=0;j<(11-1);j++)
                            rx_buff[j]=rx_buff[j+1];
                        rx_buff[11-1]=reg_srt;
                    }
                }

                //Print all reponse
                printf("Got Hex:");
                for(i=0;i<11;i++)
                    Print_HEXString(rx_buff[i]);
                printf("\n");

                //--decode 11 bytes----------------------------------------
                if ( (rx_buff[0]==0x01) && (rx_buff[1]==0x01) && (rx_buff[2]==0x03) )
                {
                    printf("AXON Motor Speed setting & Encoder loading Success~~\n");
                    rx_timeout_cnt=0;
                    left_ec=0;
                    right_ec=0;

                    left_ec=(unsigned long int)(rx_buff[3]<<24);
                    left_ec=left_ec+(unsigned long int)(rx_buff[4]<<16);
                    left_ec=left_ec+(unsigned long int)(rx_buff[5]<<8);
                    left_ec=left_ec+(unsigned long int)(rx_buff[6]);

                    right_ec=(unsigned long int)(rx_buff[7]<<24);
                    right_ec=left_ec+(unsigned long int)(rx_buff[8]<<16);
                    right_ec=left_ec+(unsigned long int)(rx_buff[9]<<8);
                    right_ec=left_ec+(unsigned long int)(rx_buff[10]);

                    /*
                    if (left_ec>32768)
                        left_ec=left_ec-0x10000;
                    if (right_ec>32768)
                        right_ec=right_ec-0x10000;
                    */
                    printf("L_Encoder=%d, R_Encoder=%d, rc.w_L=%+3.3f, rc.w_R=%+3.3f\n" , left_ec, right_ec, rc.w_L, rc.w_R);


                    //--Calculation two wheel-------------------------------------
                    current_time = ros::Time::now();
                    double dt = (current_time - last_time).toSec();
                    total_time+=dt;
                    rc.w_Ls=(left_ec*RAD_PLUSE)/dt;
                    rc.w_Rs=(right_ec*RAD_PLUSE)/dt;
                    rc.avg_w_Ls=(rc.avg_w_Ls+rc.w_Ls)/2.0;
                    rc.avg_w_Rs=(rc.avg_w_Rs+rc.w_Rs)/2.0;
                    rx_ec_flag=1;   //renew encoder (Wheelspeed) data

                    printf("Ttime=%+3.3f, LWSp=%+3.3f, R_WSp=%+3.3f, AvgLWSp=%+3.3f, AvgR_WSp=%+3.3f, dt=%+3.3f\n" , total_time, rc.w_Ls, rc.w_Rs, rc.avg_w_Ls, rc.avg_w_Rs, dt);
                    last_time = current_time;
                }
                rx_timeout_cnt++;
                if (rx_timeout_cnt>5)
                {
                    rx_timeout_cnt=100;
                    rx_stop_flag=1;
                }

                rx_count=0;
            } //end if if (rx_count>=11)
            rx_timeout_flag=1;
        } //end if (rx_timeout_flag==0)
    }//end if (rx_stop_flag==0)
}

/*
void Calculation_Odometry(void)
{
}
*/

int main(int argc, char** argv)
{    
    ros::init(argc, argv, "axon_link_node");
    ros::NodeHandle n;

    tf::TransformBroadcaster odom_broadcaster;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    odom_current_time = ros::Time::now();
    odom_last_time = ros::Time::now();

    odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 50);
    cmd_vel_sub_= n.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, cmdVelReceived);
    IMU_sub_= n.subscribe("/mavros/imu/data", 10, ImuCallback);

    //==================================================
    //load ROS Parameter
    n.getParam("main_freg", main_freq_);
    n.getParam("tf_rev_delay_t", tf_rev_delay_t_);
    n.getParam("use_rev_encode", use_rev_encode_);
    n.getParam(GETSTR_PORT, serial_port);
    ROS_INFO("I get SerialPort: %s", serial_port.c_str());
    n.getParam(GETSTR_RATE, baud_rate);
    ROS_INFO("I get BaudRate: %d", baud_rate);

    rm.L=AXON_ROBOT_L;
    rm.r=AXON_ROBOT_R;
    rc.ts=TS;
    rc.w_L=0.5;
    rc.w_R=0.5;
    rc.avg_w_Ls=0;
    rc.avg_w_Rs=0;

    try{ device.open(serial_port.c_str(), baud_rate); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the AXON serial port!!!");
        ROS_BREAK();
    }
    ROS_INFO("The AXON serial port is opened.");
    rx_stop_flag=0;

    ros::Rate r(main_freq_);
    while(ros::ok())
    {
        cmd_vel_hold_cnt=0;
        rc.w_L=-10;
        rc.w_R=-10;

        //loop----------------------------------------------------------
        if (use_rev_encode_==1)
        {
            Send_Speed_Encode_Command();

            Packet_Decode();
            ///Calculation_Odometry();
            //===================================
            //if ( (rx_ec_flag==1) && (imu_rev_flag==1) )
            if (rx_ec_flag==1)
            {
                odom_current_time = ros::Time::now();
                double odom_dt = (odom_current_time - odom_last_time).toSec();
                //--------------------------------------------------------------------
                //Robot Unicycle Model
                rc.v_L=(rm.r/2.0) *(rc.w_Rs+rc.w_Ls); //(From two wheel velocity)
                rc.w_L=(rm.r/rm.L)*(rc.w_Rs-rc.w_Ls); //(From two wheel velocity)
                //if (rc.w_L>=0.02)
                    //th=imu_pose.yaw;

                vx=rc.v_L*cos(th);
                vy=rc.v_L*sin(th);
                vth=rc.w_L;
                //compute odometry in a typical way given the velocities of the robot
                double delta_x = vx*odom_dt;
                double delta_y = vy*odom_dt;
                double delta_th = vth*odom_dt;
                x += delta_x;
                y += delta_y;
                th += delta_th;

                //since all odometry is 6DOF we'll need a quaternion created from yaw
                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
                //first, we'll publish the transform over tf
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_link";
                odom_trans.transform.translation.x = x;
                odom_trans.transform.translation.y = y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom_quat;
                //send the transform
                odom_broadcaster.sendTransform(odom_trans);

                //next, we'll publish the odometry message over ROS
                nav_msgs::Odometry odom;
                odom.header.stamp = current_time;
                odom.header.frame_id = "odom";
                //set the position
                odom.pose.pose.position.x = x;
                odom.pose.pose.position.y = y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;
                //set the velocity
                odom.child_frame_id = "base_link";
                odom.twist.twist.linear.x = vx;
                odom.twist.twist.linear.y = vy;
                odom.twist.twist.angular.z = vth;
                //publish the message
                odom_pub_.publish(odom);

                odom_last_time = odom_current_time;

                printf("x=%+3.3f, y=%+3.3f, th=%+3.3f, Yaw=%+3.3f\n",x ,y, th, imu_pose.yaw);

                rx_ec_flag=0;
                imu_rev_flag=0;
            }//end if (rx_ec_flag==1)
        }
        //loop----------------------------------------------------------
        ros::spinOnce();
        r.sleep();
    }
}
