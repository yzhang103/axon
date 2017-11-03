/*
//Title: i-Building for AXON Robot (Include Speed, Encoder load) (IoT Lab.)
//Author: Chen, Chun-Lin
//Data: 2015/12/03
//Update: 2015/12/04
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cereal_port/CerealPort.h>

#define REPLY_SIZE 1
#define TIMEOUT 10
#define TX_ONE_BYTE_DELAY 0.5
#define TX_LEN 3

#define PI           3.14159
#define RTD          180.0/PI           //Radian to Degree
#define DTR          PI/180.0           //Degree to Radian
#define TS 0.1                          //unit:s

#define AXON_ROBOT_R 0.1075             //unit:m
//#define AXON_ROBOT_L 0.2680           //unit:m
#define AXON_ROBOT_L 0.15               //unit:m
#define AXON_WHEEL_MINSPEED 0.0065      //(rad/s)
#define AXON_WHEEL_CMD_LIMIT 45

#define ONE_R_ENCODER_PLUES     51200.0                    //unit:pluse
#define RAD_PLUSE               PI/ONE_R_ENCODER_PLUES     //unit:rad/pluse

#define GETSTR_PORT "/axon_parameter/SerialPort"
#define GETSTR_RATE "/axon_parameter/BaudRate"

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

ros::Subscriber axon_info_sub_;
//Publish the AXON info. over ROS
ros::Publisher axon_info_pub_;

cereal::CerealPort device;
char reply[REPLY_SIZE];
int cmd_type=0;
unsigned char rx_buff[8];
int rx_count=0;
char rx_timeout_flag=0;
int rx_timeout_cnt=0;
int AXON_cmd_send_flag=0;
int cmd_vel_hold_cnt=0;

int left_ec=0;
int right_ec=0;

double total_time=0;
int rx_stop_flag=0;
int spin_cnt=0;
int tx_cmd_send_cnt=0;

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

        //ROS_INFO("w_ui=%d",w_ui);
        Sat_Value_ui(&w_ui, AXON_WHEEL_CMD_LIMIT, 0);
        //ROS_INFO("w_ui=%d",w_ui);

        /*
        if (rc.w_cmd==0)
        {
            if ( (w_ui!=0) && (w_ui<=10)  && (w_ui>0) )
                w_ui=10;
            if ( (w_ui!=0) && (w_ui>=-10) && (w_ui<0) )
                w_ui=-10;
        }
        */

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
        //ROS_INFO("I Send:cmd=%d, P1=%d, P2=%d", a[0], a[1], a[2]);
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

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    rc.v_cmd = cmd_vel->linear.x;
    rc.w_cmd = cmd_vel->angular.z;
    //ROS_INFO("I heard: [v_cmd=%2.3f]", rc.v_cmd);
    //ROS_INFO("I heard: [w_cmd=%2.3f]", rc.w_cmd);

    //set VW to set WLR (unit: rad/s)
    rc.w_L= (1/rm.r)*rc.v_cmd - (rm.L/rm.r)*rc.w_cmd;
    rc.w_R= (1/rm.r)*rc.v_cmd + (rm.L/rm.r)*rc.w_cmd;

    //set WLR to AXON motor speed command
    //rc.w_La=AXON_Wheel_Motor_Speed(rc.w_L);
    //rc.w_Ra=AXON_Wheel_Motor_Speed(rc.w_R);

    //rc.w_La=(int)rc.v_cmd;
    //rc.w_Ra=(int)rc.v_cmd;

    //ROS_INFO("I heard: [v_cmd=%2.4f w_cmd=%2.4f w_L=%2.4f w_R=%2.4f]", rc.v_cmd, rc.w_cmd, rc.w_L, rc.w_R);
    //ROS_INFO("I heard: [w_La=%d w_Ra=%d]", rc.w_La, rc.w_Ra);

    //Send Command to AXON
    //AXON_Command_Send(1, rc.w_La, rc.w_Ra);
    cmd_vel_hold_cnt=0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "axon_link_node");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub_;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    cmd_vel_sub_  = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, cmdVelReceived);

    rm.L=AXON_ROBOT_L;
    rm.r=AXON_ROBOT_R;
    rc.ts=TS;
    rc.w_L=10;
    rc.w_R=10;
    rc.avg_w_Ls=0;
    rc.avg_w_Rs=0;

    int i;
    int baud_rate=0;
    std::string serial_port;

    n.getParam(GETSTR_PORT, serial_port);
    ROS_INFO("I get SerialPort: %s", serial_port.c_str());
    n.getParam(GETSTR_RATE, baud_rate);
    ROS_INFO("I get BaudRate: %d", baud_rate);

    // Change the next line according to your port name and baud rate
    //try{ device.open("/dev/ttyUSB0", 115200); }
    try{ device.open(serial_port.c_str(), baud_rate); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the AXON serial port!!!");
        ROS_BREAK();
    }
    ROS_INFO("The AXON serial port is opened.");
    rx_stop_flag=0;

    ros::Rate r(120);
    while(ros::ok())
    {
        //printf("rx_stop_flag=%d, rx_timeout_cnt=%d\n", rx_stop_flag, rx_timeout_cnt);
        //Send Command
        if (rx_stop_flag==0)
        {
            tx_cmd_send_cnt++;
            if (tx_cmd_send_cnt>=10)
            {
                rc.w_La=AXON_Wheel_Motor_Speed(rc.w_L);
                rc.w_Ra=AXON_Wheel_Motor_Speed(rc.w_R);
                //printf("[w_La=%d w_Ra=%d]\n", rc.w_La, rc.w_Ra);
                AXON_Command_Send(1, rc.w_La, rc.w_Ra);
                //AXON_Command_Send(1, 45, 45);
                AXON_Command_Send(3, 0, 0);

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

        //loop----------------------------------------------------------
        if (rx_stop_flag==0)
        {

        rx_timeout_flag=0;
        // Get the reply, the last value is the timeout in ms
        try
        {
            device.read(reply, REPLY_SIZE, TIMEOUT);
            //rx_count++;
        }
        catch(cereal::TimeoutException& e)
        {
            //ROS_ERROR("Timeout!");
            rx_timeout_flag=1;
            /*
            if (rx_timeout_cnt>=1)
            {
                rx_timeout_cnt--;
                if (rx_timeout_cnt==0)
                    rx_timeout_cnt=0;
            }
            */
        }

        if (rx_timeout_flag==0)
        {
            //Print_HEXString(reply[0]);
            //printf("\n");
        }

        //=================================================================
        //Packet decode
        if (rx_timeout_flag==0)
        {
            unsigned int num;
            if (reply[0]<0)
                num=reply[0]+256;
            else
                num=reply[0];
            rx_buff[rx_count]=(unsigned char)num;
            rx_count++;
            if (rx_count>=11)//CMD1 reponse (2bytes) + CMD3 reponse (9 bytes)
            {
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

                    left_ec=(int)(rx_buff[3]<<24);
                    left_ec=left_ec+(int)(rx_buff[4]<<16);
                    left_ec=left_ec+(int)(rx_buff[5]<<8);
                    left_ec=left_ec+(int)(rx_buff[6]);

                    right_ec=(int)(rx_buff[7]<<24);
                    right_ec=left_ec+(int)(rx_buff[8]<<16);
                    right_ec=left_ec+(int)(rx_buff[9]<<8);
                    right_ec=left_ec+(int)(rx_buff[10]);
                    printf("L_Encoder=%d, R_Encoder=%d, rc.w_L=%+3.3f, rc.w_R=%+3.3f\n" , left_ec, right_ec, rc.w_L, rc.w_R);

                    //-----------------------------------------------------
                    current_time = ros::Time::now();
                    double dt = (current_time - last_time).toSec();
                    total_time+=dt;
                    rc.w_Ls=(left_ec*RAD_PLUSE)/dt;
                    rc.w_Rs=(right_ec*RAD_PLUSE)/dt;
                    rc.avg_w_Ls=(rc.avg_w_Ls+rc.w_Ls)/2.0;
                    rc.avg_w_Rs=(rc.avg_w_Rs+rc.w_Rs)/2.0;

                    printf("Ttime=%+3.3f, LWSp=%+3.3f, R_WSp=%+3.3f, AvgLWSp=%+3.3f, AvgR_WSp=%+3.3f, dt=%+3.3f\n" , total_time, rc.avg_w_Ls, rc.avg_w_Rs, rc.w_Ls, rc.w_Rs, dt);
                    last_time = current_time;
                }
                rx_timeout_cnt++;
                if (rx_timeout_cnt>5)
                {
                    rx_timeout_cnt=100;
                    rx_stop_flag=1;
                }

                rx_count=0;
            }
            rx_timeout_flag=1;
        }

        }
        //loop----------------------------------------------------------
        ros::spinOnce();
        r.sleep();
    }
}
