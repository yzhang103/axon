/*
//Title: i-Building for AXON Robot (Include Speed, Encoder load) (IoT Lab.)
//Author: Chen, Chun-Lin
//Data: 2015/12/03
//Update:
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cereal_port/CerealPort.h>

#define REPLY_SIZE 1
#define TIMEOUT 1000
#define TX_ONE_BYTE_DELAY 0.5
#define TX_LEN 3

#define PI           3.14159
#define RTD          180.0/PI       //Radian to Degree
#define DTR          PI/180.0       //Degree to Radian
#define TS 0.1                      //unit:s

#define AXON_ROBOT_R 0.1075         //unit:m
#define AXON_ROBOT_L 0.2680         //unit:m
#define AXON_WHEEL_MINSPEED 0.0065 //(rad/s)
#define AXON_WHEEL_CMD_LIMIT 45

#define GETSTR_PORT "/axon_parameter/SerialPort"
#define GETSTR_RATE "/axon_parameter/BaudRate"

cereal::CerealPort device;
char reply[REPLY_SIZE];
int cmd_type=0;
unsigned char rx_buff[8];
int rx_count=0;
char rx_timeout_flag=0;
int rx_timeout_cnt=0;
int AXON_cmd_send_flag=0;

int left_ec=0;
int right_ec=0;

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "axon_link_node");
    ros::NodeHandle n;

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

    ros::Rate r(120);
    while(ros::ok())
    {
        //Send Command
        if (rx_timeout_cnt<30)
        {
            AXON_Command_Send(1,10,10);
            AXON_Command_Send(3,0,0);
        }

        //loop----------------------------------------------------------
        rx_timeout_flag=0;
        // Get the reply, the last value is the timeout in ms
        try{ device.read(reply, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            //ROS_ERROR("Timeout!");
            rx_timeout_flag=1;
            if (rx_timeout_cnt>=1)
            {
                rx_timeout_cnt--;
                if (rx_timeout_cnt==0)
                    rx_timeout_cnt=0;
            }
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
                    printf("L_Encoder=%d, R_Encoder=%d\n" , left_ec, right_ec);
                }
                rx_count=0;
            }
        }
        rx_timeout_cnt++;

        //loop----------------------------------------------------------
        ros::spinOnce();
        r.sleep();
    }
}
