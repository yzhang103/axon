/*
//Title: i-Building for AXON Robot (IoT Lab.)
//Author: Chen, Chun-Lin
//Data: 2015/01/07
//Update: 2015/02/04
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
        for (i=0;i<TX_LEN;i++)
        {
            b[0]=(char)a[i];
            b[1]='\0';
            device.write(b,1);  //Send one byte over the serial port
            //ros::Duration d= ros::Duration(TX_ONE_BYTE_DELAY); //ROS delay function (ICL must delay between byte by byte)
        }
        AXON_cmd_send_flag=1;
        //ROS_INFO("I Send:cmd=%d, P1=%d, P2=%d", a[0], a[1], a[2]);
    }
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
        AXON_Command_Send(1,10,10);

        unsigned int aa=0,bb=0;
        unsigned int dd;
        char cc1[2];
        char cc2[2];
        char cc3;

        char b[2];
        unsigned char a[TX_LEN];
        a[0]=3;
        b[0]=(char)a[0];
        b[1]='\0';

        device.write(b,1);  //Send one byte over the serial port
        //loop----------------------------------------------------------
        rx_timeout_flag=0;
        // Get the reply, the last value is the timeout in ms
        try{ device.read(reply, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            //ROS_ERROR("Timeout!");
            rx_timeout_flag=1;
        }

        if (rx_timeout_flag==0)
        {
            //printf("Got this reply: %s\n", reply);
            if (reply[0]<0)
                dd=reply[0]+255;
            else
                dd=reply[0];
            aa=dd/16;
            bb=dd%16;

            if (aa>=10)
                aa=aa+'A'-10;
            else
                aa=aa+'0';
            if (bb>=10)
                bb=bb+'A'-10;
            else
                bb=bb+'0';

            cc1[0]=aa;
            cc1[1]='\0';
            cc2[0]=bb;
            cc2[1]='\0';
            //printf("Got aa=%d, bb=%d, dd=%d, cc1=%s, cc2=%s\n", aa, bb, dd, cc1, cc2);
            //printf("Got aa=%d, bb=%d, dd=%d\n", aa, bb, dd);
            printf("Got 0x%s%s\n", cc1, cc2);
        }
        //loop----------------------------------------------------------
        ros::spinOnce();
        r.sleep();
    }
}
