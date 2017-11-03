/*
//Title: DELTA Corp. Lab. for AXON Robot (IoT Lab.)
//Author: Chen Chun-Lin
//Data: 2015/01/07
//Update: 2015/02/04, 2017/04/11
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cereal_port/CerealPort.h>
#include <axon_link/AxonNode.h>

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
#define AXON_WHEEL_CMD_LIMIT 127

#define GETSTR_PORT "/axon_link_vonly/axon_parameter/SerialPort"
#define GETSTR_RATE "/axon_link_vonly/axon_parameter/BaudRate"

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

cereal::CerealPort device;
rbtctrl rc;
rbtmdl rm;

ros::Subscriber axon_info_sub_;
//Publish the AXON info. over ROS
ros::Publisher axon_info_pub_;
axon_link::AxonNode axon_info;

char reply[REPLY_SIZE];
int cmd_type=0;
unsigned char rx_buff[8];
int rx_count=0;
char rx_timeout_flag=0;
int rx_timeout_cnt=0;

int left_ec=0;
int right_ec=0;

int AXON_cmd_send_flag=0;

//Publish the AXON Left/Right Wheel Speed command info. over ROS
ros::Publisher axon_wlr_pub_;

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
    unsigned int w_ui=0;
    unsigned char w_Xa=0;

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
        {
            //w_ui=(unsigned int)(12.519* pow(w_X, 0.5014) );
            w_ui=(unsigned int)(16.622*w_X+0.1538);

            Sat_Value_ui(&w_ui, AXON_WHEEL_CMD_LIMIT, 0);

            if (neg_flag==1)
                w_Xa=(unsigned char)(w_ui+127);
            else
                w_Xa=(unsigned char)w_ui;
        }
        else
            w_Xa=0;

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
        for (i=0;i<TX_LEN;i++)
        {
            b[0]=(char)a[i];
            b[1]='\0';
            device.write(b,1);  //Send one byte over the serial port
            //ros::Duration d= ros::Duration(TX_ONE_BYTE_DELAY); //ROS delay function (ICL must delay between byte by byte)
        }
        AXON_cmd_send_flag=1;
        ROS_INFO("I Send:cmd=%d, P1=%d, P2=%d", a[0], a[1], a[2]);
    }
}

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    geometry_msgs::Twist wlr_cmd;
    rc.v_cmd = cmd_vel->linear.x;
    rc.w_cmd = cmd_vel->angular.z;
    //ROS_INFO("I heard: [v_cmd=%2.3f]", rc.v_cmd);
    //ROS_INFO("I heard: [w_cmd=%2.3f]", rc.w_cmd);

    //set VW to set WLR (unit: rad/s)
    rc.w_L= (1/rm.r)*rc.v_cmd - (rm.L/(2*rm.r))*rc.w_cmd;
    rc.w_R= (1/rm.r)*rc.v_cmd + (rm.L/(2*rm.r))*rc.w_cmd;

    //set WLR to AXON motor speed command
    rc.w_La=AXON_Wheel_Motor_Speed(rc.w_L);
    rc.w_Ra=AXON_Wheel_Motor_Speed(rc.w_R);

    ROS_INFO("I heard: [v_cmd=%2.4f w_cmd=%2.4f w_L=%2.4f w_R=%2.4f]", rc.v_cmd, rc.w_cmd, rc.w_L, rc.w_R);
    ROS_INFO("I heard: [w_La=%d w_Ra=%d]", rc.w_La, rc.w_Ra);

    //Send Command to AXON
    AXON_Command_Send(1, rc.w_La, rc.w_Ra);

    //Publish AXON Left/Right Wheel
    wlr_cmd.linear.x=rc.v_cmd;
    wlr_cmd.linear.y=rc.w_La;
    wlr_cmd.linear.z=rc.w_Ra;
    wlr_cmd.angular.x=0.0;
    wlr_cmd.angular.y=0.0;
    wlr_cmd.angular.z=rc.w_cmd;
    axon_wlr_pub_.publish(wlr_cmd);
}

void axonInfoSubReceived(const axon_link::AxonNode::ConstPtr& info_sub)
{
    unsigned int auto_flag;
    unsigned int ec_flag;
    auto_flag=info_sub->ad_start_flag;
    ec_flag=info_sub->ec_get_flag;

    if (auto_flag==1)
    {
        //Send Command to AXON
        AXON_Command_Send(2, 0, 0);
    }
    else if (ec_flag==1)
    {
        //Send Command to AXON
        AXON_Command_Send(3, 0, 0);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "axon_link_node");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub_;

    axon_info_pub_= n.advertise<axon_link::AxonNode>("axon_info_pub", 1);
    axon_wlr_pub_= n.advertise<geometry_msgs::Twist>("axon_wlr_cmd", 1);
    cmd_vel_sub_  = n.subscribe<geometry_msgs::Twist>("axon_cmd_vel", 1000, cmdVelReceived);
    axon_info_sub_= n.subscribe<axon_link::AxonNode>("axon_info_sub", 1000, axonInfoSubReceived);

    rm.L=AXON_ROBOT_L;
    rm.r=AXON_ROBOT_R;
    rc.ts=TS;

    int i;
    int baud_rate=0;
    std::string serial_port;

    n.getParam(GETSTR_PORT, serial_port);
    ROS_INFO("I get SerialPort: %s", serial_port.c_str());

    n.getParam(GETSTR_RATE, baud_rate);
    ROS_INFO("I get BaudRate: %d", baud_rate);

    axon_info.axon_msg=0;

    // Change the next line according to your port name and baud rate
    //try{ device.open("/dev/ttyUSB0", 115200); }
    try{ device.open(serial_port.c_str(), baud_rate); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the AXON serial port!!!");
        ROS_BREAK();
    }
    ROS_INFO("The AXON serial port is opened.");

    ros::Rate r(20);
    while(ros::ok())
    {
        //==v main loop v==========================================================================
        rx_timeout_flag=0;
        // Get the reply, the last value is the timeout in ms
        try{ device.read(reply, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            //ROS_ERROR("Timeout!");
            rx_timeout_flag=1;
        }

        //if (rx_timeout_flag==0)
            //ROS_INFO("Got this reply: %d", (unsigned char)reply[0]);

        //=================================================================
        //Packet
        if (rx_timeout_flag==0)
        {
            rx_timeout_cnt=0;
            if ( (cmd_type==0) && ( (reply[0]==0x01) || (reply[0]==0x02) || (reply[0]==0x03) ) )
            {
                cmd_type=(int)reply[0];
                //ROS_INFO("I Got cmd_type: %d", cmd_type);
            }
            else if( (cmd_type==1) || (cmd_type==2) || (cmd_type==3) )
            {
                rx_buff[rx_count]=(unsigned char)reply[0];
                //ROS_INFO("I Got rx_buff[%d]: %d", rx_count, rx_buff[rx_count]);
                rx_count++;
                if (rx_count>=1)
                {
                    if ( (cmd_type==1) && (rx_count==1) )
                    {
                        axon_info.ms_set_respone=rx_buff[0];
                        axon_info.axon_msg++;
                        if (axon_info.axon_msg>=255)
                            axon_info.axon_msg=0;
                        axon_info_pub_.publish(axon_info);
                        if (axon_info.ms_set_respone==1)
                            ROS_INFO("AXON Motor Speed setting Success~~");
                        else
                            ROS_INFO("AXON Motor Speed setting Failure!!");
                        axon_info.ms_set_respone=0;

                        //Clear RX Buff and reset counter
                        cmd_type=0;
                        rx_count=0;
                        for (i=0;i<8;i++)
                            rx_buff[i]=0;

                        AXON_cmd_send_flag=0;
                    }
                    else if ( (cmd_type==2) && (rx_count==1) )
                    {
                        axon_info.axon_msg++;
                        if (axon_info.axon_msg>=255)
                            axon_info.axon_msg=0;
                        axon_info.ad_set_respone=rx_buff[0];
                        axon_info_pub_.publish(axon_info);
                        if (axon_info.ad_set_respone==1)
                            ROS_INFO("AXON Auto-Docking start Success~~");
                        else
                            ROS_INFO("AXON Auto-Docking start Failure!!");
                        axon_info.ad_set_respone=0;

                        //Clear RX Buff and reset counter
                        cmd_type=0;
                        rx_count=0;
                        for (i=0;i<8;i++)
                            rx_buff[i]=0;

                        AXON_cmd_send_flag=0;
                    }
                    else if ( (cmd_type==3) && (rx_count==8) )
                    {
                        left_ec=0;
                        right_ec=0;

                        left_ec=(int)(rx_buff[0]<<24);
                        left_ec=left_ec+(int)(rx_buff[1]<<16);
                        left_ec=left_ec+(int)(rx_buff[2]<<8);
                        left_ec=left_ec+(int)(rx_buff[3]);

                        right_ec=(int)(rx_buff[4]<<24);
                        right_ec=left_ec+(int)(rx_buff[5]<<16);
                        right_ec=left_ec+(int)(rx_buff[6]<<8);
                        right_ec=left_ec+(int)(rx_buff[7]);

                        axon_info.left_motor_ec=left_ec;
                        axon_info.right_motor_ec=right_ec;
                        axon_info.ec_get_flag=1;
                        axon_info.axon_msg++;
                        if (axon_info.axon_msg>=255)
                            axon_info.axon_msg=0;
                        axon_info_pub_.publish(axon_info);
                        ROS_INFO("Encoder: LeftEc=%d, RighEc=%d", axon_info.left_motor_ec, axon_info.right_motor_ec);
                        //ROS_INFO("Encoder: Left=%d,%d,%d,%d Right=%d,%d,%d,%d", rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4], rx_buff[5], rx_buff[6], rx_buff[7]);
                        axon_info.ec_get_flag=0;
                        axon_info.left_motor_ec=0;
                        axon_info.right_motor_ec=0;

                        //Clear RX Buff and reset counter
                        cmd_type=0;
                        rx_count=0;
                        for (i=0;i<8;i++)
                            rx_buff[i]=0;

                        AXON_cmd_send_flag=0;
                    }
                }
            }
            else
            {
                rx_count++;
                if (rx_count>=9)
                {
                    rx_count=0;
                    cmd_type=0;
                }
            }
        }
        else
        {
            rx_timeout_cnt++;
            if (rx_timeout_cnt>100000)
            {
                cmd_type=0;
                rx_count=0;
                for (i=0;i<8;i++)
                    rx_buff[i]=0;
                AXON_cmd_send_flag=0;
            }
        } //end if (rx_timeout_flag==0)
        //==^ main loop ^=============================================================================
        ros::spinOnce();
        r.sleep();
    }   
}
