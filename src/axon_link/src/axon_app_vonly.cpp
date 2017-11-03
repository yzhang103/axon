/*
//Title: i-Building for AXON Robot (IoT Lab.)
//Author: Chen, Chun-Lin
//Data: 2015/01/07
//Update:
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <axon_link/AxonNode.h>
#include <axon_link/AxonApp.h>

#define REPLY_SIZE 1
#define TIMEOUT 1000

#define PI           3.14159
#define RTD          180.0/PI       //Radian to Degree
#define DTR          PI/180.0       //Degree to Radian
#define MAIN_FREQ    10             //HZ

unsigned int ad_respone_flag=0;
unsigned int old_auto_flag=0;

unsigned int ec_flag=0;
unsigned int ms_respone_flag=0;
unsigned int ms_failure_cnt=0;
unsigned int ms_failure_long_flag=0;

unsigned int lm_ec_cnt=0;
unsigned int rm_ec_cnt=0;
unsigned int lm_ec_zero_cnt=0;
unsigned int rm_ec_zero_cnt=0;

double cmd_vel_v=0;
double cmd_vel_w=0;
unsigned int set_auto_dock=0;

unsigned int ms_stop_flag=0;
unsigned int now_auto_dock_flag=0;
unsigned int left_motor_ec=0;
unsigned int right_motor_ec=0;

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    cmd_vel_v = cmd_vel->linear.x;
    cmd_vel_w = cmd_vel->angular.z;
    //ROS_INFO("I Get cmd_vel V=%2.4f, w=%2.4f",cmd_vel_v ,cmd_vel_w);
}

void axonAppSubReceived(const axon_link::AxonApp::ConstPtr& app_sub)
{
    set_auto_dock=app_sub->set_auto_dock;

    ROS_INFO("I Get set_auto_dock=%d", set_auto_dock);
}

void axonInfoSubReceived(const axon_link::AxonNode::ConstPtr& info_sub)
{
    ad_respone_flag=info_sub->ad_set_respone;
    ms_respone_flag=info_sub->ms_set_respone;
    ec_flag=info_sub->ec_get_flag;
    left_motor_ec=info_sub->left_motor_ec;
    right_motor_ec=info_sub->right_motor_ec;

    if (ms_respone_flag==1)
    {
        ms_failure_cnt=0;
        ms_failure_long_flag=0;
        lm_ec_cnt=0;
        rm_ec_cnt=0;
        lm_ec_zero_cnt=0;
        rm_ec_zero_cnt=0;
    }
    else
    {
        ms_failure_cnt++;
        if (ms_failure_cnt>=10)
            ms_failure_long_flag=1;
        else
            ms_failure_long_flag=0;

        if (ms_failure_long_flag==1)
        {
            if (left_motor_ec!=0)
                lm_ec_cnt++;
            if (right_motor_ec!=0)
                rm_ec_cnt++;

            if ( (lm_ec_cnt>=3) || (rm_ec_cnt>=3) )
                now_auto_dock_flag=1;
        }
    }

    if (now_auto_dock_flag==1)
    {
        if (left_motor_ec==0)
            lm_ec_zero_cnt++;
        if (right_motor_ec==0)
            rm_ec_zero_cnt++;
        if ( (lm_ec_zero_cnt>=10) && (lm_ec_zero_cnt>=10) )
        {
            now_auto_dock_flag=0;
            ms_stop_flag=0;
        }
    }

    //ROS_INFO("Now Get ad_respone_flag=%d", ad_respone_flag);
    //ROS_INFO("Now Get ec_flag=%d", ec_flag);
}

int main(int argc, char** argv)
{
    int i;

    ros::init(argc, argv, "axon_app_node");
    ros::NodeHandle n;

    //-----------------------------------------------------------------
    ros::Publisher axon_cmd_vel_pub_;
    ros::Subscriber info_sub_;
    ros::Publisher info_pub_;   //Publish the AXON info. over ROS
    info_pub_= n.advertise<axon_link::AxonNode>("axon_info_sub", 1);
    axon_cmd_vel_pub_= n.advertise<geometry_msgs::Twist>("axon_cmd_vel", 1);
    info_sub_= n.subscribe<axon_link::AxonNode>("axon_info_pub", 1000, axonInfoSubReceived);
    axon_link::AxonNode info;
    geometry_msgs::Twist axon_cmd_vel;

    //----------------------------------------------------------------
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber app_sub_;
    ros::Publisher app_pub_;   //Publish the AXON app. over ROS
    app_pub_= n.advertise<axon_link::AxonApp>("axon_app_pub", 1);
    cmd_vel_sub_= n.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, cmdVelReceived);
    app_sub_= n.subscribe<axon_link::AxonApp>("axon_app_sub", 1000, axonAppSubReceived);
    axon_link::AxonApp app;

    ros::Rate r(MAIN_FREQ);
    while(ros::ok())
    {
        /*
        i++;
        if (i>=100)
            i=0;
        */

        //speed command out
        axon_cmd_vel.linear.x=cmd_vel_v;
        axon_cmd_vel.angular.z=cmd_vel_w;
        //publish the speed message
        axon_cmd_vel_pub_.publish(axon_cmd_vel);
        //ROS_INFO("I Publosh axon_cmd_vel");

        /*
        if ((i%2)==1)
        {
            if ( (ms_stop_flag==0) && (set_auto_dock==0) )
            {
                //speed command out
                axon_cmd_vel.linear.x=cmd_vel_v;
                axon_cmd_vel.angular.z=cmd_vel_w;
                //publish the speed message
                axon_cmd_vel_pub_.publish(axon_cmd_vel);
                ROS_INFO("I Publosh axon_cmd_vel");
            }
        }
        else
        {
            //publish the info message
            if (i%10==0)
            {
                if (set_auto_dock==1)
                {
                    info.ad_start_flag=1;
                    info.ec_get_flag=0;
                }
                else
                {
                    info.ad_start_flag=0;
                    info.ec_get_flag=1;
                }
            }
            else
            {
                info.ad_start_flag=0;
                info.ec_get_flag=1;
            }
            info_pub_.publish(info);
        }

        if(now_auto_dock_flag==1)
        {
            ROS_INFO("=====Now Auto-Docking!!==========================");
            ms_stop_flag=1;
            set_auto_dock=0;
        }
        //----------------------------------------------
        app.ms_stop_flag=ms_stop_flag;
        app.now_auto_dock_flag=now_auto_dock_flag;
        app.left_motor_ec=left_motor_ec;
        app.right_motor_ec=right_motor_ec;
        app_pub_.publish(app);
        */
        //==============================================
        ros::spinOnce();
        r.sleep();
    }   
}
