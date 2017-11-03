/*
//Title: Keyboard Teleop (IoT Lab)
//Author: Chen, Chun-Lin
//Data: 2014/07/29
//Update: 2016/01/11
*/

//ROS Package name: Robot_SLAM_Sensors
//ROS Node name: keyboard_teleop_node_XXXXXXXXXX
//ROS Launch name:

/*!
 * drrobot_keyboard_teleop.cpp
 * Copyright (c) 2011, Dr Robot Inc
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
@mainpage
  drrobot_keyboard_teleop for demonstration and testing published geometry_msgs/Twist message to drrobot_player.
  It will use 4 keys to control robot move around
  a/A -- 0.5/1 full speed turn to left
  w/W -- 0.5/1 full speed forward
  d/D -- 0.5/1 full speed turn to right
  s/S -- 0.5/1 full speed backward
  if no key pressed, it will stop robot
<hr>
@section usage Usage
@par     After start roscore, you need load robot configuration file to parameter server first.
          For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
          then run drrobot_player first.
@verbatim
$ keyboard_teleop_node
@endverbatim
<hr>
@section topic ROS topics
Publishes to (name / type):
-@b drrobot_cmd_vel: will publish drrobot_cmd_vel Message to drrobot_player. For robot from Dr Robot Inc, we only need provide linear.x
    as going forward/backward speed, and angular.z as turning speed. drrobot_player will transform these command value to encoder control
    command value and send them to motion control system on the robot
<hr>
*/
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_W_CAP 0x57
#define KEYCODE_A_CAP 0x41
#define KEYCODE_S_CAP 0x53
#define KEYCODE_D_CAP 0x44

#define KEYCODE_O 0x6F
#define KEYCODE_O_CAP 0x4F
#define KEYCODE_P 0x70
#define KEYCODE_P_CAP 0x50

#define KEYCODE_Y 0x79
#define KEYCODE_I 0x69
#define KEYCODE_H 0x68
#define KEYCODE_K 0x6B
#define KEYCODE_Y_CAP 0x59
#define KEYCODE_I_CAP 0x49
#define KEYCODE_H_CAP 0x48
#define KEYCODE_K_CAP 0x4B


class DrRobotKeyboardTeleopNode
{
    private:
        geometry_msgs::Twist cmdvel_;
        ros::NodeHandle n_;
        ros::Publisher pub_;

    public:
        DrRobotKeyboardTeleopNode()
        {
            pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            ros::NodeHandle n_private("~");
        }

        ~DrRobotKeyboardTeleopNode() { }
        void keyboardLoop();

        void stopRobot()
        {
            cmdvel_.linear.x = 0;
            cmdvel_.linear.y = 0;
            cmdvel_.linear.z = 0;
            cmdvel_.angular.z = 0;
            pub_.publish(cmdvel_);
        }
};

DrRobotKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

double set_speed=0.2;
double set_zspeed=0.2;
int spset_flag=0;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"keyboard_ctrl_axon_node", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    DrRobotKeyboardTeleopNode tbk;

    boost::thread t = boost::thread(boost::bind(&DrRobotKeyboardTeleopNode::keyboardLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}

void DrRobotKeyboardTeleopNode::keyboardLoop()
{
    char c;
    double xVel = 0.2;
    double yVel = 0.2;
    double zVel = 1.0;
    double yawVel = 0.2;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the UAV speed");
    puts("Use Y/H keys to control the UAV Z axis speed");
    puts("Use O/P keys to control the UAV Yaw speed");
    puts("Use I/K keys to (+/-) X Y Yaw Speed (0.01)");
    puts("Press Shift to move faster (1.0)");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty == true)
            {
                stopRobot();
                dirty = false;
            }

            continue;
        }

        switch(c)
        {
            case KEYCODE_W:
                xVel = set_speed;
                yVel = 0;
                zVel = 0;
                yawVel = 0;
                dirty = true;
                break;
            case KEYCODE_S:
                xVel = -set_speed;
                yVel = 0;
                zVel = 0;
                yawVel = 0;
                dirty = true;
                break;
            case KEYCODE_A:
                xVel = 0;
                yVel = set_speed+0.2;
                zVel = 0;
                yawVel = 0;
                dirty = true;
                break;
            case KEYCODE_D:
                xVel = 0;
                yVel = -set_speed-0.2;
                zVel = 0;
                yawVel = 0;
                dirty = true;
                break;
            case KEYCODE_Y:
                xVel = 0;
                yVel = 0;
                zVel = set_speed;
                yawVel = 0;
                dirty = true;
                break;
            case KEYCODE_H:
                xVel = 0;
                yVel = 0;
                zVel = -set_speed;
                yawVel = 0;
                dirty = true;
                break;
            case KEYCODE_O:
                xVel = 0;
                yVel = 0;
                zVel = 0;
                yawVel = set_speed;
                dirty = true;
                break;
            case KEYCODE_P:
                xVel = 0;
                yVel = 0;
                zVel = 0;
                yawVel = -set_speed;
                dirty = true;
                break;

            case KEYCODE_K:
                set_speed=set_speed-0.01;
                if (set_speed<0)
                    set_speed=0;
                if (set_speed>1)
                    set_speed=1;
                spset_flag=1;
                dirty = true;
                break;
            case KEYCODE_I:
                set_speed=set_speed+0.01;
                if (set_speed<0)
                    set_speed=0;
                if (set_speed>1)
                    set_speed=1;
                spset_flag=1;
                dirty = true;
                break;


            default:
                xVel = 0;
                yVel = 0;
                zVel = 0;
                yawVel = 0;
                dirty = false;
        }

        cmdvel_.linear.x = xVel;
        cmdvel_.linear.y = 0;
        cmdvel_.linear.z = 0;
        cmdvel_.angular.z = yVel;
        if (spset_flag==0)
        {
            ROS_INFO("Send control command [ %f, %f, %f, %f]", cmdvel_.linear.x, cmdvel_.linear.y ,cmdvel_.linear.z, cmdvel_.angular.z);
            pub_.publish(cmdvel_);
        }
        else
        {
            ROS_INFO("Setting Base Speed [%f, %f]", set_speed, set_zspeed);
            spset_flag=0;
        }
    }
}
