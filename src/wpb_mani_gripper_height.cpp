/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "dynamic_reconfigure/server.h"
#include "wpb_mani_agent/gripper_heightConfig.h"
#include <math.h>

static ros::Publisher mani_ctrl_pub;
static float reachout_x_offset = 0;

float ReachOut(float inGrabZ)
{
    float mani_base_height = 0.25;    // link3高度
    float joint3_lenght = 0.128;            // 第一节臂长度
    float elbow_lenght =  0.024;          // 肘部舵机长度
    float joint4_lenght = 0.124;            // 第二节臂长度
    float gripper_lenght = 0.145;       //手爪尖到关节长度
    sensor_msgs::JointState mani_ctrl_msg;
    mani_ctrl_msg.name.resize(4);
    mani_ctrl_msg.position.resize(4);
    mani_ctrl_msg.velocity.resize(4);
    mani_ctrl_msg.name[0] = "joint1";
    mani_ctrl_msg.name[1] = "joint2";
    mani_ctrl_msg.name[2] = "joint3";
    mani_ctrl_msg.name[3] = "joint4";
    for(int i=0;i<4;i++)
    {
        mani_ctrl_msg.position[i] = 0;
        mani_ctrl_msg.velocity[i] = 0.35;
    }
    mani_ctrl_msg.velocity[1] = 1.05;
    mani_ctrl_msg.velocity[3] = 0.5;
    // 下限
    float min_grab_z = mani_base_height - joint4_lenght;
    if(inGrabZ < min_grab_z)
    {
        ROS_WARN("[ ReachOut ] 抱歉，高度值低于下限 ...");
        return 0;
    }
    float max_grab_z = mani_base_height + joint3_lenght + joint4_lenght;
     if(inGrabZ > max_grab_z)
    {
        ROS_WARN("[ ReachOut ] 抱歉，高度值高于上限 ...");
        return 0;
    }
    // 计算第一节臂的俯仰角
    float angle = 0;
    float z_offset = inGrabZ - mani_base_height;
    if(z_offset <= joint3_lenght)
    {
        // 第一种情况，高度低于第一节手臂长度，则倾斜第一节手臂。
        // 解方程 joint3_lenght*cos(angle) - (joint4_lenght + elbow_lenght)*sin(angle) = z_offset
        double tmp = sqrtf(joint3_lenght * joint3_lenght + (joint4_lenght + elbow_lenght)* (joint4_lenght + elbow_lenght));
        double b = asin(joint3_lenght/tmp);
        angle = b - asin(z_offset/tmp);
        ROS_WARN("第一节手臂角度angle = %.2f",angle);

        reachout_x_offset = joint3_lenght*sin(angle) + elbow_lenght * cos(angle) + joint3_lenght*cos(angle) + gripper_lenght;
        ROS_WARN("手爪向前探出 reachout_x_offset = %.3f",reachout_x_offset);

        mani_ctrl_msg.position[1] = angle;
        mani_ctrl_msg.position[3] = -angle;
    }
    else
    {
        // 第二种情况，高度高于第一节手臂长度，则抬起第二节手臂。
        double z2 = z_offset - joint3_lenght;
        double tmp = z2 / joint4_lenght;
        angle = asin(tmp);
        ROS_WARN("第二节手臂角度angle = %.2f",angle);

        reachout_x_offset = elbow_lenght + joint4_lenght * cos(angle) + gripper_lenght;
        ROS_WARN("手爪向前探出 reachout_x_offset = %.3f",reachout_x_offset);

        mani_ctrl_msg.position[2] = -angle;
        mani_ctrl_msg.position[3] = angle;
    }
    
    mani_ctrl_pub.publish(mani_ctrl_msg);

    return reachout_x_offset;
}

void ManiGripper(float inGripperVal)
{
    sensor_msgs::JointState mani_ctrl_msg;
    mani_ctrl_msg.name.resize(1);
    mani_ctrl_msg.position.resize(1);
    mani_ctrl_msg.velocity.resize(1);
    mani_ctrl_msg.name[0] = "gripper";
    mani_ctrl_msg.position[0] = inGripperVal;
    mani_ctrl_msg.velocity[0] = 1.0;
    mani_ctrl_pub.publish(mani_ctrl_msg);
}

void cbGripperHeight(wpb_mani_agent::gripper_heightConfig &config, uint32_t level) 
{
    ROS_WARN("手爪高度 gripper_height= %f ", config.gripper_height);
    ReachOut(config.gripper_height);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wpb_mani_mani_height");

    ros::NodeHandle nh;
    mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpb_mani/joint_ctrl", 30);

    dynamic_reconfigure::Server<wpb_mani_agent::gripper_heightConfig> server;
    dynamic_reconfigure::Server<wpb_mani_agent::gripper_heightConfig>::CallbackType f;

    f = boost::bind(&cbGripperHeight, _1, _2);
    server.setCallback(f);

    ros::Time time = ros::Time(0);
    float grab_z = 0.15;
    float gripper = 0;
    int nCount = 0;
    ros::Rate r(10);
    while(nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
