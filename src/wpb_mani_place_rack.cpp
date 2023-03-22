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
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

// 放置参数调节（单位：米）
static float place_y_offset = 0.0f;          //机器人的横向位移补偿量
static float place_z_offset = 0.03f;       //手臂抬起高度的补偿量
static float place_forward_offset = 0.0f;    //手臂抬起后，机器人向前移动的位移补偿量
static float place_gripper_value = 0.07;     //放置物品时，手爪松开后的手指间距

static float forward_lv3 = 0.00;   //第三层货架(最高层)前进的修正量(单位:米)
static float forward_lv2 = 0.00;   //第二层货架(中间层)前进的修正量(单位:米)
static float forward_lv1 = 0.00;   //第一层货架(最底层)前进的修正量(单位:米)

static float target_y_k = 0.5; 
static float vel_max = 0.5;                     //移动限速

#define STEP_WAIT                   0
#define STEP_PLACE_DIST     1
#define STEP_FORWARD          2
#define STEP_RELEASE            3
#define STEP_BACKWARD       4
#define STEP_Y_BACK               5
#define STEP_ARM_BACK         6
#define STEP_DONE                   7
#define STEP_FAILED                 8
static int nStep = STEP_WAIT;

static std::string pc_topic;
static ros::Publisher vel_pub;
static ros::Publisher mani_ctrl_pub;
static sensor_msgs::JointState mani_ctrl_msg;
static ros::Publisher result_pub;
static std_msgs::String result_msg;
static ros::Publisher odom_ctrl_pub;
static std_msgs::String odom_ctrl_msg;
static geometry_msgs::Pose2D pose_diff;

void VelCmd(float inVx , float inVy, float inTz);
float ReachOut(float inGrabZ);
float TargetX_Fixed(float inZ);

static float fPlaceX = 0;
static float fPlaceY = 0;
static float fPlaceZ = 0;
static float fMoveTargetX = 0;
static float fMoveTargetY = 0;

static int nTimeDelayCounter = 0;
static float reachout_x_offset = 0.0;

/*****************************************************************
* 设置抓取位置的回调函数
******************************************************************/
void PlaceActionCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 放置物品的坐标
    fPlaceX = msg->position.x;
    fPlaceY = msg->position.y;
    fPlaceZ = msg->position.z;
    ROS_WARN("[Place] x = %.2f y= %.2f ,z= %.2f " ,fPlaceX, fPlaceY, fPlaceZ);
    odom_ctrl_msg.data = "pose_diff reset";
    odom_ctrl_pub.publish(odom_ctrl_msg);

    // ajudge the dist to place
    ROS_WARN("放置高度 = %.3f",fPlaceZ + place_z_offset);
    float x_offset = ReachOut(fPlaceZ + place_z_offset);
    ROS_WARN("手臂伸出长度 x_offset = %.3f",x_offset);
    if(x_offset == 0.0f)
    {
        ROS_WARN("手臂姿态规划失败...");
        nStep = STEP_FAILED;
        return;
    }
    fMoveTargetX = fPlaceX - x_offset - 0.22 + TargetX_Fixed(fPlaceZ);
    fMoveTargetY = fPlaceY + place_y_offset;
    ROS_WARN("[MOVE_TARGET] x = %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
    nTimeDelayCounter = 0;
    nStep = STEP_PLACE_DIST;
}

/*****************************************************************
* 返回局部里程距离的回调函数
******************************************************************/
void PoseDiffCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    pose_diff.x = msg->x;
    pose_diff.y = msg->y;
    pose_diff.theta = msg->theta;
}

float VelFixed(float inVel,float inMax)
{
    float retVel = inVel;
    if(retVel > inMax)
        retVel = inMax;
    if(retVel < -inMax)
        retVel = -inMax;
    return retVel;
}

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = VelFixed(inVx , vel_max);
    vel_cmd.linear.y = VelFixed(inVy , vel_max);
    vel_cmd.angular.z = VelFixed(inTz , vel_max);
    vel_pub.publish(vel_cmd);
}

float TargetX_Fixed(float inZ)
{
    if(inZ < 0.18)
        return forward_lv1;
    else if(inZ < 0.33)
        return forward_lv2;
    else if(inZ < 0.47)
        return forward_lv3;
    else
        return 0;
}

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
    if(inGrabZ < min_grab_z - 0.02)
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

        reachout_x_offset = joint3_lenght*sin(angle) + (elbow_lenght + joint4_lenght) * cos(angle) + gripper_lenght;
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

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = msg->data.find("place stop");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[place stop] ");
        nStep = STEP_WAIT;
        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
        vel_pub.publish(vel_cmd);
    }

}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wpb_mani_place_rack");

    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpb_mani/joint_ctrl", 30);
    result_pub = nh.advertise<std_msgs::String>("/wpb_mani/place_result", 30);

    ros::Subscriber sub_grab_pose = nh.subscribe("/wpb_mani/place_action", 1, PlaceActionCallback);
    odom_ctrl_pub = nh.advertise<std_msgs::String>("/wpb_mani/ctrl", 30);
    ros::Subscriber pose_diff_sub = nh.subscribe("/wpb_mani/pose_diff", 1, PoseDiffCallback);

    ros::NodeHandle nh_param("~");
    nh_param.getParam("grab/target_y_k", target_y_k);
    ROS_WARN("[grab_box] target_y_k = %f",target_y_k);

    ros::Rate r(30);
    while(nh.ok())
    {
        //1、左右平移对准放置目标点，同时抬起手臂 
        if(nStep == STEP_PLACE_DIST)
        {
            float vx,vy;
            vx = 0;//(fMoveTargetX - pose_diff.x)/2;
            vy = (fMoveTargetY - pose_diff.y)/2;
            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);
            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                if(nTimeDelayCounter > 6.5*30)
                {
                    odom_ctrl_msg.data = "pose_diff reset";
                    odom_ctrl_pub.publish(odom_ctrl_msg);
                    fMoveTargetY = 0;
                    nTimeDelayCounter = 0;
                    nStep = STEP_FORWARD;
                    continue;
                }
            }
            else
            {
                VelCmd(vx,vy,0);
            }

            nTimeDelayCounter ++;

            result_msg.data = "dist to place";
            result_pub.publish(result_msg);
        }

        //2、前进靠近放置点
        if(nStep == STEP_FORWARD)
        {
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2;
            vy = (fMoveTargetY - pose_diff.y)/2;

            VelCmd(vx,vy,0);
            //ROS_INFO("[STEP_FORWARD] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                odom_ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(odom_ctrl_msg);
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_RELEASE] grab_gripper_value = %.2f",place_gripper_value);
                nStep = STEP_RELEASE;
            }

            result_msg.data = "forward";
            result_pub.publish(result_msg);
        }

        //3、释放物品
        if(nStep == STEP_RELEASE)
        {
            if(nTimeDelayCounter == 0)
            {
                result_msg.data = "release";
                result_pub.publish(result_msg);
            }
            ManiGripper(place_gripper_value);      //释放物品手爪闭合宽度

            nTimeDelayCounter++;
            VelCmd(0,0,0);
            if(nTimeDelayCounter > 2.5*30)
            {
                nTimeDelayCounter = 0;
                fMoveTargetX = -(fPlaceX - 0.5 + place_forward_offset);
                //fMoveTargetY = 0;
                fMoveTargetY = -(fPlaceY + place_y_offset);
                ROS_WARN("[STEP_BACKWARD] x= %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
                nStep = STEP_BACKWARD;
            }
        }

        //4、后退
        if(nStep == STEP_BACKWARD)
        {
            //ROS_WARN("[STEP_BACKWARD] nTimeDelayCounter = %d " ,nTimeDelayCounter);
            //nTimeDelayCounter++;
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2;
            if(fabs(fMoveTargetX - pose_diff.x) > 0.1) //距离小于0.25(0.9-0.65）时就已经安全了
            {
                vy = 0;
            }
            else
            {
                vy = (fMoveTargetY - pose_diff.y)/2;
                nTimeDelayCounter ++;
            }
            VelCmd(vx,vy,0);
            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);
            if(fabs(vx) < 0.01 && fabs(vy) < 0.01 && nTimeDelayCounter > 8*30)
            {
                VelCmd(0,0,0);
                odom_ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(odom_ctrl_msg);
                nTimeDelayCounter = 0;
                nStep = STEP_ARM_BACK;
               
            }
            result_msg.data = "backward";
            result_pub.publish(result_msg);
        }
        
        //6、
        if(nStep == STEP_ARM_BACK)
        {
            if(nTimeDelayCounter < 7*30)
            {
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
                mani_ctrl_msg.position[1] = -1.4;
                mani_ctrl_msg.position[2] = 1.4;
                mani_ctrl_msg.velocity[1] = 1.05;
                mani_ctrl_msg.velocity[3] = 0.21;
                mani_ctrl_pub.publish(mani_ctrl_msg);
            }
            else
            {
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_DONE]");
                nStep = STEP_DONE;
            }
            nTimeDelayCounter++;
            VelCmd(0,0,0);
            result_msg.data = "arm back";
            result_pub.publish(result_msg);
        }

        //7、放置任务完毕
        if(nStep == STEP_DONE)
        {
            if(nTimeDelayCounter < 30)
            {
                VelCmd(0,0,0);
                nTimeDelayCounter ++;
                result_msg.data = "done";
                result_pub.publish(result_msg);
            }
            else
            {
                nStep = STEP_WAIT;
            }
        }

        //6、放置任务失败
        if(nStep == STEP_FAILED)
        {
            if(nTimeDelayCounter < 30)
            {
                VelCmd(0,0,0);
                nTimeDelayCounter ++;
                result_msg.data = "done";
                result_pub.publish(result_msg);
            }
            else
            {
                nStep = STEP_WAIT;
            }
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}