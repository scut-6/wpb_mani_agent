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
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <wpb_mani_behaviors/Coord.h>
#include <math.h>
#include <wpr_rack_pkg/ObjectBox.h>
#include <geometry_msgs/Pose2D.h>

// 抓取参数调节（单位：米）
static float grab_y_offset = -0.01f;                  //机器人对准物品，横向位移偏移量
static float grab_z_offset = 0.02f;                  //手臂抬起高度的补偿偏移量
static float grab_gripper_value = 0.038;    //抓取物品时，手爪闭合后的手指间距

//手臂抬起后，机器人向前抓取物品移动的位移偏移量
static float forward_lv3 = 0.00;   //第三层货架(最高层)前进的修正量(单位:米)
static float forward_lv2 = 0.00;   //第二层货架(中间层)前进的修正量(单位:米)
static float forward_lv1 = 0.00;   //第一层货架(最底层)前进的修正量(单位:米)

static float fTargetGrabX = 0.62;        //抓取时目标物品的x坐标
static float fTargetGrabY = 0.0;        //抓取时目标物品的y坐标

static float target_x_k = 0.5; 
static float target_y_k = 0.5; 

static ros::Publisher box_track_pub;
static geometry_msgs::Pose box_track_msg;
static ros::Publisher rack_pub;
static std_msgs::String rack_msg;
static ros::Publisher vel_pub;
static ros::Publisher mani_ctrl_pub;
static ros::Publisher grab_result_pub;
static std_msgs::String grab_result_msg;
static ros::Publisher odom_ctrl_pub;
static std_msgs::String odom_ctrl_msg;
static geometry_msgs::Pose2D pose_diff;

static float fObjGrabX = 0;
static float fObjGrabY = 0;
static float fObjGrabZ = 0;
static float fMoveTargetX = 0;
static float fMoveTargetY = 0;

static float box_track_x = 0.0;
static float box_track_y = 0.0;
static float box_track_z = 0.0;
static float reachout_x_offset = 0.0;
static float forward_dist = 0;

#define STEP_READY                     0
#define STEP_FACE_BOX             1
#define STEP_RACK_POS            2
#define STEP_BOX_POS               3
#define STEP_TAKE_AIM              4
#define STEP_REACH_OUT         5
#define STEP_FORWARD              6
#define STEP_GRAB_BOX            7
#define STEP_TAKE_OVER          8
#define STEP_BACKWARD           9
#define STEP_DONE                       10
int step = STEP_READY;

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
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

/*****************************************************************
* 返回锁定盒子的回调函数
******************************************************************/
void BoxCoordCB(const wpb_mani_behaviors::Coord::ConstPtr &msg)
{
    if(step == STEP_TAKE_AIM)
    {
        // 获取盒子检测结果
        box_track_x = msg->x[0];
        box_track_y = msg->y[0];
        box_track_z = msg->z[0];
        ROS_INFO("[BoxCoordCB]  %s  (%.2f , %.2f , %.2f)",msg->name[0].c_str(),msg->x[0],msg->y[0],msg->z[0]);
    }
}

/*****************************************************************
* 返回货架位置的回调函数
******************************************************************/
void RackPositionCB(const geometry_msgs::Pose::ConstPtr &msg)
{
    if( step == STEP_RACK_POS)
    {
       VelCmd(0,0,0);
        rack_msg.data = "detect object";
        rack_pub.publish(rack_msg);
        step = STEP_BOX_POS;
    }
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
/*****************************************************************
* 返回货架上料盒位置的回调函数
******************************************************************/
void ObjectsBoxCB(const wpr_rack_pkg::ObjectBox::ConstPtr &msg)
{
    if(step == STEP_BOX_POS)
    {
        VelCmd(0,0,0);
        // 查看是否在正前方有盒子可以抓取
        int nNumObj = msg->name.size();
        if(nNumObj > 0)
        {
            int nGrabIndex = 0;
            float fMinY = 100;
            for(int i=0;i<nNumObj;i++)
            {
                float obj_x = msg->xMin[i];
                float obj_y = (msg->yMin[i] + msg->yMax[i])/2;
                float obj_z = msg->zMin[i];
                ROS_INFO("[grab_rack] 物品%d (%.2f , %.2f , %.2f)",i,obj_x,obj_y,obj_z);
                if(fabs(obj_y) < fMinY)
                {
                    fMinY = fabs(obj_y);
                    nGrabIndex = i;
                }
            }
            float obj_x = msg->xMin[nGrabIndex];
            float obj_y = (msg->yMin[nGrabIndex] + msg->yMax[nGrabIndex])/2;
            float obj_z = msg->zMin[nGrabIndex];
            ROS_WARN("[grab_rack] --------------------------------");
            ROS_WARN("[grab_rack] 最终目标 %d (%.2f , %.2f , %.2f) ",nGrabIndex,obj_x,obj_y,obj_z);
            ROS_WARN("[grab_rack] --------------------------------");

            if(fabs(obj_y) < 0.03 && fabs(obj_x - fTargetGrabX) < 0.03)
            {
                box_track_x = obj_x;
                box_track_y = obj_y;
                box_track_z = obj_z;
                box_track_msg.position.x = box_track_x;
                box_track_msg.position.y = box_track_y;
                box_track_msg.position.z = box_track_z;
                box_track_pub.publish(box_track_msg);
                ROS_WARN("[grab_rack] 开始锁定目标 (%.2f , %.2f , %.2f) " ,box_track_x, box_track_y, box_track_z);
                step = STEP_TAKE_AIM;
            }
            else
            {
                fMoveTargetX = obj_x - fTargetGrabX;
                fMoveTargetY = obj_y - fTargetGrabY + grab_y_offset;
                ROS_WARN("[grab_rack] 没对准 ... 再次平移 x = %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
                step = STEP_FACE_BOX;
            }
        }
    }
}
void GrabBoxCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 抓取目标的初始坐标
    fObjGrabX = msg->position.x;
    fObjGrabY = msg->position.y;
    fObjGrabZ = msg->position.z;
    ROS_WARN("[grab_rack] 接收到抓取指令 x = %.2f y= %.2f ,z= %.2f " ,fObjGrabX, fObjGrabY, fObjGrabZ);
    odom_ctrl_msg.data = "pose_diff reset";
    odom_ctrl_pub.publish(odom_ctrl_msg);

    // ajudge the dist to obj
    fMoveTargetX = fObjGrabX - fTargetGrabX;
    fMoveTargetY = fObjGrabY - fTargetGrabY + grab_y_offset;
    ROS_WARN("[grab_rack] 需要再平移 x = %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);

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
    mani_ctrl_msg.position[3] = 0.5;
    mani_ctrl_msg.velocity[1] = 1.05;
    mani_ctrl_msg.velocity[2] = 1.05;
    mani_ctrl_msg.velocity[3] = 0.21;
    mani_ctrl_pub.publish(mani_ctrl_msg);
    ManiGripper(0.7);

    step = STEP_FACE_BOX;
}

float VelFixed(float inValue)
{
    float retValue = inValue;
    if(retValue > 0.5)
    {
        retValue = 0.5;
    }
    if(retValue < -0.5)
    {
        retValue = -0.5;
    }
    return retValue;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wpb_mani_grab_rack");

    ros::NodeHandle nh;
    ros::Publisher behaviors_pub = nh.advertise<std_msgs::String>("/wpb_mani/behaviors", 10);
    box_track_pub =  nh.advertise<geometry_msgs::Pose>("/wpb_mani/box_track", 10);
    ros::Subscriber box_result_sub = nh.subscribe("/wpb_mani/boxes_3d", 10 , BoxCoordCB);
    rack_pub = nh.advertise<std_msgs::String>("rack/command", 10); 
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    odom_ctrl_pub = nh.advertise<std_msgs::String>("/wpb_mani/ctrl", 30);
    ros::Subscriber pose_diff_sub = nh.subscribe("/wpb_mani/pose_diff", 1, PoseDiffCallback);
    mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpb_mani/joint_ctrl", 30);
    ros::Subscriber track_sub = nh.subscribe("/wpb_mani/grab_box", 10, GrabBoxCallback);
    grab_result_pub = nh.advertise<std_msgs::String>("/wpb_mani/grab_result", 30);
    ros::Subscriber rack_sub = nh.subscribe("rack/rack_position", 10, RackPositionCB);
    ros::Subscriber obj_sub = nh.subscribe("rack/obj_position", 10, ObjectsBoxCB);

    ros::NodeHandle nh_param("~");
    nh_param.getParam("grab/target_y_k", target_y_k);
    ROS_WARN("[grab_box] target_y_k = %f",target_y_k);

    tf::TransformListener tf_listener; 
    tf::StampedTransform ar_transform;

    ros::Time time = ros::Time(0);
    float grab_z = 0.15;
    float gripper = 0;
    int nCount = 0;
    ros::Rate r(30);
    while(nh.ok())
    {
        // 第一次对准目标盒子
        if(step == STEP_FACE_BOX)
        {
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2;
            vy = (fMoveTargetY - pose_diff.y)/2;

            VelCmd(vx,vy,0);
            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                odom_ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(odom_ctrl_msg);
                std_msgs::String behavior_msg;
                rack_msg.data = "detect pallet rack";
                rack_pub.publish(rack_msg);
                behavior_msg.data = "face_box ok!";
                behaviors_pub.publish(behavior_msg);
                nCount = 0;
                step = STEP_RACK_POS;
            }
        }

        // 锁定目标盒子
        if(step == STEP_TAKE_AIM)
        {
            float diff_x = box_track_x - fTargetGrabX;
            float diff_y = box_track_y - grab_y_offset;

            float vx = VelFixed(diff_x * target_x_k);
            float vy = VelFixed(diff_y * target_y_k); 

            VelCmd(vx,vy,0);
            ROS_WARN("diff_y = %.3f",diff_y);
            if(fabs(diff_x) < 0.01 && fabs(diff_y) < 0.005)
            {
                VelCmd(0,0,0);
                std_msgs::String behavior_msg;
                behavior_msg.data = "box_track stop";
                behaviors_pub.publish(behavior_msg);
                nCount = 0;
                step = STEP_REACH_OUT;
            }
        }

        // 伸手准备抓取
        if(step == STEP_REACH_OUT)
        {
            if(nCount == 0)
            {
                ROS_WARN("抓取高度 = %.3f",fObjGrabZ + grab_z_offset);
                float x_offset = ReachOut(fObjGrabZ + grab_z_offset);
                ROS_WARN("手臂伸出长度 x_offset = %.3f",x_offset);
                fMoveTargetX = fTargetGrabX - x_offset - 0.16 + TargetX_Fixed(fObjGrabZ + grab_z_offset);
                ManiGripper(0.7);
                odom_ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(odom_ctrl_msg);
            }
            nCount ++;
            if(nCount > 6* 30)
            {
                nCount = 0;
                step = STEP_FORWARD;
            }
        }

        // 前进进行抓取
        if(step == STEP_FORWARD)
        {
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2;
            vy = 0;
            VelCmd(vx,vy,0);
            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);
            if(fabs(vx) < 0.01 && fabs(vy) < 0.01 )
            {
                VelCmd(0,0,0);
                odom_ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(odom_ctrl_msg);
                nCount = 0;
                step = STEP_GRAB_BOX;
            }
        }

        // 闭合手爪
        if(step == STEP_GRAB_BOX)
        {
            VelCmd(0,0,0);
            ManiGripper(grab_gripper_value);
            nCount ++;
            if(nCount > 3* 30)
            {
                nCount = 0;
                step = STEP_BACKWARD;
            }
        }

        // 后退
        if(step == STEP_BACKWARD)
        {
            VelCmd(-0.1,0,0);
            nCount ++;
            if(nCount > 1* 30)
            {
                nCount = 0;
                step = STEP_TAKE_OVER;
            }
        }

        // 收回手臂
        if(step == STEP_TAKE_OVER)
        {
            VelCmd(0,0,0);
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
            nCount ++;
            if(nCount > 6.5 * 30)
            {
                nCount = 0;
                step = STEP_DONE;
            }
        }

        // 结束
        if(step == STEP_DONE)
        {
            if(nCount < 5)
            {
                VelCmd(0,0,0);
                grab_result_msg.data = "done";
                grab_result_pub.publish(grab_result_msg);
            }
            nCount ++;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
