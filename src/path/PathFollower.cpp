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
#include "PathFollower.h"
#include "LocalMap.h"

static float ranges[720];
static bool bTargetPose = false;
CPathFollower::CPathFollower()
{
    m_goal_reached = false;
    m_bInitialized = false;
    m_nPathIndex = 0;
    m_bFirstStep = true;
    m_tf_listener = NULL; 
    m_nStep = PF_STEP_WAIT;

    m_max_vel_trans = 0.3;
    m_max_vel_rot = 0.9;
    m_acc_scale_trans = 1.0;
    m_acc_scale_rot = 0.6;
    m_goal_dist_tolerance = 0.2;
    m_goal_yaw_tolerance = 0.03;

    InitHelper("LocalMap");
}

CPathFollower::~CPathFollower()
{
    if(m_tf_listener != NULL)
        delete m_tf_listener;
}

void CPathFollower::Initialize()
{
    if(!m_bInitialized)
    {	
        ros::NodeHandle n;
        m_path_pub  = n.advertise<nav_msgs::Path>("follow_path_display",10);
        m_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",10);
        m_lidar_sub = n.subscribe<sensor_msgs::LaserScan>("scan",1,&CPathFollower::LidarCallback,this);
        m_path_sub = n.subscribe<geometry_msgs::PoseArray>("follow_path",1,&CPathFollower::PathCallback,this);
        result_pub = n.advertise<std_msgs::String>("follow_path_result", 10);

        std::string ns = n.getNamespace();
        // ROS_WARN("[CPathFollower] 命名空间 = %s",ns.c_str());
        m_global_frame_id = /*ns + */"map"; 
        m_robot_base_frame_id = "base_footprint";

        m_tf_listener = new tf::TransformListener;
        m_bInitialized = true;
    }
}

static double fScaleD2R = 3.14159 / 180;
void CPathFollower::LidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //ROS_INFO("CPathFollower::LidarCallback");
    //记录激光距离点
    for (int i=0;i<720;i++)
    {
        ranges[i] = scan->ranges[i];
    }
}

static geometry_msgs::PoseStamped tmp_pose;
void CPathFollower::PathCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    int pose_num = msg->poses.size();
    ROS_INFO("[CPathFollower::PathCallback] path.sieze = %d",(int) pose_num);
    while(m_global_plan.size() > 0)
    {
        m_global_plan.clear();
    }
    m_nPathIndex = 0;

    if(pose_num == 0)   //停止信号
    {
        m_nStep = PF_STEP_WAIT;
        return;
    }
    
    for(int i=0;i<pose_num;i++)
    {
        tmp_pose.header.seq = i + 1;
        tmp_pose.header.stamp = ros::Time::now();
        tmp_pose.header.frame_id = "map";
        tmp_pose.pose = msg->poses[i];
        m_global_plan.push_back(tmp_pose);
    }
    m_goal_reached = false;
    m_nStep = PF_STEP_GOTO;
    m_bFirstStep = true;

    // 判断是否在终点调整角度
    if(msg->poses[pose_num-1].orientation.w != 1.0)
    {
        ROS_WARN("[CPathFollower] 这是要调整终点角度的");
        bTargetPose = true;
    }
    else
    {
        ROS_WARN("[CPathFollower] 不需要调整角度");
        bTargetPose = false;
    }

    // 显示路径
    nav_msgs::Path path_msg;
    for(int i=0;i<msg->poses.size();i++)
    {
        tmp_pose.header.seq = i + 1;
        tmp_pose.header.stamp = ros::Time::now();
        tmp_pose.header.frame_id = "map";
        tmp_pose.pose = msg->poses[i];
        path_msg.poses.push_back(tmp_pose);
    }
    path_msg.header = tmp_pose.header;
    m_path_pub.publish(path_msg);
}

static double CalDirectAngle(double inFromX, double inFromY, double inToX, double inToY)
{
    double res = 0;
    double dx = inFromX - inToX;
    double dy = -(inFromY - inToY);
    if (dx == 0)
    {
        if (dy > 0)
        {
            res = 180 - 90;
        }
        else
        {
            res = 0 - 90;
        }
    }
    else
    {
        double fTan = dy / dx;
        res = atan(fTan) * 180 / 3.1415926;

        if (dx < 0)
        {
            res = res - 180;
        }
    }
    res = 180 - res;
    if (res < 0)
    {
        res += 360;
    }
    if (res > 360)
    {
        res -= 360;
    }
    res = res*3.1415926/180;
    return res;
}

static double AngleFix(double inAngle, double inMin, double inMax)
{
    if (inMax - inMin > 6.28)
    {
        return inAngle;
    }
    
    double retAngle = inAngle;
    while (retAngle < inMin)
    {
        retAngle += 6.28;
    }
    while (retAngle > inMax)
    {
        retAngle -= 6.28;
    }
    return retAngle;
}

static double target_x, target_y, target_th;
bool CPathFollower::ComputeVelocity()
{
    // ROS_WARN("CPathFollower::ComputeVelocity() ");
    if(!m_bInitialized)
    {
        ROS_ERROR("CPathFollower has not been initialized, please call Initialize() before using this planner");
        return false;
    }

    if(m_nStep == PF_STEP_WAIT)
        return false;

    int path_num = m_global_plan.size();
    if(path_num == 0)
    {
        return false;
    }

    m_cmd_vel.linear.x = 0;
    m_cmd_vel.linear.y = 0;
    m_cmd_vel.angular.z = 0;
    bool res = true;

    /////////////////////////////////////////////////
    if(m_bFirstStep == true)
    {
        while(m_nPathIndex < path_num-1)
        {
            getTransformedPosition(m_global_plan[m_nPathIndex], m_robot_base_frame_id, target_x, target_y, target_th);
            if(sqrt(target_x*target_x + target_y*target_y) < m_goal_dist_tolerance)
            {
                m_nPathIndex ++;
                ROS_WARN("[PF-GOTO]target = %d ",m_nPathIndex);
            }
            else
            {
                break;  //target is far enough
            }
        }

        double face_target = CalDirectAngle(0, 0, target_x, target_y);
        face_target = AngleFix(face_target,-2.1,2.1);
        if(fabs(face_target)> 0.09)
        {
            //turn in place
            m_cmd_vel.linear.x = 0;
            m_cmd_vel.linear.y = 0;
            m_cmd_vel.angular.z = face_target * m_acc_scale_rot;
            if(m_cmd_vel.angular.z > 0) m_cmd_vel.angular.z +=0.2;
            if(m_cmd_vel.angular.z < 0) m_cmd_vel.angular.z -=0.2;
        }
        else
        {
            m_bFirstStep = false;
        }
    }
    ////////////////////////////////////////////////////////
    
    if(m_nStep == PF_STEP_ARRIVED)
    {
        // ROS_WARN("[PF_ARRIVED](%.2f %.2f):%.2f",m_cmd_vel.linear.x,m_cmd_vel.linear.y,m_cmd_vel.angular.z);
        result_msg.data = "done";
        result_pub.publish(result_msg);
        m_nStep = PF_STEP_WAIT;
        return true;
    }

        /////////////////////////////////////////////////////////////
    //getTransformedPosition(m_global_plan[m_nPathIndex], m_robot_base_frame_id, gx, gy, gth);
    double goal_x,goal_y,goal_th;
    getTransformedPosition(m_global_plan.back(), m_robot_base_frame_id, goal_x, goal_y, goal_th);
    //ROS_WARN("goal(%.2f dy= %.2f) th= %.2f",goal_x, goal_y, goal_th);

    //  double face_goal = CalDirectAngle(0, 0, goal_x, goal_y);
    //  face_goal = AngleFix(face_goal,-2.1,2.1);
    //  ROS_WARN("face = %.2f goal(%.2f dy= %.2f) th= %.2f",face_goal, goal_x, goal_y, goal_th);
    
    if(m_nStep == PF_STEP_GOTO)
    {
        // check if global goal is near
        double goal_dist = sqrt(goal_x*goal_x + goal_y*goal_y);
        if(goal_dist < m_goal_dist_tolerance)
        {
            m_nStep = PF_STEP_NEAR;
            //ROS_WARN("[PF-GOTO] -> [PF_NEAR] (%.2f,%.2f) %.2f",goal_x, goal_y, goal_th);
        }
        else
        {
                /////////////////////////////////
            // 未靠近目标点,这时候需要避障
            //ROS_WARN("---------------PF_STEP_GOTO-----------------");
            // 障碍点
            ClearObst();
            SetRanges(ranges);
            ///////////////////////////////////
            //check if target is near
            double target_x, target_y, target_th;
            int path_index = m_nPathIndex;
            while(m_nPathIndex < path_num-1)
            {
                getTransformedPosition(m_global_plan[m_nPathIndex], m_robot_base_frame_id, target_x, target_y, target_th);
                if(sqrt(target_x*target_x + target_y*target_y) < m_goal_dist_tolerance  || (ChkTarget(target_y/0.05+50,target_x/0.05+50) == false))
                {
                    m_nPathIndex ++;
                    //ROS_WARN("[PF-GOTO]target = %d ",m_nPathIndex);
                }
                else
                {
                    break;  //target is far enough
                }
            }

            // 路径点
            double gpath_x, gpath_y, gpath_th;
            ClearTarget();
            for(int i=m_nPathIndex;i<path_num;i++)
            {
                getTransformedPosition(m_global_plan[i], m_robot_base_frame_id, gpath_x, gpath_y, gpath_th);
                if(i == m_nPathIndex)
                {
                    SetTarget(gpath_y/0.05+50,gpath_x/0.05+50,true);
                }
                else
                {
                    SetTarget(gpath_y/0.05+50,gpath_x/0.05+50,false);
                }
            }
            // 局部路径
            res = OutLine();
            if(res == false)
            {
                m_cmd_vel.linear.x = 0;
                m_cmd_vel.linear.y = 0;
                m_cmd_vel.angular.z = 0;
                return true;
            }
            if(GetHelperNum() > 5 && (path_num - m_nPathIndex) > 1)
            {
                target_x = GetFixX();
                target_y = GetFixY();
                gpath_x = GetFaceX();
                gpath_y = GetFaceY();
            }
            else
            {
                getTransformedPosition(m_global_plan[m_nPathIndex], m_robot_base_frame_id, target_x, target_y, target_th);
                int nFaceIndex = m_nPathIndex;
                double face_x,face_y,face_th;
                face_x = target_x;
                face_y = target_y;
                while(nFaceIndex < path_num)
                {
                    getTransformedPosition(m_global_plan[nFaceIndex], m_robot_base_frame_id, face_x,face_y,face_th);
                    double tmpDist = sqrt(face_x*face_x + face_y*face_y);
                    if(tmpDist > 0.2)
                    {
                        break;
                    }
                    nFaceIndex ++;
                }
                gpath_x = face_x;
                gpath_y = face_y;
                gpath_th = face_th;            
            }
            //ROS_WARN("target(%.2f , %.2f)       %d/%d",target_x,target_y,m_nPathIndex,path_num);

            // 朝向target
            double face_target = 0;
            if((target_x !=gpath_x) || (target_y !=gpath_y) )
            {
                face_target =CalDirectAngle(target_x, target_y, gpath_x, gpath_y);
            }
            else
            {
                face_target = CalDirectAngle(0, 0, gpath_x, gpath_y);
            }
            face_target = AngleFix(face_target,-2.1,2.1);
            if(fabs(face_target)> 0.8)
            {
                // turn in place
                m_cmd_vel.linear.x = 0;
                m_cmd_vel.linear.y = 0;
                m_cmd_vel.angular.z = face_target * m_acc_scale_rot;
                if(m_cmd_vel.angular.z > 0) m_cmd_vel.angular.z +=0.2;
                if(m_cmd_vel.angular.z < 0) m_cmd_vel.angular.z -=0.2;
            }
            else
            {
                // start to move
                m_cmd_vel.linear.x = target_x * m_acc_scale_trans;
                m_cmd_vel.linear.y = target_y * m_acc_scale_trans;
                m_cmd_vel.angular.z = face_target * m_acc_scale_rot;
            }

            if(m_cmd_vel.linear.x > 0) m_cmd_vel.linear.x+=0.05;
            if(m_cmd_vel.linear.x < 0) m_cmd_vel.linear.x-=0.05;
            if(m_cmd_vel.linear.y > 0) m_cmd_vel.linear.y+=0.02;
            if(m_cmd_vel.linear.y < 0) m_cmd_vel.linear.y-=0.02;
        }
    }

    if(m_nStep == PF_STEP_NEAR)
    {
        
        m_cmd_vel.linear.x = 0;
        m_cmd_vel.linear.y = 0;
        m_cmd_vel.angular.z = goal_th;

        if(fabs(goal_th) < m_goal_yaw_tolerance || bTargetPose == false)
        {
            m_goal_reached = true;
            m_nStep = PF_STEP_ARRIVED;
            m_cmd_vel.angular.z = 0;
            ROS_WARN("[PF-ARRIVED] goal (%.2f,%.2f) %.2f",goal_x, goal_y, goal_th);
        }
    }

    if(m_cmd_vel.linear.x > m_max_vel_trans) m_cmd_vel.linear.x = m_max_vel_trans;
    if(m_cmd_vel.linear.x < -m_max_vel_trans) m_cmd_vel.linear.x = -m_max_vel_trans;
    if(m_cmd_vel.linear.y > m_max_vel_trans) m_cmd_vel.linear.y = m_max_vel_trans;
    if(m_cmd_vel.linear.y < -m_max_vel_trans) m_cmd_vel.linear.y = -m_max_vel_trans;
    if(m_cmd_vel.angular.z > m_max_vel_rot) m_cmd_vel.angular.z = m_max_vel_rot;
    if(m_cmd_vel.angular.z < -m_max_vel_rot) m_cmd_vel.angular.z = -m_max_vel_rot;

    m_last_cmd = m_cmd_vel;
    m_vel_pub.publish(m_cmd_vel);
    //ROS_WARN("[m_cmd_vel](%.2f %.2f):%.2f",m_cmd_vel.linear.x,m_cmd_vel.linear.y,m_cmd_vel.angular.z);
    return true;
}


void CPathFollower::getTransformedPosition(geometry_msgs::PoseStamped& pose, std::string& frame_id, double& x, double& y, double& theta)
{
    geometry_msgs::PoseStamped tf_pose;
    pose.header.stamp = ros::Time(0);
    m_tf_listener->transformPose(frame_id, pose, tf_pose);
    x = tf_pose.pose.position.x;
    y = tf_pose.pose.position.y,
    theta = tf::getYaw(tf_pose.pose.orientation);
}
