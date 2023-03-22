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
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <wpr_rack_pkg/ObjectBox.h>
#include <wpr_rack_pkg/BoxParam.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/BatteryState.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <waterplus_map_tools/GetChargerByName.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include "RobotInfoSend.h"
#include "WPRAgent.h"
#include "PlanPathSend.h"
#include "PathHolder.h"
#include "PathFinder.h"
#include "WaypointHolder.h"

static int Robot_Dev = DEV_WPB_MANI;       //机器人设备类型

static float pallet_face_dist = 0.9;                    //与小型货架保持距离
static float pallet_grab_y_offset = 0.00;      //小型货架的抓取左右修正
static float pallet_grab_z_offset = 0.00;        //小型货架的抓取高度修正

static float pallet_place_y_offset = 0.00;        //小型货架放置动作的左右修正
static float pallet_place_z_offset = 0.00;        //小型货架放置动作的高度修正

static float pallet_center_x = pallet_face_dist;
static float pallet_center_y = 0.0;
static float pallet_center_z = 0.25;

static float max_vx = 0.2;  // 前后移动的速度上限
static float max_vy = 0.2;  // 左右移动的速度上限
static float max_vth = 1.0;   // 旋转速度上限

static int Robot_ID = 1;                                        //机器人ID（建议在 agent.yaml 配置文件中修改）
static string ServerIP = "127.0.0.1";               //服务器IP地址（建议在 agent.yaml 配置文件中修改）

// 其他类型货架的操作参数（未使用）
static float mobile_face_dist = 1.20;       //与启程3移动货架对准距离
static float mobile_place_x = 1.10;            //对启程3移动货架放置的前后距离
static float mobile_place_y = 0.0;            //对启程3移动货架放置的左右偏移
static float mobile_place_z = 0.885;         //对启程3移动货架放置的垂直高度
static float mobile_grab_x_offset = -0.03;
static float mobile_grab_y_offset = 0.0;
static float mobile_grab_z = 0.88;

static float storage_face_dist = 1.25;              //与存储货架对准距离
static float storage_place_y_offset = 0.0;         //对存储货架放置的左右偏移
static float storage_place_x_offset = 0.0;         //对存储货架放置的前后距离修正量
static float storage_place_z_offset = -0.02;      //对存储货架放置的垂直高度修正量
static float storage_grab_x_offset = -0.03;     //存储货架的抓取前后修正
static float storage_grab_y_offset = -0.03;     //存储货架的抓取左右修正
static float storage_grab_z_offset = 0.0;         //存储货架的抓取高度修正

static std::string name_space = "";
static bool sim_flag = false;

static int path_index = 0;
static ros::Publisher path_pub;
static geometry_msgs::PoseArray path_msg;
static ros::Publisher move_pub;

static CWPRAgent wpr_agent;
static CRobotInfoSend robot_info_send;
static int nRobotState = RBT_ST_STOP;
static stCommandMsg cmd_recv;
static CPlanPathSend plan_path_send;

static geometry_msgs::Pose navi_msg;
static tf::Quaternion quat;
static ros::Publisher behavior_pub;
static std_msgs::String behavior_msg;

static bool bNaviDone = false;

static ros::Publisher rack_pub;
static std_msgs::String rack_msg;
static ros::Publisher grab_action_pub;
static geometry_msgs::Pose grab_action_msg;
static ros::Publisher place_pub;
static geometry_msgs::Pose place_msg;
static ros::Publisher odom_ctrl_pub;
static std_msgs::String odom_ctrl_msg;
static geometry_msgs::Pose2D pose_diff;
static ros::Publisher vel_pub;
static ros::Publisher navi_cancel_pub;
static actionlib_msgs::GoalID cancel_msg;
static ros::Publisher mani_ctrl_pub;
static sensor_msgs::JointState mani_ctrl_msg;
static bool bGrabbing = false;
static ros::Publisher pose_init_pub;
static std_msgs::String pose_init_msg;
static ros::Publisher relocation_pub;
static geometry_msgs::PoseWithCovarianceStamped relocation_msg; 

std::vector<geometry_msgs::Pose> arPlacePose;

static float face_rack_x = 0;
static float face_rack_y = 0;
static float face_rack_th = 0;

static tf::TransformListener* m_tf_listener = NULL; 
static std::string m_robot_base_frame_id = "base_footprint";
static ros::Publisher marker_pub;
static visualization_msgs::Marker text_marker;
static ros::Publisher team_mate_pub;

static float robot_battery = 0;
static int relocation_count = 0;
static float move_x = 0;
static float move_y = 0;
static float move_yaw = 0;

CPathFinder path_finder;
CWaypointHolder wp_holder;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void NaviResultCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = msg->data.find("done");
    if( nFindIndex >= 0 )
    {
        bNaviDone = true;
    }
}

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

void SetRobotPose(float inX, float inY, float inYaw)
{
    relocation_msg.header.stamp = ros::Time::now();
    relocation_msg.header.frame_id = "map";
    //position
     relocation_msg.pose.pose.position.x = inX;
     relocation_msg.pose.pose.position.y = inY;
     relocation_msg.pose.pose.position.z = 0;
    //yaw
    tf::Quaternion quat_pose = tf::createQuaternionFromYaw(inYaw);
    //quat_pose.setRPY(0.0, 0.0, inYaw);
    relocation_msg.pose.pose.orientation.x = quat_pose.getX();
    relocation_msg.pose.pose.orientation.y = quat_pose.getY();
    relocation_msg.pose.pose.orientation.z = quat_pose.getZ();
    relocation_msg.pose.pose.orientation.w = quat_pose.getW();
    //publish msg
    relocation_pub.publish(relocation_msg);
}

void getTransformedPosition(geometry_msgs::PoseStamped& pose, std::string& frame_id, double& x, double& y, double& theta)
{
    geometry_msgs::PoseStamped tf_pose;
    pose.header.stamp = ros::Time(0);
    m_tf_listener->transformPose(frame_id, pose, tf_pose);
    x = tf_pose.pose.position.x;
    y = tf_pose.pose.position.y,
    theta = tf::getYaw(tf_pose.pose.orientation);
}

void MoveTo(float inX , float inY , float inYaw)
{
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id="map";
    target_pose.pose.position.x = inX;
    target_pose.pose.position.y = inY;
    target_pose.pose.position.z = 0;
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, inYaw);
    quaternionTFToMsg(quat, target_pose.pose.orientation);
    double local_x ,local_y,local_yaw;
    getTransformedPosition(target_pose, m_robot_base_frame_id, local_x ,local_y,local_yaw);
    //ROS_WARN("[MoveTo] = ( %.2f , %.2f ) %.2f",local_x ,local_y,local_yaw);

    geometry_msgs::PoseStamped move_msg;
    move_msg.pose.position.x = local_x;
    move_msg.pose.position.y = local_y;
    move_msg.pose.position.z = 0;
    tf::Quaternion local_quat;
    local_quat.setRPY(0.0, 0.0, local_yaw);
    quaternionTFToMsg(local_quat, move_msg.pose.orientation);
    move_pub.publish(move_msg);
}

void SendFollowPath(int inPathIndex)
{
    if(inPathIndex < 0)
    {
        nRobotState = RBT_ST_FOLLOW_PATH_END;
        return;
    }
    path_pub.publish(path_msg);
}

void StopFollowPath()
{
    path_msg.poses.clear();
    path_pub.publish(path_msg);
}

void PathFollowResultCB(const std_msgs::String::ConstPtr &msg)
{
    if( nRobotState == RBT_ST_FOLLOW_PATH)
    {
        int nFindIndex = msg->data.find("done");
        if( nFindIndex >= 0 )
        {
            nRobotState = RBT_ST_FOLLOW_PATH_END;
        }
    }
    if( nRobotState == RBT_ST_SRC_MOVE)
    {
        int nFindIndex = msg->data.find("done");
        if( nFindIndex >= 0 )
        {
            //---------------------------------------------------
            // [1]循环移动，测试用
            wp_holder.GotoNextWaypoint();
            //---------------------------------------------------
            // [2] 检测货架，接着检测物品
            // rack_msg.data = "detect storage rack";
            // rack_pub.publish(rack_msg);
            // nRobotState = RBT_ST_SRC_DETECT;
        }
    }
}

void StopAll()
{
    behavior_msg.data = "grab stop";
    behavior_pub.publish(behavior_msg);
    behavior_msg.data = "place stop";
    behavior_pub.publish(behavior_msg);
    navi_cancel_pub.publish(cancel_msg);
    StopFollowPath();
}

void CalFaceRack(float inX, float inY, float inAngle, float inFaceDist)
{
    face_rack_x = inX - inFaceDist*cos(inAngle);
    face_rack_y = inY - inFaceDist*sin(inAngle);
    face_rack_th = inAngle;
}

void PathToNextWaypoint()
{

}

float CalDist(float inX_1, float inY_1, float inZ_1, float inX_2, float inY_2, float inZ_2)
{
    float dist = sqrt(pow(inX_1 - inX_2,2) + pow(inY_1 - inY_2,2) + pow(inZ_1 - inZ_2,2));
    return dist;
}

/*****************************************************************
* 返回货架位置的回调函数
******************************************************************/
void RackPositionCB(const geometry_msgs::Pose::ConstPtr &msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if(nRobotState == RBT_ST_PALLET_DETECT)
    {
        ROS_WARN("[RackPositionCB] pallet pos (%.2f , %.2f): %.2f",msg->position.x,msg->position.y,yaw);
        //位置比较正的时候启动空位检测，否则继续调整
        float dist_diff = fabs(msg->position.x-pallet_face_dist);
        if(dist_diff <0.1 && msg->position.y> -0.1 && msg->position.y< 0.1 && fabs(yaw) < 0.08)
        {
            rack_msg.data = "detect object";
            rack_pub.publish(rack_msg);
            nRobotState = RBT_ST_PALLET_MEASURE;
            ROS_WARN("nRobotState = RBT_ST_PALLET_MEASURE ");
        }
        else
        {
            CalFaceRack(msg->position.x,msg->position.y,yaw, pallet_face_dist );
            ROS_WARN("[RackPositionCB] face pallet pos (%.2f , %.2f): %.2f",face_rack_x,face_rack_y,face_rack_th);
            odom_ctrl_msg.data = "pose_diff reset";
            odom_ctrl_pub.publish(odom_ctrl_msg);
            nRobotState = RBT_ST_PALLET_F_MOVE;
        }
    }

    if(nRobotState == RBT_ST_GP_DETECT)
    {
        ROS_WARN("[RackPositionCB] GP rack pos (%.2f , %.2f): %.2f",msg->position.x,msg->position.y,yaw);
        //位置比较正的时候启动放置行为，否则继续对准
        float dist_diff = fabs(msg->position.x-pallet_face_dist);
        if(dist_diff <0.1 && msg->position.y> -0.1 && msg->position.y<0.1 && fabs(yaw) < 0.08)
        {
            rack_msg.data = "detect object";
            rack_pub.publish(rack_msg);
            nRobotState = RBT_ST_GP_OBJECT;
        }
        else
        {
            CalFaceRack(msg->position.x,msg->position.y,yaw, pallet_face_dist );
            ROS_WARN("[RackPositionCB] face pallet pos (%.2f , %.2f): %.2f",face_rack_x,face_rack_y,face_rack_th);
            odom_ctrl_msg.data = "pose_diff reset";
            odom_ctrl_pub.publish(odom_ctrl_msg);
            nRobotState = RBT_ST_GP_F_MOVE;
        }
    }

    if(nRobotState == RBT_ST_SRC_DETECT)
    {
        ROS_WARN("[RackPositionCB] Search Obj rack pos (%.2f , %.2f): %.2f",msg->position.x,msg->position.y,yaw);
        //只是检测有没有物品，不需要对太准，直接检测
        rack_msg.data = "detect object";
        rack_pub.publish(rack_msg);
        nRobotState = RBT_ST_SRC_OBJ;
       
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
     if(nRobotState == RBT_ST_GP_OBJECT)
    {
        // 挑选最靠中间的抓取
        int nNumObj = msg->name.size();
        ROS_WARN("[RBT_ST_GP_OBJECT] 得到物品坐标个数 =  %d",nNumObj);
         if(nNumObj > 0)
        {
            int nGrabIndex = 0;
            float fMinDist = 100;
            for(int i=0;i<nNumObj;i++)
            {
                float obj_x = msg->xMin[i];
                float obj_y = (msg->yMin[i] + msg->yMax[i])/2 + pallet_grab_y_offset;
                float obj_z = msg->zMin[i] + pallet_grab_z_offset;
                ROS_WARN("[Grab] Obj_%d (%.2f , %.2f , %.2f)",i,obj_x,obj_y,obj_z);
                
                float obj_dist = CalDist(obj_x, obj_y, obj_z, pallet_center_x , pallet_center_y, pallet_center_z);
                if(obj_dist < fMinDist)
                {
                    fMinDist = obj_dist;
                    nGrabIndex = i;
                }
            }
            float obj_x = msg->xMin[nGrabIndex];
            float obj_y = (msg->yMin[nGrabIndex] + msg->yMax[nGrabIndex])/2 + pallet_grab_y_offset;
            float obj_z = msg->zMin[nGrabIndex] + pallet_grab_z_offset;
            ROS_WARN("[Grab] Obj_%d (%.2f , %.2f , %.2f)",nGrabIndex,obj_x,obj_y,obj_z); 
            grab_action_msg.position.x = obj_x;
            grab_action_msg.position.y = obj_y;
            grab_action_msg.position.z = obj_z;
            grab_action_pub.publish(grab_action_msg);
            nRobotState = RBT_ST_GP_GRAB;
        }
        else
        {
            // 货架上没有物品，是不是该去下一个地点
        }
    }

    // 物品搜索，有物品的话直接进入抓取
     if(nRobotState == RBT_ST_SRC_OBJ)
    {
        // 挑选最靠中间的抓取
        int nNumObj = msg->name.size();
        ROS_WARN("[RBT_ST_SRC_OBJ] 得到物品坐标个数 =  %d",nNumObj);
         if(nNumObj > 0)
        {
            // 有物品，进入正式的抓取
            rack_msg.data = "detect storage rack";
            rack_pub.publish(rack_msg);
            nRobotState = RBT_ST_GS_DETECT;
        }
        else
        {
            wp_holder.GotoNextWaypoint();
            nRobotState = RBT_ST_SRC_MOVE;
        }
    }
}
/*****************************************************************
* 返回放置位置的回调函数
******************************************************************/
void PlacePositionCB(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    int nNumPlace = msg->poses.size();
    ROS_WARN("[PlacePositionCB] 可放置的位置 = %d",nNumPlace);
    arPlacePose.clear();
    geometry_msgs::Pose pose;
    for(int i=0;i<nNumPlace;i++)
    {
        pose.position = msg->poses[i].position;
        arPlacePose.push_back(pose);
    }
    if(nRobotState == RBT_ST_PALLET_MEASURE)
    {
        if( nNumPlace > 0)
        {
            int nPlaceIndex = 0;
            float fMinDist = 100;
            for(int i=0;i<nNumPlace;i++)
            {
                float place_x = arPlacePose[i].position.x;
                float place_y = arPlacePose[i].position.y;
                float place_z = arPlacePose[i].position.z;
                ROS_INFO("[PlacePositionCB] 放置_%d (%.2f , %.2f , %.2f)",i,place_x,place_y,place_z);
                
                float place_dist = CalDist(place_x, place_y, place_z, pallet_center_x , pallet_center_y, pallet_center_z);
                if(place_dist < fMinDist)
                {
                    fMinDist = place_dist;
                    nPlaceIndex = i;
                }
            }
            place_msg.position.x = arPlacePose[nPlaceIndex].position.x;  //放置的前后距离
            place_msg.position.y = arPlacePose[nPlaceIndex].position.y + pallet_place_y_offset;  //放置的左右偏移量
            place_msg.position.z = arPlacePose[nPlaceIndex].position.z + pallet_place_z_offset;  //放置的高度
            place_pub.publish(place_msg);
            ROS_WARN("[PlacePositionCB] 最终放置_%d (%.2f , %.2f , %.2f)",nPlaceIndex, place_msg.position.x, place_msg.position.y, place_msg.position.z);
            nRobotState = RBT_ST_PALLET_PLACE;
        }
    }
}
/*****************************************************************
* grab_action 的执行结果
******************************************************************/
void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[GrabResultCB] %s",msg->data.c_str());
    if(nRobotState == RBT_ST_BOX_GRAB)
    {
        int nFindIndex = 0;
        nFindIndex = msg->data.find("done");
        if( nFindIndex >= 0 )
        {
            ROS_WARN("box grab done!");
            nRobotState = RBT_ST_BOX_DONE;
        }
    }

    if(nRobotState == RBT_ST_GM_GRAB)
    {
        int nFindIndex = 0;
        nFindIndex = msg->data.find("done");
        if( nFindIndex >= 0 )
        {
            ROS_WARN("移动货架抓取完成 !");
            nRobotState = RBT_ST_GM_DONE;
        }
    }

    if(nRobotState == RBT_ST_GS_GRAB)
    {
        int nFindIndex = 0;
        nFindIndex = msg->data.find("done");
        if( nFindIndex >= 0 )
        {
            ROS_WARN("储存货架抓取完成 !");
            nRobotState = RBT_ST_GS_DONE;
        }
    }

    if(nRobotState == RBT_ST_GP_GRAB)
    {
        int nFindIndex = 0;
        nFindIndex = msg->data.find("done");
        if( nFindIndex >= 0 )
        {
            ROS_WARN("小型货架抓取完成 !");
            nRobotState = RBT_ST_GP_DONE;
        }
    }
}

/*****************************************************************
* 返回放置位置的回调函数
******************************************************************/
void PlaceResultCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[PlaceResultCB] %s",msg->data.c_str());
    if(nRobotState == RBT_ST_PALLET_PLACE)
    {
        int nFindIndex = 0;
        nFindIndex = msg->data.find("done");
        if( nFindIndex >= 0 )
        {
            ROS_WARN("agent_node 接收到 Pallet货架放置完成消息!");
            nRobotState = RBT_ST_PALLET_DONE;
        }
    }
}
/*****************************************************************
* 返回全局路径的回调函数
******************************************************************/
void PlanPathCB(const nav_msgs::Path::ConstPtr &msg)
{
    int nPathPoseNum = msg->poses.size();
    if(nPathPoseNum < 100)
    {
        plan_path_send.path_msg.len = nPathPoseNum;
        for(int i=0;i<nPathPoseNum;i++)
        {
            plan_path_send.path_msg.path_x[i] = msg->poses[i].pose.position.x;
            plan_path_send.path_msg.path_y[i] = msg->poses[i].pose.position.y;
        }
    }
    else
    {
        float step = (float)nPathPoseNum/100;
        plan_path_send.path_msg.len = 100;
        for(int i=0;i<100;i++)
        {
            int pose_index = (int)(i*step);
            plan_path_send.path_msg.path_x[i] = msg->poses[pose_index].pose.position.x;
            plan_path_send.path_msg.path_y[i] = msg->poses[pose_index].pose.position.y;
        }
    }
    plan_path_send.SendPath();
}
/*****************************************************************
* 返回关节信息的回调函数
******************************************************************/
void JointStatesCB(const sensor_msgs::JointState::ConstPtr &msg)
{
    int nNumJoint = msg->position.size();
    // ROS_WARN("nNumJoint =  %d",nNumJoint);
    if(nNumJoint > 10) nNumJoint = 10;
    for(int i=0;i<nNumJoint;i++)
    {
        // ROS_WARN("joint %d name=  %s",i,msg->name[i].c_str());
        robot_info_send.info_msg.joint_pos[i] = msg->position[i];
    }
}
/*****************************************************************
* 返回激光雷达数据的回调函数
******************************************************************/
static float arRanges[1081];
void LidarCB(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    for(int i=0;i<1081;i++)
    {
        arRanges[i] = scan->ranges[i] ;
    }
}

void DrawRobotID(int inRobotID, std::string inBase)
{
    text_marker.header.frame_id = inBase;
    text_marker.ns = "id";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = inRobotID;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = 0.2;
    text_marker.color.r = 1.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = 0;
    text_marker.pose.position.y = 0;
    text_marker.pose.position.z = 0.8;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(1.0);

    std::ostringstream stringStream;
    stringStream <<"ID = "<< inRobotID;
    std::string retStr = stringStream.str();
    text_marker.text = retStr;

    marker_pub.publish(text_marker);
}

/*****************************************************************
* 接收到测试指令的回调函数
******************************************************************/
void AgentTestCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[AgentTestCB] recv %s",msg->data.c_str());
    int nFindIndex = 0;
    nFindIndex = msg->data.find("pallet grab");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("pallet grab test!");
        wpr_agent.cmd_new.command = CMD_GRAB_PALLET;
        wpr_agent.bNewCmd = true;
    }
    nFindIndex = msg->data.find("pallet place");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("pallet place test!");
        wpr_agent.cmd_new.command = CMD_PLACE_PALLET;
        wpr_agent.bNewCmd = true;
    }
    nFindIndex = msg->data.find("goto");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("goto test!");
        wpr_agent.cmd_new.command = RBT_ST_GOTO;
        wpr_agent.cmd_new.map_x = 0.09;
        wpr_agent.cmd_new.map_y = 0.06;
        wpr_agent.cmd_new.map_yaw = -0.03;
        wpr_agent.bNewCmd = true;
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wpb_mani_agent_node");

    path_finder.Initialize();
    wp_holder.Initialize();
    wp_holder.pPF = &path_finder;

    ros::NodeHandle n_param("~");
    n_param.getParam("robot_id", Robot_ID);
    ROS_WARN("[wpb_mani_agent_node] 机器人ID = %d",Robot_ID);
    n_param.getParam("server_ip", ServerIP);
    ROS_WARN("[wpb_mani_agent_node] 服务器IP = %s",ServerIP.c_str());
    m_tf_listener = new tf::TransformListener;
    
    MoveBaseClient ac("move_base", true);
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        if(!ros::ok())
            break;
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ros::NodeHandle n;
    ros::Publisher navi_pub = n.advertise<geometry_msgs::Pose>( "navi_pose", 10);
    ros::Subscriber navi_result_sub = n.subscribe("navi_result", 10, NaviResultCB);
    behavior_pub = n.advertise<std_msgs::String>("wpb_mani/behaviors", 30);
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    rack_pub = n.advertise<std_msgs::String>("rack/command", 10);
    grab_action_pub = n.advertise<geometry_msgs::Pose>("wpb_mani/grab_box", 1);
    ros::Subscriber grab_res_sub = n.subscribe("wpb_mani/grab_result", 30, GrabResultCB);
    place_pub = n.advertise<geometry_msgs::Pose>("wpb_mani/place_action", 1);
    ros::Subscriber place_res_sub = n.subscribe("wpb_mani/place_result", 30, PlaceResultCB);
    ros::Subscriber rack_sub = n.subscribe("rack/rack_position", 10, RackPositionCB);
    ros::Subscriber obj_sub = n.subscribe("rack/obj_position", 10, ObjectsBoxCB);
    ros::Subscriber place_sub = n.subscribe("rack/place_positon", 10, PlacePositionCB);
    odom_ctrl_pub = n.advertise<std_msgs::String>("wpb_mani/ctrl", 10);
    ros::Subscriber pose_diff_sub = n.subscribe("wpb_mani/pose_diff", 1, PoseDiffCallback);
    mani_ctrl_pub = n.advertise<sensor_msgs::JointState>("wpb_mani/joint_ctrl", 30);
    relocation_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 5, true);
    ros::Subscriber path_sub = n.subscribe("move_base/GlobalPlanner/plan", 1, PlanPathCB);
    ros::Subscriber test_sub = n.subscribe( "wpb_mani/agent_test", 30, AgentTestCB);
    ros::Subscriber joint_states_sub = n.subscribe("joint_states", 10, JointStatesCB);
    navi_cancel_pub = n.advertise<actionlib_msgs::GoalID>( "move_base/cancel",1);
    pose_init_pub = n.advertise<std_msgs::String>("map_cali_cmd", 1);
    ros::Subscriber scan_sub = n.subscribe( "scan",30,LidarCB);
    ros::Subscriber path_follow_result_sub = n.subscribe("follow_path_result", 1, PathFollowResultCB);
    ros::Subscriber path_follow_display_sub = n.subscribe("follow_path_display", 1, PlanPathCB);
    path_pub = n.advertise<geometry_msgs::PoseArray>("follow_path", 2);
    move_pub = n.advertise<geometry_msgs::PoseStamped>("wpb_mani/move", 10);
    marker_pub = n.advertise<visualization_msgs::Marker>("robot_id", 10);
    team_mate_pub = n.advertise<geometry_msgs::PoseArray>("team_mate_pose", 10);
    name_space = n.getNamespace();
    ROS_WARN("[wpb_mani_agent_node] 命名空间 = %s",name_space.c_str());

    int nPort = 20201;
    wpr_agent.InitUDPServer(nPort);
    ROS_WARN("[wpb_mani_agent_node] 监听端口 = %d",nPort);
    robot_info_send.SetID(Robot_ID);
    robot_info_send.SetDevType(Robot_Dev);
    robot_info_send.Initial();
    robot_info_send.InitUDPClient(ServerIP.c_str(),20202);
    plan_path_send.SetID(Robot_ID);
    plan_path_send.SetDevType(Robot_Dev);
    plan_path_send.InitUDPClient(ServerIP.c_str(),20203);

    ros::Rate r(30.0);
    while(ros::ok())
    {
        /*******************
         * 获取服务器发来的指令
         *******************/
        if(wpr_agent.bNewCmd == true)
        {
            StopAll();
            memcpy(&cmd_recv,&(wpr_agent.cmd_new),sizeof(stCommandMsg));
            switch (cmd_recv.command)
            {
            case CMD_STOP:
                behavior_msg.data = "move stop";
                behavior_pub.publish(behavior_msg);
                nRobotState = RBT_ST_STOP;
                break;
            case CMD_ROBOT_MOVE:
                nRobotState = RBT_ST_MOVE;
                move_x = cmd_recv.map_x;
                move_y = cmd_recv.map_y;
                move_yaw = cmd_recv.map_yaw;
                MoveTo(move_x ,move_y , move_yaw);
                // ROS_WARN("[wpr_agent - CMD_ROBOT_MOVE] = ( %.2f , %.2f ) %.2f",move_x ,move_y , move_yaw);
                // SendTeamMatePose(&cmd_recv);
                break;
            case CMD_ROBOT_GOTO:
                ROS_WARN("收到 CMD_ROBOT_GOTO ( %.2f , %.2f )  %.2f",cmd_recv.map_x,cmd_recv.map_y,cmd_recv.map_yaw);
                nRobotState = RBT_ST_GOTO;
                navi_msg.position.x = cmd_recv.map_x;
                navi_msg.position.y = cmd_recv.map_y;
                quat.setRPY(0.0, 0.0, cmd_recv.map_yaw);
                navi_msg.orientation.x = quat.getX();
                navi_msg.orientation.y = quat.getY();
                navi_msg.orientation.z = quat.getZ();
                navi_msg.orientation.w = quat.getW();
                navi_pub.publish(navi_msg);
                bNaviDone = false;
                break;
            case CMD_ROBOT_POSE:
                ROS_WARN("收到 CMD_ROBOT_POSE ( %.2f , %.2f )  %.2f",cmd_recv.map_x,cmd_recv.map_y,cmd_recv.map_yaw);
                SetRobotPose(cmd_recv.map_x,cmd_recv.map_y,cmd_recv.map_yaw);
                break;
            case CMD_ROBOT_TELEOP:
                nRobotState = RBT_ST_TELEOP;
                break;
            case CMD_PLACE_PALLET:
                rack_msg.data = "detect pallet rack";
                rack_pub.publish(rack_msg);
                nRobotState = RBT_ST_PALLET_DETECT;
                break;
            case CMD_GRAB_PALLET:
                rack_msg.data = "detect pallet rack";
                rack_pub.publish(rack_msg);
                nRobotState = RBT_ST_GP_DETECT;
                break;
            case CMD_FOLLOW_PATH:
                path_index = cmd_recv.path_index;
                SendFollowPath(path_index);
                nRobotState = RBT_ST_FOLLOW_PATH;
                break;
            case CMD_PATH_TO:
                ROS_WARN("收到 CMD_PATH_TO  ( %.2f , %.2f )",cmd_recv.map_x,cmd_recv.map_y);
                path_finder.PathTo(cmd_recv.map_x,cmd_recv.map_y,cmd_recv.map_yaw);
                nRobotState = RBT_ST_FOLLOW_PATH;
                break;
            case CMD_SEARCH_OBJ:
                ROS_WARN("收到 CMD_SEARCH_OBJ ");
                printf("搜索航点：");
                for(int i=0; i< 10; i++)
                {
                    wp_holder.arWaypoint[i] = cmd_recv.search_waypoint[i];
                    printf(" %d", wp_holder.arWaypoint[i]);
                }
                printf("\n");
                wp_holder.nIndex = 0;
                wp_holder.GotoNextWaypoint();
                nRobotState = RBT_ST_SRC_MOVE;
                break;
            default:
                break;
            }
            
            wpr_agent.bNewCmd = false;
        }
        
         /*******************
         * 机器人行为状态
         *******************/
        if(nRobotState == RBT_ST_STOP)
        {
            VelCmd(0,0,0);
        }

        // [1] 导航
        if(nRobotState == RBT_ST_GOTO)
        {
            if(bNaviDone == true)
            {
                nRobotState = RBT_ST_ARRIVED;
            }
        }

        if(nRobotState == RBT_ST_ARRIVED)
        {
        }
 
        // 遥控
        if(nRobotState == RBT_ST_TELEOP)
        {
            VelCmd(cmd_recv.vel_x,cmd_recv.vel_y,cmd_recv.vel_angular);
        }

        // 直线移动
        if(nRobotState == RBT_ST_MOVE)
        {
            MoveTo(move_x ,move_y , move_yaw);
        }
        // 跟随者
        if(nRobotState == RBT_ST_FOLLOWER)
        {
            MoveTo(move_x ,move_y , move_yaw);
        }

        // 小货架的料盒放置
        if(nRobotState == RBT_ST_PALLET_DETECT)
        {
            VelCmd(0,0,0);
        }

        if(nRobotState == RBT_ST_PALLET_F_MOVE)
        {
            float vx,vy;
            vx = (face_rack_x - pose_diff.x)/2;
            vy = (face_rack_y - pose_diff.y)/2;
            // 速度太快的话需要限速
            if(vx > max_vx) vx=max_vx;
            if(vx < -max_vx) vx = -max_vx;
            if(vy > max_vy) vy=max_vy;
            if(vy < -max_vy) vy= -max_vy;
            //ROS_INFO("[STEP_FACE_MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,face_rack_x, face_rack_y, pose_diff.x ,pose_diff.y,vx,vy);
            if(fabs(vx) <= 0.01 && fabs(vy) <= 0.01)
            {
                VelCmd(0,0,0);
                nRobotState = RBT_ST_PALLET_F_ROT;
                ROS_INFO("nRobotState = RBT_ST_PALLET_F_ROT");
            }
            else
            {
                VelCmd(vx,vy,0);
            }
        }

        if(nRobotState == RBT_ST_PALLET_F_ROT)
        {
            float vth = (face_rack_th - pose_diff.theta);
            // 速度太快就限制一下
            if(vth > max_vth) vth = max_vth;
            if(vth < -max_vth) vth = -max_vth;
            if(fabs(vth) < 0.01)
            {
                VelCmd(0,0,0);
                odom_ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(odom_ctrl_msg);
                // 让rack_detect_node再次启动货架检测
                rack_msg.data = "detect pallet rack";
                rack_pub.publish(rack_msg);
                nRobotState = RBT_ST_PALLET_DETECT;
                ROS_INFO("nRobotState = RBT_ST_PALLET_DETECT");
            }
            else
            {
                VelCmd(0,0,vth);
            }
        }

        if(nRobotState == RBT_ST_PALLET_MEASURE)
        {
            VelCmd(0,0,0);
        }

        if(nRobotState == RBT_ST_PALLET_PLACE)
        {
        }

        if(nRobotState == RBT_ST_PALLET_DONE)
        {
        }

        if(nRobotState == RBT_ST_GP_F_MOVE)
        {
            float vx,vy;
            vx = (face_rack_x - pose_diff.x)/2;
            vy = (face_rack_y - pose_diff.y)/2;
            // 速度太快的话需要限速
            if(vx > max_vx) vx=max_vx;
            if(vx < -max_vx) vx = -max_vx;
            if(vy > max_vy) vy=max_vy;
            if(vy < -max_vy) vy= -max_vy;
            //ROS_INFO("[STEP_FACE_MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,face_rack_x, face_rack_y, pose_diff.x ,pose_diff.y,vx,vy);
            if(fabs(vx) <= 0.01 && fabs(vy) <= 0.01)
            {
                VelCmd(0,0,0);
                nRobotState = RBT_ST_GP_F_ROT;
                ROS_INFO("nRobotState = RBT_ST_GP_F_ROT");
            }
            else
            {
                VelCmd(vx,vy,0);
            }
        }

        if(nRobotState == RBT_ST_GP_F_ROT)
        {
            float vth = (face_rack_th - pose_diff.theta);
            // 速度太快就限制一下
            if(vth > max_vth) vth = max_vth;
            if(vth < -max_vth) vth = -max_vth;
            if(fabs(vth) < 0.01)
            {
                VelCmd(0,0,0);
                odom_ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(odom_ctrl_msg);
                // 让rack_detect_node再次启动货架检测
                rack_msg.data = "detect pallet rack";
                rack_pub.publish(rack_msg);
                nRobotState = RBT_ST_GP_DETECT;
                ROS_INFO("nRobotState = RBT_ST_GP_DETECT");
            }
            else
            {
                VelCmd(0,0,vth);
            }
        }

        if(nRobotState == RBT_ST_GP_GRAB)
        {
        }

        if(nRobotState == RBT_ST_GP_DONE)
        {
        }

        if(nRobotState == RBT_ST_TELEOP)
        {
            VelCmd(cmd_recv.vel_x,cmd_recv.vel_y,cmd_recv.vel_angular);
        }
        

        /*******************
         * 发送机器人新状态 
         *******************/
        robot_info_send.GetMapPosition();
        robot_info_send.info_msg.state = nRobotState;
        robot_info_send.info_msg.cmd_recv = cmd_recv.command;
        robot_info_send.info_msg.battery = robot_battery;
        robot_info_send.SendInfo();

        DrawRobotID(Robot_ID,  "base_footprint");

        ros::spinOnce();
        r.sleep();
    }
    delete m_tf_listener;
    return 0;
}