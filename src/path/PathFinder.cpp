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
#include "PathFinder.h"

CPathFinder::CPathFinder()
{
    m_bInitialized = false;
    m_tf_listener = NULL; 

}

CPathFinder::~CPathFinder()
{
    if(m_tf_listener != NULL)
        delete m_tf_listener;
}

void CPathFinder::Initialize()
{
    if(!m_bInitialized)
    {	
        // 发布和订阅话题
        ros::NodeHandle n;
        m_path_pub = n.advertise<geometry_msgs::PoseArray>("follow_path",10);

        std::string ns = n.getNamespace();
        // ROS_WARN("[CPathFinder] 命名空间 = %s",ns.c_str());
        m_global_frame_id = /*ns + */ "map";  
        m_robot_base_frame_id = /*ns + */"base_footprint";
        m_tf_listener = new tf::TransformListener;
        // 读取路径
        DIR *dir;
        struct dirent *ptr;
        char base[1000];
        std::string basePath =  GetPathDir();
        if ((dir=opendir(basePath.c_str())) == NULL)
        {
            ROS_WARN("[PathFinder] %s 路径文件夹打开失败 ... \n",basePath.c_str());
        }
        else
        {
            while ((ptr=readdir(dir)) != NULL)
            {
                if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
                    continue;
                else if(ptr->d_type == 8)    ///file
                {
                    std::string filename(ptr->d_name);
                    CPathHolder path_holder;
                    path_holder.LoadPathFromFile(basePath + "/" + filename);
                    arPathHolder.push_back(path_holder);
                }
                // else if(ptr->d_type == 8)    ///file
                //     printf("d_name:%s/%s\n",basePath.c_str(),ptr->d_name);
                // else if(ptr->d_type == 10)    ///link file
                //     printf("d_name:%s/%s\n",basePath.c_str(),ptr->d_name);
                // else if(ptr->d_type == 4)    ///dir
                // {
                //     memset(base,'\0',sizeof(base));
                //     strcpy(base,basePath.c_str());
                //     strcat(base,"/");
                //     strcat(base,ptr->d_name);
                // }
            }
            closedir(dir);
        }
        //////////////////
        // 显示加载的路径信息
        int  num_path = arPathHolder.size();
        ROS_WARN("arPathHolder 里有 %d 个对象",num_path);
        for(int i=0 ;i <num_path;i++)
        {
            int np = arPathHolder[i].arPoint.size();
            ROS_INFO("arPathHolder[%d] 有 %d 个点",i,np);
        }
        ///////////////////
        // 初始化完成
        m_bInitialized = true;
    }
}

void CPathFinder::getTransformedPosition(geometry_msgs::PoseStamped& pose, std::string& frame_id, double& x, double& y, double& theta)
{
    geometry_msgs::PoseStamped tf_pose;
    pose.header.stamp = ros::Time(0);
    m_tf_listener->transformPose(frame_id, pose, tf_pose);
    x = tf_pose.pose.position.x;
    y = tf_pose.pose.position.y,
    theta = tf::getYaw(tf_pose.pose.orientation);
}

std::string CPathFinder::GetPathDir()
{
    std::string strPathDir;
    char const* home = getenv("HOME");
    strPathDir = home;
    strPathDir += "/WPR_Path";
    return strPathDir;
}

int CPathFinder::GetPathIndex(float inSrcX, float inSrcY, float inTargetX, float inTargetY)
{
    float min_dist = 999999;
    int min_index = 0;
    int  num_path = arPathHolder.size();
    if(num_path == 0)
        return -1;
    for(int i=0 ;i <num_path;i++)
    {
        int last_index = arPathHolder[i].arPoint.size()-1;
        if( last_index > 0)
        {
            float diff_src_x = inSrcX - arPathHolder[i].arPoint[0].x;
            float diff_src_y = inSrcY - arPathHolder[i].arPoint[0].y;
            float diff_tar_x = inTargetX - arPathHolder[i].arPoint[last_index].x;
            float diff_tar_y = inTargetY - arPathHolder[i].arPoint[last_index].y;
            float dist = sqrt(diff_src_x * diff_src_x + diff_src_y * diff_src_y) + sqrt(diff_tar_x * diff_tar_x + diff_tar_y * diff_tar_y);
            if(dist < min_dist)
            {
                min_dist = dist;
                min_index = i;
            }
        }
    }
    return min_index;
}

void CPathFinder::PathTo(float inX, float inY, float inYaw)
{
    double self_x,self_y,self_yaw;
    geometry_msgs::PoseStamped self_pose;
    self_pose.header.frame_id=m_robot_base_frame_id;
    self_pose.pose.position.x = 0;
    self_pose.pose.position.y = 0;
    self_pose.pose.position.z = 0;
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0);
    quaternionTFToMsg(quat, self_pose.pose.orientation);
    getTransformedPosition(self_pose, m_global_frame_id, self_x, self_y, self_yaw);
    ROS_WARN("[CPathFinder::PathTo] 自身坐标 ( %.2f , %.2f)",self_x,self_y);
    ROS_WARN("[CPathFinder::PathTo] 目标坐标 ( %.2f , %.2f)",inX,inY);
    // 从路径集合中获取最佳路径
    int best_path_index = GetPathIndex(self_x,self_y,inX,inY);
    int nPointNum = arPathHolder[best_path_index].arPoint.size();
    ROS_WARN("[CPathFinder::PathTo] 最佳路径 %d  ( %.2f , %.2f)-> ( %.2f , %.2f)",
        best_path_index,
        arPathHolder[best_path_index].arPoint[0].x,
        arPathHolder[best_path_index].arPoint[0].y,
        arPathHolder[best_path_index].arPoint[nPointNum-1].x,
        arPathHolder[best_path_index].arPoint[nPointNum-1].y);

    path_msg.poses.clear();
    geometry_msgs::Pose pose;
    // 1、从机器人坐标到最佳路径起始之间进行离散放在开头 
    GenPath(self_x,self_y, arPathHolder[best_path_index].arPoint[0].x, arPathHolder[best_path_index].arPoint[0].y);

    // 2、将最佳路径插入中段
    for(int i=0;i<nPointNum;i++)
    {
        pose.position.x = arPathHolder[best_path_index].arPoint[i].x;
        pose.position.y = arPathHolder[best_path_index].arPoint[i].y;
        pose.orientation.x = 0;//1.0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    // 3、从最佳路径末端到目标点之间进行离散放在末尾
    GenPath(arPathHolder[best_path_index].arPoint[nPointNum-1].x, arPathHolder[best_path_index].arPoint[nPointNum-1].y,inX,inY);

    // 4、将目标点放置进去
    pose.position.x = inX;
    pose.position.y = inY;
    // tf::Quaternion quat;
    // 目标姿态,函数三个参数分别为滚转,俯仰和偏转角,单位为弧度
    quat.setRPY(0.0, 0.0, inYaw);
    // 将欧拉角旋转量转换成四元数表达
    tf::StampedTransform transform;
    transform.setRotation(quat);
    pose.orientation.x = transform.getRotation().getX();
    pose.orientation.y = transform.getRotation().getY();
    pose.orientation.z = transform.getRotation().getZ();
    pose.orientation.w = transform.getRotation().getW();
    path_msg.poses.push_back(pose);

    // 将路径点发送出去
    m_path_pub.publish(path_msg);
}

void CPathFinder::GenPath(float inSrcX, float inSrcY, float inDestX, float inDestY)
{
    float fDist = sqrt((inSrcX -inDestX) * (inSrcX -inDestX) + (inSrcY -inDestY) * (inSrcY -inDestY));
    int nStep = fDist/0.05;
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;//1.0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1.0;
    float fStep_x = (inDestX - inSrcX)/nStep;
    float fStep_y = (inDestY - inSrcY)/nStep;
    for(int i=1;i<=nStep;i++)
    {
        pose.position.x = inSrcX + fStep_x*i;
        pose.position.y = inSrcY + fStep_y*i;
        path_msg.poses.push_back(pose);
    }
}