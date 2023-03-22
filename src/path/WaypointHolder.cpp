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
#include "WaypointHolder.h"

CWaypointHolder::CWaypointHolder()
{
    for(int i=0;i<10;i++)
        arWaypoint[10] = 0;
    nIndex = 0;
}

CWaypointHolder::~CWaypointHolder()
{
    
}

void CWaypointHolder::Initialize()
{
    if(!m_bInitialized)
    {	
        // 发布和订阅话题
        ros::NodeHandle n;
        cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("waterplus/get_waypoint_name");
        // 初始化完成
        m_bInitialized = true;
    }
}

bool CWaypointHolder::Goto(std::string inName)
{
    float x,y,yaw;
    waterplus_map_tools::GetWaypointByName srvN;
    srvN.request.name = inName;
    if (cliGetWPName.call(srvN))
    {
        std::string name = srvN.response.name;
        x = srvN.response.pose.position.x;
        y = srvN.response.pose.position.y;
        tf::Quaternion quat(srvN.response.pose.orientation.x,srvN.response.pose.orientation.y,srvN.response.pose.orientation.z,srvN.response.pose.orientation.w);
         yaw = tf::getYaw(quat);
        ROS_INFO("Get_wp_name: name = %s (%.2f,%.2f)  - %.2f", inName.c_str(),x,y,yaw);
    }
    else
    {
        ROS_ERROR("Failed to call service get_waypoint_name");
        return false;
    }
    pPF->PathTo(x,y,yaw);
    return true;
}
   
 void CWaypointHolder::GotoNextWaypoint()
 {
    while(arWaypoint[nIndex] == 0)
    {
        nIndex ++;
        if(nIndex >= 10)
            nIndex = 0;
    }
    std::ostringstream stringStream;
    stringStream << arWaypoint[nIndex] ;
    std::string strName = stringStream.str();
    Goto(strName);
    ROS_WARN("[CWaypointHolder] 去往航点 %s",strName.c_str());
    nIndex ++;
 }