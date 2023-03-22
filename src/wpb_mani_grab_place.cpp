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

static ros::Publisher test_pub;
static std_msgs::String test_msg;

/*****************************************************************
* 返回抓取行为结果的回调函数
******************************************************************/
void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("done");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("货架抓取完成！开始放置！");
        test_msg.data = "pallet place";
        test_pub.publish(test_msg);
    }
}

/*****************************************************************
* 返回放置行为结果的回调函数
******************************************************************/
void PlaceResultCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("done");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("货架放置完成！结束！");
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wpb_mani_grab_place"); 

    ros::NodeHandle n;
    test_pub = n.advertise<std_msgs::String>("/wpb_mani/agent_test", 1);
    ros::Subscriber grab_res_sub = n.subscribe("wpb_mani/grab_result", 1, GrabResultCB);
    ros::Subscriber place_res_sub = n.subscribe("wpb_mani/place_result", 1, PlaceResultCB);

    ROS_WARN("[main] wpb_mani_grab_place");

    sleep(10);

    test_msg.data = "pallet grab";
    test_pub.publish(test_msg);

    ros::spin();

    return 0;
}