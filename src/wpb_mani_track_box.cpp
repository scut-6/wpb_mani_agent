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
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/Image.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <wpb_mani_behaviors/Coord.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>

// 默认检测范围
static float filter_x_max = 1.5;
static float filter_x_min = 0.3;
static float filter_y_max = 0.75;
static float filter_y_min = -0.75;
static float filter_z_max = 0.5;
static float filter_z_min = 0.1;

// 工作模式
#define MODE_IDLE                   0
#define MODE_BOX_TRACK    2
static int nMode = MODE_IDLE;

//#define  KD_TREE_M

using namespace std;

static std::string pc_topic;
static bool start_flag = false;
static ros::Publisher pc_pub;
static ros::Publisher marker_pub;
static ros::Publisher coord_pub;
static tf::TransformListener *tf_listener; 
void DrawBox(ros::Publisher* inPub, int inID, float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB);
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void DrawPath(float inX, float inY, float inZ);
void RemoveBoxes();
void SortObjectsInTrack();
static visualization_msgs::Marker line_box;
static visualization_msgs::Marker line_plane;
static visualization_msgs::Marker text_marker;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
static ros::Publisher segmented_boxes;
static bool bPlaneHeightAssigned = false;

typedef struct stBoxMarker
{
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float zMax;
    float zMin;
}stBoxMarker;

static stBoxMarker boxMarker;

typedef struct stObjectDetected
{
    string name;
    float x;
    float y;
    float z;
    float probability;
}stObjectDetected;

static stObjectDetected tmpObj;
static vector<stObjectDetected> arObj;
static float plane_height = 0.2;
static float box_track_x = 1.0;
static float box_track_y = 0.0;
static float box_track_z = 0.2;

void ProcCloudCB(const sensor_msgs::PointCloud2 &input)
{
    //MODE_IDLE 不处理数据
    if(nMode == MODE_IDLE)
        return;

    //to footprint
    sensor_msgs::PointCloud2 pc_footprint;
    bool res = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(5.0)); 
    if(res == false)
    {
        return;
    }
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    //source cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);
    int src_pnt_num = cloud_src.size();
    // ROS_INFO("cloud_src size = %d",src_pnt_num); 

    RemoveBoxes();

    if(nMode == MODE_BOX_TRACK)
    {
        // 截取z轴方向，平面高度到0.5米内的点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_ptr;
        cloud_source_ptr = cloud_src.makeShared(); 
        pcl::PassThrough<pcl::PointXYZRGB> pass;//设置滤波器对象
        pass.setInputCloud (cloud_source_ptr);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (plane_height , plane_height + 0.05);
        pass.filter (*cloud_source_ptr);
        pass.setInputCloud (cloud_source_ptr);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (box_track_x - 0.05 , box_track_x + 0.05);
        pass.filter (*cloud_source_ptr);
        pass.setInputCloud (cloud_source_ptr);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (box_track_y - 0.03 , box_track_y + 0.03);
        pass.filter (*cloud_source_ptr);
        segmented_boxes.publish(cloud_source_ptr);
        int clound_points_num = cloud_source_ptr->size();
        // ROS_INFO("[MODE_BOX_TRACK] cloud_filtered  size = %d",clound_points_num); 

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    #ifdef  KD_TREE_M
        // [方法一] 对截取后的点云进行欧式距离分割
        static pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        static std::vector<pcl::PointIndices> cluster_indices;	                        // 点云团索引
        cluster_indices.clear();
        static pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;       // 欧式聚类对象
        tree->setInputCloud (cloud_source_ptr);		               // 输入截取后的点云
        ec.setClusterTolerance (0.02);			                                                 // 设置近邻搜索的搜索半径为2cm
        ec.setMinClusterSize (100);			                                                        // 设置一个聚类需要的最少的点数目为200
        ec.setMaxClusterSize (10000);			                                                // 设置一个聚类需要的最大点数目为10000
        ec.setSearchMethod (tree);			                                                      // 设置点云的搜索机制
        ec.setInputCloud (cloud_source_ptr);                            // 输入截取后的点云
        ec.extract (cluster_indices);                                                               // 执行欧式聚类分割

        arObj.clear();
        int nBoxCnt = 0;
        int object_num = cluster_indices.size();                                       // 分割出的点云团个数
        ROS_WARN("[track_box] 分割出物品个数 = %d",object_num);
        for(int i = 0 ; i < object_num ; i ++)
        {
            int point_num =  cluster_indices[i].indices.size();                 // 点云团i中的点数
            float points_x_sum = 0;
            float points_y_sum = 0;
            float points_z_sum = 0;
            bool bFirstPoint = true;
            for(int j = 0 ; j < point_num ; j ++)
            {
                int point_index = cluster_indices[i].indices[j];
                points_x_sum += cloud_source_ptr->points[point_index].x;
                points_y_sum += cloud_source_ptr->points[point_index].y;
                points_z_sum += cloud_source_ptr->points[point_index].z;
                pcl::PointXYZRGB p = cloud_source_ptr->points[point_index];
                if(bFirstPoint == true)
                {
                    boxMarker.xMax = boxMarker.xMin = p.x;
                    boxMarker.yMax = boxMarker.yMin = p.y;
                    boxMarker.zMax = boxMarker.zMin = p.z;
                    bFirstPoint = false;
                }

                if(p.x < boxMarker.xMin) { boxMarker.xMin = p.x;}
                if(p.x > boxMarker.xMax) { boxMarker.xMax = p.x;}
                if(p.y < boxMarker.yMin) { boxMarker.yMin = p.y;}
                if(p.y > boxMarker.yMax) { boxMarker.yMax = p.y;}
                if(p.z < boxMarker.zMin) { boxMarker.zMin = p.z;}
                if(p.z > boxMarker.zMax) { boxMarker.zMax = p.z;}
            }
            float object_width = fabs(boxMarker.yMax-boxMarker.yMin);
            //ROS_WARN("fobject_width = %.2f",object_width);
            if(/*boxMarker.xMin > 0.5 && boxMarker.yMin > -0.5 && boxMarker.yMax < 0.5 &&*/ object_width < 0.1)
            {
                DrawBox(&marker_pub,i,boxMarker.xMin, boxMarker.xMax, boxMarker.yMin, boxMarker.yMax, boxMarker.zMin, boxMarker.zMax, 1, 0, 1);

                std::ostringstream stringStream;
                stringStream << "box_" << nBoxCnt;
                std::string obj_id = stringStream.str();
                float object_x = boxMarker.xMin;
                float object_y = (boxMarker.yMin+boxMarker.yMax)/2;
                float object_z = plane_height + 0.02;
                DrawText(obj_id,0.06, boxMarker.xMax,object_y,boxMarker.zMax + 0.05, 1,0,1);
                tmpObj.name = obj_id;
                tmpObj.x = object_x;
                tmpObj.y = object_y;
                tmpObj.z = object_z;
                tmpObj.probability = 1.0f;
                arObj.push_back(tmpObj);

                nBoxCnt++;
                // ROS_WARN("[box_%d] xMin= %.2f yMin = %.2f yMax = %.2f  z = %.2f pnt= %d",i,boxMarker.xMin, boxMarker.yMin, boxMarker.yMax,boxMarker.zMin,point_num);
            }
        }
        if(nBoxCnt > 0)
        {
            // 找出离锁定位置最近的，更新追踪坐标
            SortObjectsInTrack();
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    #else
        //  [方法二]  直接对分割后的点云进行坐标统计
        float points_x_sum = 0;
        float points_y_sum = 0;
        float points_z_sum = 0;
        bool bFirstPoint = true;
        for(int i=0 ; i<clound_points_num ; i++)
        {
            points_x_sum += cloud_source_ptr->points[i].x;
            points_y_sum += cloud_source_ptr->points[i].y;
            points_z_sum += cloud_source_ptr->points[i].z;
            pcl::PointXYZRGB p = cloud_source_ptr->points[i];
            if(bFirstPoint == true)
            {
                boxMarker.xMax = boxMarker.xMin = p.x;
                boxMarker.yMax = boxMarker.yMin = p.y;
                boxMarker.zMax = boxMarker.zMin = p.z;
                bFirstPoint = false;
            }

            if(p.x < boxMarker.xMin) { boxMarker.xMin = p.x;}
            if(p.x > boxMarker.xMax) { boxMarker.xMax = p.x;}
            if(p.y < boxMarker.yMin) { boxMarker.yMin = p.y;}
            if(p.y > boxMarker.yMax) { boxMarker.yMax = p.y;}
            if(p.z < boxMarker.zMin) { boxMarker.zMin = p.z;}
            if(p.z > boxMarker.zMax) { boxMarker.zMax = p.z;}
        }
        float object_width = fabs(boxMarker.yMax-boxMarker.yMin);
        //ROS_WARN("fobject_width = %.2f",object_width);
        if(/*boxMarker.xMin > 0.5 && boxMarker.yMin > -0.5 && boxMarker.yMax < 0.5 &&*/ object_width < 0.1)
        {
            DrawBox(&marker_pub,0,boxMarker.xMin, boxMarker.xMax, boxMarker.yMin, boxMarker.yMax, boxMarker.zMin, boxMarker.zMax, 1, 0, 1);
            std::string obj_id = "box";
            float object_x = boxMarker.xMin;
            float object_y = (boxMarker.yMin+boxMarker.yMax)/2;
            float object_z = plane_height + 0.02;
            DrawText(obj_id,0.06, boxMarker.xMax,object_y,boxMarker.zMax + 0.05, 1,0,1);
            tmpObj.name = obj_id;
            tmpObj.x = object_x;
            tmpObj.y = object_y;
            tmpObj.z = object_z;
            tmpObj.probability = 1.0f;
            arObj.clear();
            arObj.push_back(tmpObj);
            // ROS_WARN("[box] xMin= %.2f yMin = %.2f yMax = %.2f  z = %.2f pnt= %d",boxMarker.xMin, boxMarker.yMin, boxMarker.yMax,boxMarker.zMin,clound_points_num);
            // 发送结果消息
            wpb_mani_behaviors::Coord coord;
            coord.name.push_back(arObj[0].name);
            coord.x.push_back(arObj[0].x);
            coord.y.push_back(arObj[0].y);
            coord.z.push_back(arObj[0].z);
            coord.probability.push_back(arObj[0].probability);
            coord_pub.publish(coord);
            // 更新追踪坐标
            box_track_x = arObj[0].x;
            box_track_y = arObj[0].y;
            ROS_WARN("[track_box] 排序出最终坐标 ( %.2f , %.2f ) ",box_track_x,box_track_y);
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    #endif //  KD_TREE_M
    }
}

void SortObjectsInTrack()
{
    int nNum = arObj.size();
    if (nNum == 0)
        return;
    int nTrackIndex =0;
    if(nNum > 0)
    {
        // 找出离跟踪位置最近的
         float minObjDist =sqrt((arObj[0].x - box_track_x) * (arObj[0].x - box_track_x) + (arObj[0].y - box_track_y) * (arObj[0].y - box_track_y));
        for(int n = 0; n<nNum; n++)
        {
            float curDist =sqrt((arObj[n].x - box_track_x) * (arObj[n].x - box_track_x) + (arObj[n].y - box_track_y) * (arObj[n].y - box_track_y));
            if(curDist < minObjDist)
            {
                // 交换位置
                nTrackIndex = n;
                minObjDist = curDist;
            }
         }
    }
    // 搜索完毕，发送消息
    wpb_mani_behaviors::Coord coord;
    coord.name.push_back(arObj[nTrackIndex].name);
    coord.x.push_back(arObj[nTrackIndex].x);
    coord.y.push_back(arObj[nTrackIndex].y);
    coord.z.push_back(arObj[nTrackIndex].z);
    coord.probability.push_back(arObj[nTrackIndex].probability);
    coord_pub.publish(coord);
    // 更新追踪坐标
    box_track_x = arObj[nTrackIndex].x;
    box_track_y = arObj[nTrackIndex].y;
    ROS_WARN("[track_box] 索引出最终坐标 ( %.2f , %.2f )",box_track_x,box_track_y);
}

void DrawBox(ros::Publisher* inPub, int inID, float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB)
{
    line_box.header.frame_id = "base_footprint";
    line_box.ns = "line_box";
    line_box.action = visualization_msgs::Marker::ADD;
    line_box.id = inID;
    line_box.type = visualization_msgs::Marker::LINE_LIST;
    line_box.scale.x = 0.005;
    line_box.color.r = inR;
    line_box.color.g = inG;
    line_box.color.b = inB;
    line_box.color.a = 1.0;
    line_box.pose.orientation=tf::createQuaternionMsgFromYaw(0.0);

    geometry_msgs::Point p;
    p.z = inMinZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.z = inMaxZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);
    inPub->publish(line_box);
    line_box.points.clear();
}

static int nTextNum = 2;
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "base_footprint";
    text_marker.ns = "line_text";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = nTextNum;
    nTextNum ++;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = inScale;
    text_marker.color.r = inR;
    text_marker.color.g = inG;
    text_marker.color.b = inB;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = inX;
    text_marker.pose.position.y = inY;
    text_marker.pose.position.z = inZ;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(1.0);

    text_marker.text = inText;

    marker_pub.publish(text_marker);
}

void RemoveBoxes()
{
    line_box.action = 3;
    line_box.points.clear();
    marker_pub.publish(line_box);
    text_marker.action = 3;
    marker_pub.publish(text_marker);
}

void BoxTrackCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 目标物品的坐标
    box_track_x = msg->position.x;
    box_track_y = msg->position.y;
    box_track_z = msg->position.z;
    ROS_WARN("[track_box] 接收到锁定坐标 ( %.2f , %.2f , %.2f) " ,box_track_x, box_track_y, box_track_z);
    plane_height = box_track_z;
    nMode = MODE_BOX_TRACK;
}

void PlaneHeightCB(const std_msgs::Float64::ConstPtr &msg)
{
    plane_height = msg->data;
}

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
   
    nFindIndex = msg->data.find("box_track stop");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[track_box] 接收到指令 box_track stop");
        RemoveBoxes();
        nMode = MODE_IDLE;
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wpb_mani_track_box");
    ROS_INFO("wpb_mani_track_box start!");
    tf_listener = new tf::TransformListener(); 

    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("topic", pc_topic, "/points2");
   
    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe(pc_topic, 10 , ProcCloudCB);
    ros::Subscriber beh_sub = nh.subscribe("/wpb_mani/behaviors", 30, BehaviorCB);
    ros::Subscriber plane_height_sub = nh.subscribe("/wpb_mani/plane_height", 30, PlaneHeightCB);
    ros::Subscriber track_sub = nh.subscribe("/wpb_mani/box_track", 1, BoxTrackCallback);

    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("obj_pointcloud",1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("obj_marker", 10);
    coord_pub = nh.advertise<wpb_mani_behaviors::Coord>("/wpb_mani/boxes_3d", 10);

    segmented_boxes = nh.advertise<PointCloud> ("segmented_boxes",1);

    ros::spin();

    delete tf_listener; 

    return 0;

}