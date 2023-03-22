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
#include "PathHolder.h"

CPathHolder::CPathHolder()
{
}

CPathHolder::~CPathHolder()
{
}

void CPathHolder::SetPoint(float inX,float inY)
{
     if(arPoint.size() >0)
    {
        int nIndex = arPoint.size()-1;
        float dist = fabs(inX-arPoint[nIndex].x) + fabs(inY-arPoint[nIndex].y);
        if(dist < 0.05)
            return;
    }
    stPathPoint pntNew;
    pntNew.x = inX;
    pntNew.y = inY;
    arPoint.push_back(pntNew);
    ROS_WARN(" [CPathHolder::SetPoint] (%.2f , %.2f)", pntNew.x,pntNew.y);
}

void CPathHolder::AddNewPoint(stPathPoint inPnt)
{
    int nLastPntIndex = arPoint.size()-1;
    if(nLastPntIndex >= 0 )
    {
        // 离散出连续点
        stPathPoint pntLast = arPoint[nLastPntIndex];
        float fDist = sqrt((inPnt.x - pntLast.x) * (inPnt.x - pntLast.x) + (inPnt.y - pntLast.y) * (inPnt.y - pntLast.y));
        int nStep = fDist/0.05;
        stPathPoint pntMid;
        float fStep_x = (inPnt.x - pntLast.x)/nStep;
        float fStep_y = (inPnt.y - pntLast.y)/nStep;
        for(int i=1;i<=nStep;i++)
        {
            pntMid.x = pntLast.x + fStep_x*i;
            pntMid.y = pntLast.y + fStep_y*i;
            arPoint.push_back(pntMid);
        }
    }
    arPoint.push_back(inPnt);
}

std::string Flt2Str(float inVal)
{
    std::ostringstream stringStream;
    stringStream << inVal;
    std::string retStr = stringStream.str();
    return retStr;
}

std::string CPathHolder::GetPathDir()
{
    std::string strPathDir;
    char const* home = getenv("HOME");
    strPathDir = home;
    strPathDir += "/WPR_Path";
    return strPathDir;
}

bool CPathHolder::SavePathToFile(std::string inFilename)
{
    TiXmlDocument *docSave = new TiXmlDocument();
    TiXmlElement *RootElement = new TiXmlElement("WPR_Path");
    docSave->LinkEndChild(RootElement);

    int nNumPathPoints = arPoint.size();
    for(int i=0;i<nNumPathPoints;i++)
    {
        TiXmlElement *PathPointElement = new TiXmlElement("PathPoint");
        PathPointElement->InsertEndChild(TiXmlElement("x"))->InsertEndChild(TiXmlText(Flt2Str(arPoint[i].x)));
        PathPointElement->InsertEndChild(TiXmlElement("y"))->InsertEndChild(TiXmlText(Flt2Str(arPoint[i].y)));
       
        RootElement->InsertEndChild(*PathPointElement);  
    }

    bool res = docSave->SaveFile(inFilename);
    if(res == true)
        ROS_INFO("Saved path to file! filename = %s", inFilename.c_str());
    else
        ROS_INFO("Failed to save path... filename = %s", inFilename.c_str());

    return res;
}

bool CPathHolder::LoadPathFromFile(std::string inFilename)
{
    TiXmlDocument docLoad(inFilename);
    bool resLoad = docLoad.LoadFile();
    if(resLoad == false)
    {
        ROS_INFO("Failed to load path... filename = %s", inFilename.c_str());
        return false;
    }

    arPoint.clear();
    stPathPoint newPathPoint;
    TiXmlElement* RootElement = docLoad.RootElement();
    for(TiXmlNode* item = RootElement->FirstChild("PathPoint");item;item = item->NextSibling("PathPoint"))
    {
        TiXmlNode* child = item->FirstChild();
        const char* pos_x = child->ToElement()->GetText();
        newPathPoint.x = std::atof(pos_x);
        child = item->IterateChildren(child);
        const char* pos_y = child->ToElement()->GetText();
        newPathPoint.y = std::atof(pos_y);
        child = item->IterateChildren(child);
        AddNewPoint(newPathPoint);
        //arPoint.push_back(newPathPoint);
    }
    ROS_WARN(" load path from = %s  Num= %d", inFilename.c_str(),(int)arPoint.size());
    return true;
}

bool CPathHolder::LoadPath(std::string inFilename)
{
    std::string strPathDir = GetPathDir();
    std::string strFullName = strPathDir + "/" + inFilename;
    LoadPathFromFile(strFullName);
}
