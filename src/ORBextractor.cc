/**
* This file is part of ORB-SLAM2.
* This file is based on the file orb.cpp from the OpenCV library (see BSD license below).
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
/**
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include "ORBextractor.h"


using namespace cv;
using namespace std;

namespace ORB_SLAM2
{

const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD = 19;

/**
 * @brief 用于计算特征点的方向，这里是返回角度作为方向
 * @details 计算方向 
 * @note 全局静态函数
 * @param[in] image 需要进行操作的原图像
 * @param[in] pt 要计算特征点方向的特征点的坐标
 * @param[in] u_max 图像块每一行的坐标边界
 * @return float 角度
 */
static float IC_Angle(const Mat& image, 
                      Point2f pt,  
                      const vector<int> & u_max)
{
    // 图像的矩 前者是按照图像块的y坐标加权 后者是按照图像块的x坐标加权
    int m_01 = 0, m_10 = 0;

    // 这里是获取特征点所在的图像块的中心点指针，其实就是特征点像素指针
    const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
    // 这条中心线的计算需要特殊对待 
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
        // 注意这里的center下标可以是负的 中心水平线上的像素按x坐标加权 本行中x属于[-HALF_PATCH_SIZE, +HALF_PATCH_SIZE]
        m_10 += u * center[u];

    // Go line by line in the circuI853lar patch
    // 这里的step1表示这个二维图像中的行有几个字节
    int step = (int)image.step1();
    // 注意这里是以中心线维对称轴 然后对称的每成对的两行之间进行遍历 这样处理加快了计算速度
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
    {
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
        for (int u = -d; u <= d; ++u)
        {
            int val_plus = center[u + v*step], val_minus = center[u - v*step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }

    return fastAtan2((float)m_01, (float)m_10);
}


const float factorPI = (float)(CV_PI/180.f);
static void computeOrbDescriptor(const KeyPoint& kpt,
                                 const Mat& img, const Point* pattern,
                                 uchar* desc)
{
    float angle = (float)kpt.angle*factorPI;
    float a = (float)cos(angle), b = (float)sin(angle);

    const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
    const int step = (int)img.step;

    #define GET_VALUE(idx) \
        center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
               cvRound(pattern[idx].x*a - pattern[idx].y*b)]


    for (int i = 0; i < 32; ++i, pattern += 16)
    {
        int t0, t1, val;
        t0 = GET_VALUE(0); t1 = GET_VALUE(1);
        val = t0 < t1;
        t0 = GET_VALUE(2); t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;
        t0 = GET_VALUE(4); t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        t0 = GET_VALUE(6); t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        t0 = GET_VALUE(8); t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        t0 = GET_VALUE(10); t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        t0 = GET_VALUE(12); t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        t0 = GET_VALUE(14); t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;

        desc[i] = (uchar)val;
    }

    #undef GET_VALUE
}

static int bit_pattern_31_[256*4] =
{
    8,-3, 9,5/*mean (0), correlation (0)*/,
    4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
    -11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
    7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
    2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
    1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
    -2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
    -13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
    -13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
    10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
    -13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
    -11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
    7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
    -4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
    -13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
    -9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
    12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
    -3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
    -6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
    11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
    4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
    5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
    3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
    -8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
    -2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
    -13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
    -7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
    -4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
    -10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
    5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
    5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
    1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
    9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
    4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
    2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
    -4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
    -8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
    4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
    0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
    -13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
    -3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
    -6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
    8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
    0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
    7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
    -13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
    10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
    -6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
    10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
    -13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
    -13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
    3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
    5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
    -1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
    3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
    2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
    -13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
    -13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
    -13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
    -7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
    6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
    -9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
    -2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
    -12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
    3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
    -7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
    -3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
    2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
    -11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
    -1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
    5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
    -4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
    -9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
    -12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
    10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
    7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
    -7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
    -4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
    7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
    -7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
    -13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
    -3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
    7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
    -13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
    1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
    2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
    -4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
    -1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
    7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
    1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
    9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
    -1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
    -13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
    7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
    12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
    6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
    5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
    2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
    3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
    2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
    9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
    -8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
    -11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
    1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
    6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
    2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
    6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
    3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
    7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
    -11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
    -10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
    -5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
    -10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
    8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
    4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
    -10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
    4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
    -2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
    -5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
    7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
    -9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
    -5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
    8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
    -9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
    1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
    7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
    -2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
    11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
    -12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
    3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
    5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
    0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
    -9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
    0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
    -1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
    5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
    3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
    -13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
    -5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
    -4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
    6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
    -7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
    -13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
    1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
    4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
    -2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
    2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
    -2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
    4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
    -6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
    -3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
    7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
    4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
    -13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
    7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
    7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
    -7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
    -8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
    -13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
    2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
    10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
    -6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
    8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
    2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
    -11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
    -12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
    -11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
    5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
    -2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
    -1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
    -13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
    -10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
    -3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
    2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
    -9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
    -4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
    -4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
    -6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
    6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
    -13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
    11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
    7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
    -1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
    -4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
    -7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
    -13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
    -7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
    -8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
    -5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
    -13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
    1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
    1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
    9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
    5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
    -1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
    -9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
    -1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
    -13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
    8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
    2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
    7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
    -10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
    -10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
    4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
    3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
    -4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
    5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
    4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
    -9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
    0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
    -12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
    3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
    -10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
    8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
    -8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
    2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
    10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
    6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
    -7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
    -3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
    -1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
    -3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
    -8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
    4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
    2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
    6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
    3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
    11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
    -3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
    4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
    2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
    -10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
    -13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
    -13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
    6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
    0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
    -13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
    -9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
    -13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
    5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
    2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
    -1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
    9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
    11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
    3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
    -1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
    3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
    -13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
    5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
    8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
    7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
    -10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
    7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
    9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
    7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
    -1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
};

ORBextractor::ORBextractor(int _nfeatures,          // 指定要提取特征点的数目
                           float _scaleFactor,      // 指定图像金字塔的缩放系数
                           int _nlevels,            // 指定图像金字塔的层数
                           int _iniThFAST,          // 默认FAST阈值
                           int _minThFAST) :        // 最小FAST阈值
                                                nfeatures(_nfeatures),              
                                                scaleFactor(_scaleFactor), 
                                                nlevels(_nlevels),
                                                iniThFAST(_iniThFAST), 
                                                minThFAST(_minThFAST)
{
    
    // 存储每层图像缩放系数的vector调整为符合图层数目的大小
    mvScaleFactor.resize(nlevels);
    // Sigma^2实际就是每层图像相对于初始图像缩放因子的平方
    mvLevelSigma2.resize(nlevels);
    
    // 初始图像的参数都是1
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;

    // 逐层计算
    for(int i=1; i<nlevels; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }

    // 用来保存缩放因子的倒数
    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);

    // 计算倒数
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }

    // 根据设置的金字塔层数设置金字塔vector
    mvImagePyramid.resize(nlevels);

    // 同样设置每层需要提取的特征点数目向量
    mnFeaturesPerLevel.resize(nlevels);
    
    // 缩放系数的倒数
    float factor = 1.0f / scaleFactor;

    // TODO: 计算机视觉life课程中解释的不是很清楚，实际上使用factor而不是factor的平方会使得特征点数目分布的更加均匀，不会集中在前几层
    // 计算第0层的特征点数目
    float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));

    // 逐层计算所需要的特征点数目
    int sumFeatures = 0;
    for( int level = 0; level < nlevels-1; level++ )
    {
        // mnFeaturesPerLevel 保存每一层应该分配的特征点数目
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    // 考虑圆整的影响，最后一层直接用所需的全部特征点数目减去之前已经分配的特征点数目，防止分配的特征点数目不足
    mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);

    // HACK: 成员变量pattern的长度，也就是点的个数，这里的512表示512个点，数组中存储的是坐标，所以是256*2*2
    const int npoints = 512;

    // HACK: 获取用于计算BRIEF描述子的随机采样点点集头指针
    // pattern0的数据类型是Point*， 而bit_pattern_31_是int[]，所以要进行强制转换
    const Point* pattern0 = (const Point*)bit_pattern_31_;
    // INFO: std::copy(start, end, std::back_inserter(container)), start,end 是输入序列的两个迭代器，容器container支持push_back，实际上就是
    // 把pattern从头位置到512位复制到 成员变量pattern中 
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

    //This is for orientation
    // pre-compute the end of a row in a circular patch
    // 下面是跟特征点的旋转计算有关的
    // 预先计算圆形patch中行的结束位置 HALF_PATCH_SIZE=15
    umax.resize(HALF_PATCH_SIZE + 1);

    int v,      // 循环辅助变量 
        v0,     // 辅助变量
        vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);    // HACK: 计算圆的最大行号，+1是把中间行也给考虑进去了
        // 这里的最大行号指的是计算的时候的最大行号，此行和圆的交点在45°圆心角的一边上，这样选择是为了对称性
    int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
    // 半径的平方
    const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(sqrt(hp2 - v * v));

    // Make sure we are symmetric
    // 保持对称性
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }
}

/**
 * @brief 计算特征点的方向
 * @details 并非是类的成员函数
 * @param[in] image 特征点所在的图像
 * @param[in] keypoints 存储有特征点的vector
 * @param[in] umax 每个特征点所在图像区块的每行的边界
 */ 
static void computeOrientation(const Mat& image,                // 特征点所在的图像 
                               vector<KeyPoint>& keypoints,     // 存储有特征点的vector
                               const vector<int>& umax)         // 以及每个特征点所在图像区块的每行的边界u_max组成的vector
{
    // 遍历完所有的特征点
    for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
         keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
    {
        // 计算这个特征点的方向
        keypoint->angle = IC_Angle(image,           // 特征点所在图层的图像 
                                   keypoint->pt,    // 特征点在这张图像中的坐标
                                   umax);           // 图像区块的边界
    }
}

// 将提取器节点分成四个子节点，同时完成图像区域的划分，特征点归属的划分，同时更新相关标志位
void ExtractorNode::DivideNode(ExtractorNode &n1,           // 四个提取器节点 
                               ExtractorNode &n2, 
                               ExtractorNode &n3, 
                               ExtractorNode &n4)
{
    // 得到当前提取器节点图像的一半长宽
    const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
    const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

    //Define boundaries of childs
    // 将一个图像区域细分成四个小的图像区域
    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x+halfX,UL.y);
    n1.BL = cv::Point2i(UL.x,UL.y+halfY);
    n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
    // 存储在该节点对应的图像网格中提取出来的特征点
    n1.vKeys.reserve(vKeys.size());

    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x,UL.y+halfY);
    n2.vKeys.reserve(vKeys.size());

    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x,BL.y);
    n3.vKeys.reserve(vKeys.size());

    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    //Associate points to childs
    // 遍历父节点中vKey存储的特征点
    for(size_t i=0;i<vKeys.size();i++)
    {
        // 根据特征点的位置，将其划分到相应的子节点中
        const cv::KeyPoint &kp = vKeys[i];
        if(kp.pt.x<n1.UR.x)
        {
            if(kp.pt.y<n1.BR.y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        }
        else if(kp.pt.y<n1.BR.y)
            n2.vKeys.push_back(kp);
        else
            n4.vKeys.push_back(kp);
    }

    // 判断当前子节点中的特征点的数目，更新标志位，决定是否还要继续分裂
    if(n1.vKeys.size()==1)
        n1.bNoMore = true;
    if(n2.vKeys.size()==1)
        n2.bNoMore = true;
    if(n3.vKeys.size()==1)
        n3.bNoMore = true;
    if(n4.vKeys.size()==1)
        n4.bNoMore = true;

}

// 使用八叉树法(四叉树)对一个图层中的特征点进行平均和分发
vector<cv::KeyPoint> ORBextractor::DistributeOctTree(const vector<cv::KeyPoint>& vToDistributeKeys,     // 等待分配的特征点  
                                                     const int &minX,                                   // 当前图层的边界
                                                     const int &maxX, 
                                                     const int &minY, 
                                                     const int &maxY, 
                                                     const int &N,                                      // 希望提取出的特征点个数
                                                     const int &level)                                  // 指定的图层
{
    // Compute how many initial nodes   
    // 生成的初始节点数目，由图像的大小决定，一般是一个或两个
    const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));

    // 一个初始节点的x方向有多少个像素
    const float hX = static_cast<float>(maxX-minX)/nIni;

    // 存储提取器节点的列表
    list<ExtractorNode> lNodes;

    // 存储初始提取器节点指针的vector
    vector<ExtractorNode*> vpIniNodes;
    // 重新设置其大小
    vpIniNodes.resize(nIni);

    // 生成指定个数的初始提取器节点
    for(int i=0; i<nIni; i++)
    {
        // 生成一个提取器节点
        ExtractorNode ni;
        // 设置提取器节点的图像边界
        // 这里的节点的边界先都按照 边缘扩充图像 下的坐标系来理解
        ni.UL = cv::Point2i(hX*static_cast<float>(i),0);
        ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);
        ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
        ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
        // 重新设置vKeys的大小 vKeys为保存有当前节点的特征点的向量
        ni.vKeys.reserve(vToDistributeKeys.size());
        // 将生成的提取器节点添加到列表中
        // 虽然这里的ni是局部变量，但是由于这里的std::vector::push_back()是拷贝参数的内容到一个新的对象中，然后再添加到列表中，因此这里的内存不会成为野指针
        lNodes.push_back(ni);
        // 存储这个初始的提取器节点句柄
        vpIniNodes[i] = &lNodes.back();
    }

    //Associate points to childs
    // 将特征点分配到子提取器节点中
    for(size_t i=0;i<vToDistributeKeys.size();i++)
    {
        // 获取这个特征点对象
        const cv::KeyPoint &kp = vToDistributeKeys[i];
        // 按特征点的横轴位置，分配给属于那个图像区域的提取器节点 最初的提取器节点
        vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
    }

    // 当遍历此提取器节点列表，标记那些不可再分裂的节点，删除那些没有被分配到特征点的节点
    list<ExtractorNode>::iterator lit = lNodes.begin();

    while(lit!=lNodes.end())
    {
        // 如果初始的提取器节点所分配到的特征点个数为1
        if(lit->vKeys.size()==1)
        {
            // 那么就标志位置位，表示此节点不可再分
            lit->bNoMore=true;
            // 更新迭代器
            lit++;
        }
        // 如果一个提取器节点没有被分配到特征点，那么就从列表中直接删除它
        else if(lit->vKeys.empty())
            lit = lNodes.erase(lit);
            // 由于是直接删除它，因此迭代器没有必要更新，不然会导致元素的跳过
        else
            lit++;
    }

    // 结束标志位清空
    bool bFinish = false;

    // HACK: 迭代次数，用于累计第一组分裂时的迭代次数，但是在后文中并没有得到实际的使用
    int iteration = 0;

    // 声明一个vector 用来储存节点的vSize和句柄对
    // 这个变量记录了在一次分裂中，那些可以继续进行分裂的节点中包含的特征点数目和其句柄
    vector<pair<int,ExtractorNode*> > vSizeAndPointerToNode;
    // 调整其大小，一个初始节点分裂成四个，当然实际不会那么多
    vSizeAndPointerToNode.reserve(lNodes.size()*4);

    // 利用四叉树的方法对图像进行划分区域
    while(!bFinish)
    {
        // 更新迭代次数计数器
        iteration++;

        // 保存当前节点个数
        int prevSize = lNodes.size();

        // 重新定位迭代器指向列表头部
        lit = lNodes.begin();

        // 需要展开的节点计数，这个一直保持累计，不清零
        int nToExpand = 0;

        // 
        vSizeAndPointerToNode.clear();

        // 开始遍历列表内的所有提取器节点，并进行分解或保留
        while(lit!=lNodes.end())
        {
            // 如果提取器节点中只有一个特征点
            if(lit->bNoMore)
            {
                // If node only contains one point do not subdivide and continue
                // 没有必要进行细分，直接跳过
                lit++;
                continue;
            }
            else
            {
                // If more than one point, subdivide
                // 提取器节点中有超过一个节点，继续进行细分
                ExtractorNode n1,n2,n3,n4;
                lit->DivideNode(n1,n2,n3,n4);

                // Add childs if they contain points
                // 如果子节点图像区域中包含特征点，那么就将该节点加入列表中
                if(n1.vKeys.size()>0)
                {
                    // 添加到列表前面
                    lNodes.push_front(n1);                    
                    if(n1.vKeys.size()>1)
                    {
                        // 如果节点内的特征点数大于1，那么待展开的节点数加一，意味着该节点还要继续进行分解
                        nToExpand++;
                        // 保存这个节点的特征点数目和节点指针
                        vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));

                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n2.vKeys.size()>0)
                {
                    lNodes.push_front(n2);
                    if(n2.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n3.vKeys.size()>0)
                {
                    lNodes.push_front(n3);
                    if(n3.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n4.vKeys.size()>0)
                {
                    lNodes.push_front(n4);
                    if(n4.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                // 父节点分解完毕后，就将其删除
                lit=lNodes.erase(lit);
                continue;
            }
        }       

        // Finish if there are more nodes than required features
        // or all nodes contain just one point
        // 停止这个过程的条件有两个
        // 1.当前的节点数目已经超过了需要的特征点数
        // 2.当前所有节点中，只包含一个特征点
        if((int)lNodes.size()>=N || 
                                    (int)lNodes.size()==prevSize)   // prevSize是分裂之前的节点数，如果分裂之前的节点数和分裂之后的节点数相等，那么每个节点
                                                                    // 都只包含一个特征点，都没有进行进一步的分解
        {
            bFinish = true;
        }
        // 如果下一次划分后的节点数大于要求的数目时
        // 就慢慢划分直到使其刚刚达到或者超过要求的特征点的数目
        // 这里原本应该是nToExpand x 4， nToExpand表示的是可以展开的子节点个数
        // 即一个list中的sub-node分裂成4个subsub-Node，数目上看是增加了3个，因为sub-node会被删除
        else if(((int)lNodes.size()+nToExpand*3)>N)
        {

            while(!bFinish)
            {
                // 获取当前的list中的节点个数
                prevSize = lNodes.size();

                // 保留那些还可以分裂的节点信息
                vector<pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                // 清空
                vSizeAndPointerToNode.clear();

                // 对需要划分的部分进行排序，即对兴趣点数较多的区域进行划分
                // 默认是从小到大排序 让特征点稀疏的区域尽可能保留更多的特征点
                sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end());
                // 遍历这个存储了pair对的vector，注意不是遍历整个list了
                for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
                {
                    ExtractorNode n1,n2,n3,n4;
                    // 对每个需要进行分裂的节点进行分裂
                    vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                    if((int)lNodes.size()>=N)
                        break;
                }

                if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                    bFinish = true;

            }
        }
    }

    // Retain the best point in each node
    // 保留每个区域响应值最大的一个兴趣点
    // 存储感兴趣的特征点的过滤结果
    vector<cv::KeyPoint> vResultKeys;
    // 调整大小为要提取的特征点数目
    vResultKeys.reserve(nfeatures);
    // 遍历这个节点列表
    for(list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
    {
        // 得到这个节点区域中的特征点容器句柄
        vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
        // 得到指向第一个特征点的指针
        cv::KeyPoint* pKP = &vNodeKeys[0];
        // 初始化最大响应值
        float maxResponse = pKP->response;

        for(size_t k=1;k<vNodeKeys.size();k++)
        {
            if(vNodeKeys[k].response>maxResponse)
            {
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }
        // 将这个节点区域中响应值最大的特征点加入最终结果容器
        vResultKeys.push_back(*pKP);
    }
    // 返回最终结果容器
    return vResultKeys;
}

// 计算八叉树的特征点，函数名字后面的OctTree只是说明了在过滤和分配特征点时所用的方式
// 所有的特征点，这里的第一层vector存储的是某图层里面的所有特征点，第二层存储的是整个图像金字塔中的所有图层里面的所有特征点
void ORBextractor::ComputeKeyPointsOctTree(vector<vector<KeyPoint> >& allKeypoints, const vector<cv::Rect2i> &vPersonArea)
{
    // 重新调整图像层数
    allKeypoints.resize(nlevels);

    // 图像cell的尺寸，是个正方形，可以理解为边长in像素坐标
    const float W = 30;

    // 对每一层图像做处理
    // 遍历所有图像
    for (int level = 0; level < nlevels; ++level)
    {
        // 计算这层图像的坐标边界， 注意这里是坐标边界， EDGE_THRESHOLD指的是应该可以提取特征点的有效图像边界，后面会一直使用“有效图像边界”这个自创名词
        const int minBorderX = EDGE_THRESHOLD-3;                                    // 这里的3是因为在计算FAST特征点的时候，需要建立一个半径为3的圆
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD+3;
        const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD+3;

        // 存储需要进行平均分配的特征点
        vector<cv::KeyPoint> vToDistributeKeys;
        // 一般都是过量采集，所以这里预分配的空间大小是nfeatures*10
        vToDistributeKeys.reserve(nfeatures*10);

        // 计算进行特征点提取的图像区域尺寸
        const float width = (maxBorderX-minBorderX);
        const float height = (maxBorderY-minBorderY);

        // 计算网格在当前层的图像有的行数和列数
        const int nCols = width/W;
        const int nRows = height/W;
        // 计算每个图像网格所占的像素行数和列数
        const int wCell = ceil(width/nCols);
        const int hCell = ceil(height/nRows);

        // 开始遍历图像网格，还是以行遍历的
        for(int i=0; i<nRows; i++)
        {
            // 计算当前网格初始行坐标
            const float iniY =minBorderY+i*hCell;
            // 计算当前行的最大行坐标
            // TODO:+6是为了什么，minBorderY不已经是扩充3后的边界了吗
            float maxY = iniY+hCell+6;
            // 如果初始行坐标就已经超过了有效的图像边界了，这里的有效图像是指原始的，可以提取FAST特征点的图像区域
            if(iniY>=maxBorderY-3)
                continue;
            // 图像大小不能划分出整齐的图像网格，那么就要委屈最后一行了
            if(maxY>maxBorderY)
                maxY = maxBorderY;
            // 开始列的遍历
            for(int j=0; j<nCols; j++)
            {
                // 初始列坐标
                const float iniX =minBorderX+j*wCell;
                // 最大列坐标
                float maxX = iniX+wCell+6;
                if(iniX>=maxBorderX-6)
                    continue;
                if(maxX>maxBorderX)
                    maxX = maxBorderX;



                // FAST提取兴趣点，自适应阈值
                // 这个向量存储这个ceil中的特征点
                vector<cv::KeyPoint> vKeysCell;
                // 调用OpenCV的库函数
                FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),     // 待检测的图像，这里就是当前遍历到的图像块
                     vKeysCell,                                                         // 存储角点位置的容器
                     iniThFAST,                                                         // 检测阈值
                     true);                                                             // 使能非极大值抑制
                // 如果这个图像块中使用默认的FAST检测阈值没有能够检测到角点
                if(vKeysCell.empty())
                {
                    // 使用更低的阈值来检测
                    FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                         vKeysCell,minThFAST,true);
                }
                // 当图像cell中检测到FAST角点的时候执行下面的语句
                if(!vKeysCell.empty())
                {
                    // 遍历cell中的全部角点
                    for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
                    {
                        // 目前这些角点坐标都是基于cell的，我们要将这个坐标恢复回去
                        (*vit).pt.x+=j*wCell;
                        (*vit).pt.y+=i*hCell;
                        vToDistributeKeys.push_back(*vit);
                    }
                }

                /*********************** self define *******************************/
                // cout << "Before select in level " << level << ", the amount of kp = " << vKeysCell.size() << endl;
                // temp recover the keypoint
                float scale = mvScaleFactor[level];
                for (auto vit = vToDistributeKeys.begin(); vit != vToDistributeKeys.end(); vit++)
                {
                    vit->pt.x += minBorderX;
                    vit->pt.y += minBorderY;
                    vit->pt *= scale;
                }


                bool Find = false;
                for (auto vit_kp = vToDistributeKeys.begin(); vit_kp != vToDistributeKeys.end();)
                {
                    for (auto vit_area = vPersonArea.begin(); vit_area != vPersonArea.end(); vit_area++)
                    {
                        Find = false;

                        if (vit_area->contains(vit_kp->pt))
                        {
                            Find = true;
                            // cout << "Find " << endl;
                            vit_kp = vToDistributeKeys.erase(vit_kp);
                            break;
                        }
                    }
                    if (!Find)
                    {
                        ++vit_kp;
                        Find = false;
                    }
                }



                float scale_inverse = 1 / scale;
                for (auto vit = vToDistributeKeys.begin(); vit != vToDistributeKeys.end(); vit++)
                {
                    vit->pt *= scale_inverse;
                    vit->pt.x -= minBorderX;
                    vit->pt.y -= minBorderY;
                }
                // cout << "After select in level " << level << ", the amount of kp = " << vKeysCell.size() << endl;
                /*********************** self define *******************************/

            }
        }

        // 声明一个对当前特征点的容器的引用
        vector<KeyPoint> & keypoints = allKeypoints[level];
        // 调整其大小为欲提取出来的特征点个数，这里也是扩大的
        keypoints.reserve(nfeatures);

        // 根据mnFeaturesPerLevel，即该层的兴趣点数，对特征点进行剔除
        // 返回值是一个保存有特征点的vector容器，含有剔除后的保留下来的特征点
        // 得到的特征点的坐标，依旧是在当前图层下来讲的
        keypoints = DistributeOctTree(vToDistributeKeys,                // 当前图层提取出来的特征点，也即是等待剔除的特征点     
                                      minBorderX,                       // 
                                      maxBorderX,
                                      minBorderY, 
                                      maxBorderY,
                                      mnFeaturesPerLevel[level],        // 希望保留下来的当前特征点的个数
                                      level);                           // 当前图像所在的层数

        
        // patch_size是对于底层的初始图像来说的，现在要根据当前图层的尺度缩放倍数进行缩放得到缩放后的PATCH大小和特征点的方向计算有关
        const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

        // Add border to coordinates and scale information
        // 获取剔除过程后保留下来的特征点数目
        const int nkps = keypoints.size();
        // 遍历这些特征点
        for(int i=0; i<nkps ; i++)
        {
            // 对于每一个保留下来的特征点，恢复到相对于当前图层“边缘扩充图像下”的坐标系的坐标
            keypoints[i].pt.x+=minBorderX;
            keypoints[i].pt.y+=minBorderY;
            keypoints[i].octave=level;
            keypoints[i].size = scaledPatchSize;
        }
    }

    // compute orientations
    // 计算这些特征点的方向信息，注意这里还是分层计算的
    for (int level = 0; level < nlevels; ++level)
        computeOrientation(mvImagePyramid[level],       // 对应的图层的图像 
                           allKeypoints[level],         // 这个图层中提取并保留下来的特征点容器
                           umax);                       // PATCH的横坐标边界
}

void ORBextractor::ComputeKeyPointsOld(std::vector<std::vector<KeyPoint> > &allKeypoints)
{
    allKeypoints.resize(nlevels);

    float imageRatio = (float)mvImagePyramid[0].cols/mvImagePyramid[0].rows;

    for (int level = 0; level < nlevels; ++level)
    {
        const int nDesiredFeatures = mnFeaturesPerLevel[level];

        const int levelCols = sqrt((float)nDesiredFeatures/(5*imageRatio));
        const int levelRows = imageRatio*levelCols;

        const int minBorderX = EDGE_THRESHOLD;
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD;
        const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD;

        const int W = maxBorderX - minBorderX;
        const int H = maxBorderY - minBorderY;
        const int cellW = ceil((float)W/levelCols);
        const int cellH = ceil((float)H/levelRows);

        const int nCells = levelRows*levelCols;
        const int nfeaturesCell = ceil((float)nDesiredFeatures/nCells);

        vector<vector<vector<KeyPoint> > > cellKeyPoints(levelRows, vector<vector<KeyPoint> >(levelCols));

        vector<vector<int> > nToRetain(levelRows,vector<int>(levelCols,0));
        vector<vector<int> > nTotal(levelRows,vector<int>(levelCols,0));
        vector<vector<bool> > bNoMore(levelRows,vector<bool>(levelCols,false));
        vector<int> iniXCol(levelCols);
        vector<int> iniYRow(levelRows);
        int nNoMore = 0;
        int nToDistribute = 0;


        float hY = cellH + 6;

        for(int i=0; i<levelRows; i++)
        {
            const float iniY = minBorderY + i*cellH - 3;
            iniYRow[i] = iniY;

            if(i == levelRows-1)
            {
                hY = maxBorderY+3-iniY;
                if(hY<=0)
                    continue;
            }

            float hX = cellW + 6;

            for(int j=0; j<levelCols; j++)
            {
                float iniX;

                if(i==0)
                {
                    iniX = minBorderX + j*cellW - 3;
                    iniXCol[j] = iniX;
                }
                else
                {
                    iniX = iniXCol[j];
                }


                if(j == levelCols-1)
                {
                    hX = maxBorderX+3-iniX;
                    if(hX<=0)
                        continue;
                }


                Mat cellImage = mvImagePyramid[level].rowRange(iniY,iniY+hY).colRange(iniX,iniX+hX);

                cellKeyPoints[i][j].reserve(nfeaturesCell*5);

                FAST(cellImage,cellKeyPoints[i][j],iniThFAST,true);

                if(cellKeyPoints[i][j].size()<=3)
                {
                    cellKeyPoints[i][j].clear();

                    FAST(cellImage,cellKeyPoints[i][j],minThFAST,true);
                }


                const int nKeys = cellKeyPoints[i][j].size();
                nTotal[i][j] = nKeys;

                if(nKeys>nfeaturesCell)
                {
                    nToRetain[i][j] = nfeaturesCell;
                    bNoMore[i][j] = false;
                }
                else
                {
                    nToRetain[i][j] = nKeys;
                    nToDistribute += nfeaturesCell-nKeys;
                    bNoMore[i][j] = true;
                    nNoMore++;
                }

            }
        }


        // Retain by score

        while(nToDistribute>0 && nNoMore<nCells)
        {
            int nNewFeaturesCell = nfeaturesCell + ceil((float)nToDistribute/(nCells-nNoMore));
            nToDistribute = 0;

            for(int i=0; i<levelRows; i++)
            {
                for(int j=0; j<levelCols; j++)
                {
                    if(!bNoMore[i][j])
                    {
                        if(nTotal[i][j]>nNewFeaturesCell)
                        {
                            nToRetain[i][j] = nNewFeaturesCell;
                            bNoMore[i][j] = false;
                        }
                        else
                        {
                            nToRetain[i][j] = nTotal[i][j];
                            nToDistribute += nNewFeaturesCell-nTotal[i][j];
                            bNoMore[i][j] = true;
                            nNoMore++;
                        }
                    }
                }
            }
        }

        vector<KeyPoint> & keypoints = allKeypoints[level];
        keypoints.reserve(nDesiredFeatures*2);

        const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

        // Retain by score and transform coordinates
        for(int i=0; i<levelRows; i++)
        {
            for(int j=0; j<levelCols; j++)
            {
                vector<KeyPoint> &keysCell = cellKeyPoints[i][j];
                KeyPointsFilter::retainBest(keysCell,nToRetain[i][j]);
                if((int)keysCell.size()>nToRetain[i][j])
                    keysCell.resize(nToRetain[i][j]);


                for(size_t k=0, kend=keysCell.size(); k<kend; k++)
                {
                    keysCell[k].pt.x+=iniXCol[j];
                    keysCell[k].pt.y+=iniYRow[i];
                    keysCell[k].octave=level;
                    keysCell[k].size = scaledPatchSize;
                    keypoints.push_back(keysCell[k]);
                }
            }
        }

        if((int)keypoints.size()>nDesiredFeatures)
        {
            KeyPointsFilter::retainBest(keypoints,nDesiredFeatures);
            keypoints.resize(nDesiredFeatures);
        }
    }

    // and compute orientations
    for (int level = 0; level < nlevels; ++level)
        computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
}

static void computeDescriptors(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors,
                               const vector<Point>& pattern)
{
    descriptors = Mat::zeros((int)keypoints.size(), 32, CV_8UC1);

    for (size_t i = 0; i < keypoints.size(); i++)
        computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
}

void ORBextractor::operator()( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints,
                      OutputArray _descriptors, const vector<cv::Rect2i> &vPersonArea)
{ 
    // 如果图像为空，跳过
    if(_image.empty())
        return;
    // 获取图像并判断格式正确与否
    Mat image = _image.getMat();
    assert(image.type() == CV_8UC1 );

    // Pre-compute the scale pyramid
    // 构建图像金字塔
    ComputePyramid(image);

    // 特征点的提取与分配
    // 存储特征点的向量
    vector < vector<KeyPoint> > allKeypoints;
    // 使用八叉树的方式计算每层图像的特征点并进行分配
    ComputeKeyPointsOctTree(allKeypoints, vPersonArea);
    //ComputeKeyPointsOld(allKeypoints);


    // 描述子计算
    // 保存描述子用的变量
    Mat descriptors;

    // 统计整个图像金字塔中的特征点
    int nkeypoints = 0;
    for (int level = 0; level < nlevels; ++level)
        nkeypoints += (int)allKeypoints[level].size();
    
    // 如果本图像金字塔中没有任何的特征点
    if( nkeypoints == 0 )
        // 强制释放矩阵
        _descriptors.release();
    else
    {
        // 如果图像金字塔中有特征点 那么就创建这个存储描述子的矩阵 这个矩阵是存储整个图像金字塔中特征点的描述子的
        _descriptors.create(nkeypoints,         // 矩阵的行数 对应为特征点的总个数 
                            32,                 // 矩阵的列数 对应为使用32*8=256位描述子
                            CV_8U);             // 矩阵元素的格式
        descriptors = _descriptors.getMat();
    }
    
    // 清空用作返回特征点提取结果的vector容器
    _keypoints.clear();
    // 预分配正确的大小
    _keypoints.reserve(nkeypoints);

    // 保存寻址时的偏移量
    int offset = 0;
    // 开始遍历每一层的图像
    for (int level = 0; level < nlevels; ++level)
    {
        // 获取在allKeypoints中当前层特征点容器的句柄
        vector<KeyPoint>& keypoints = allKeypoints[level];
        // 本层的特征点数
        int nkeypointsLevel = (int)keypoints.size();

        // 如果设定是本层提取特征点0个 就直接跳过
        if(nkeypointsLevel==0)
            continue;

        // preprocess the resized image
        // 首先得到对当前图层的拷贝
        Mat workingMat = mvImagePyramid[level].clone();
        // 进行高斯模糊 提取特征点时 使用的是清晰的原图像 然后在计算描述子时 为了避免图像噪声的影响 使用了高斯模糊
        GaussianBlur(workingMat,                // 原图像 
                     workingMat,                // 输出图像
                     Size(7, 7),                // 高斯滤波器模版大小
                     2,                         // 高斯滤波器在横向的滤波系数
                     2,                         // 高斯滤波器在纵向的滤波系数
                     BORDER_REFLECT_101);       // 边缘扩展点的插值类型 线性插值

        // Compute the descriptors
        // desc存储当前图层的描述子
        Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
        computeDescriptors(workingMat, keypoints, desc, pattern);

        offset += nkeypointsLevel;

        // Scale keypoint coordinates
        if (level != 0)
        {
            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
            for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                 keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
                keypoint->pt *= scale;
        }
        // And add the keypoints to the output
        _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
    }
}


// 构建图像金字塔
// image 输入图像,这个输入的图像所有像素都是有效的，也就是说可以在其上提取出FAST角点的
void ORBextractor::ComputePyramid(cv::Mat image)
{
    // 开始遍历所有图层
    for (int level = 0; level < nlevels; ++level)
    {
        // 获取本层图像的缩放系数
        float scale = mvInvScaleFactor[level];
        // 计算本层图像的像素尺寸大小
        Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
        // HACK： 真正的包括无效图像区域的大小，实际上作者这样做是将图像进行裁边，EDGE_THRESHOLD区域内的图像不进行FAST角点检测
        Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
        // 声明两个临时变量，第一个保持和有效图像大小相同，另外一个看上去是要作为掩膜
        Mat temp(wholeSize, image.type()), masktemp;
        // 给图像金字塔该图层预分配Mat
        mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        // Compute the resized image
        // 计算非第0层图像 resize
        if( level != 0 )
        {
            // 对于非底层图像就要进行缩放操作了
            resize(mvImagePyramid[level-1],             // 输入图像 
                   mvImagePyramid[level],               // 输出图像
                   sz,                                  // 输出图像的尺寸
                   0,                                   // 水平方向上的缩放系数，0表示自动计算
                   0,                                   // 垂直方向上的缩放系数，0表示自动计算
                   INTER_LINEAR);                       // 图像缩放的差值算法类型，这里是线性差值算法
            
            // 滤波所用的工具函数，增加边界
            // TODO: 貌似这样做是因为在计算描述子前，进行高斯滤波的时候，图像边界会导致一些问题
            copyMakeBorder(mvImagePyramid[level],                       // 原图像
                           temp,                                        // 目标图像(此时已经有大了一圈的尺寸了)
                           EDGE_THRESHOLD,                              // top
                           EDGE_THRESHOLD,                              // bottom
                           EDGE_THRESHOLD,                              // left
                           EDGE_THRESHOLD,                              // right
                           BORDER_REFLECT_101+BORDER_ISOLATED);         // 扩充方式
        }
        else
        {
            // 对于底层图像，直接就扩充边界了
            copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101);            
        }
    }

}

} //namespace ORB_SLAM
