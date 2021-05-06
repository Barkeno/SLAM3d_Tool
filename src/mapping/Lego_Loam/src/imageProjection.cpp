// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.


#include "imageProjection.h"

namespace lego_loam
{

ImageProjection::ImageProjection(){

    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;

    allocateMemory();
    resetParameters();
}

void ImageProjection::allocateMemory(){

    laserCloudIn.reset(new pcl::PointCloud<PointType>());
    laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

    fullCloud.reset(new pcl::PointCloud<PointType>());
    fullInfoCloud.reset(new pcl::PointCloud<PointType>());

    groundCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
    outlierCloud.reset(new pcl::PointCloud<PointType>());

    fullCloud->points.resize(N_SCAN*Horizon_SCAN);
    fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
    neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
    neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
    neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

    allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
    allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

    queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
    queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];

    /////////////////////////////
    startRingIndex.assign(N_SCAN, 0);
    endRingIndex.assign(N_SCAN, 0);

    segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
    segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
    segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);
}

void ImageProjection::resetParameters(){
    laserCloudIn->clear();
    groundCloud->clear();
    segmentedCloud->clear();
    segmentedCloudPure->clear();
    outlierCloud->clear();

    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
    intensityMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8UC1, cv::Scalar::all(0));
    labelCount = 1;

    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
}

ImageProjection::~ImageProjection(){}

void ImageProjection::copyPointCloud(const pcl::PointCloud<PointType> &laserCloud){

    std::vector<int> indices;
    pcl::copyPointCloud(laserCloud,*laserCloudIn);
    pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
}

void ImageProjection::loadPointCloud(const pcl::PointCloud<PointType> &cloud_in)
{
    // 1. Convert ros message to pcl point cloud
    copyPointCloud(cloud_in);

    // 2. Start and end angle of a scan
    findStartEndAngle();

    //3. Range image projection
    projectPointCloud();

    //4. Mark ground points
    groundRemoval();

    //5. Point Cloud segmentation
    cloudSegmentation();

    //6. Publish all clouds

    //7. Reset paramters for next iteration
//    resetParameters();

}



void ImageProjection::findStartEndAngle(){
    // start and end orientation of this cloud
    int cloudSize = laserCloudIn->points.size();
    if(cloudSize<0)
        return;

    startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
    endOrientation = -atan2(laserCloudIn->points[cloudSize - 1].y, laserCloudIn->points[cloudSize - 1].x) +2 * M_PI;

    // avoid exception ..
    if (endOrientation - startOrientation > 3 * M_PI)
        endOrientation -= 2 * M_PI;
    else if (endOrientation - startOrientation < M_PI)
        endOrientation += 2 * M_PI;
    orientationDiff = endOrientation - startOrientation;
}

void ImageProjection::projectPointCloud(){
    // range image projection
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize; 
    PointType thisPoint;

    cloudSize = laserCloudIn->points.size();
//    cout << "SIZE: " << cloudSize << endl;

    for (size_t i = 0; i < cloudSize; ++i){

        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        // find the row and column index in the iamge for this point
        if (useCloudRing == true){
            rowIdn = laserCloudInRing->points[i].ring;
        }
        else{
            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        }
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;

        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        if (range < sensorMinimumRange)
            continue;
        
        rangeMat.at<float>(rowIdn, columnIdn) = range;
        intensityMat.at<int>(rowIdn, columnIdn) = laserCloudIn->points[i].intensity;

        thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

        index = columnIdn  + rowIdn * Horizon_SCAN;
        fullCloud->points[index] = thisPoint;
        fullInfoCloud->points[index] = thisPoint;
        fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
    }
}


void ImageProjection::groundRemoval(){
    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;
    // groundMat
    // -1, no valid info to check if ground of not
    //  0, initial value, after validation, means not ground
    //  1, ground
    for (size_t j = 0; j < Horizon_SCAN; ++j){
        for (size_t i = 0; i < groundScanInd; ++i){

            lowerInd = j + ( i )*Horizon_SCAN;
            upperInd = j + (i+1)*Horizon_SCAN;

            if (fullCloud->points[lowerInd].intensity == -1 ||
                fullCloud->points[upperInd].intensity == -1){
                // no info to check, invalid points
                groundMat.at<int8_t>(i,j) = -1;
                continue;
            }
                
            diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
            diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
            diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

            angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

            if (abs(angle - sensorMountAngle) <= 10){
                groundMat.at<int8_t>(i,j) = 1;
                groundMat.at<int8_t>(i+1,j) = 1;
            }
        }
    }
    // extract ground cloud (groundMat == 1)
    // mark entry that doesn't need to label (ground and invalid point) for segmentation
    // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
    for (size_t i = 0; i < N_SCAN; ++i){
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                labelMat.at<int>(i,j) = -1;
            }
        }
    }
//    if (pubGroundCloud.getNumSubscribers() != 0){
//        for (size_t i = 0; i <= groundScanInd; ++i){
//            for (size_t j = 0; j < Horizon_SCAN; ++j){
//                if (groundMat.at<int8_t>(i,j) == 1)
//                    groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
//            }
//        }
//    }
}

void ImageProjection::cloudSegmentation(){
    // segmentation process
    for (size_t i = 0; i < N_SCAN; ++i)
        for (size_t j = 0; j < Horizon_SCAN; ++j)
            if (labelMat.at<int>(i,j) == 0)
                labelComponents(i, j);

    int sizeOfSegCloud = 0;
    // extract segmented cloud for lidar odometry
    for (size_t i = 0; i < N_SCAN; ++i) {

        startRingIndex[i] = sizeOfSegCloud-1 + 5;

        for (size_t j = 0; j < Horizon_SCAN; ++j) {
            if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                // outliers that will not be used for optimization (always continue)
                if (labelMat.at<int>(i,j) == 999999){
                    if (i > groundScanInd && j % 5 == 0){
                        outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        continue;
                    }else{
                        continue;
                    }
                }
                // majority of ground points are skipped
                if (groundMat.at<int8_t>(i,j) == 1){
                    if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                        continue;
                }
                // mark ground points so they will not be considered as edge features later
                segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                // mark the points' column index for marking occlusion later
                segmentedCloudColInd[sizeOfSegCloud] = j;
                // save range info
                segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                // save seg cloud
                segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                // size of seg cloud
                ++sizeOfSegCloud;
            }
        }

        endRingIndex[i] = sizeOfSegCloud-1 - 5;
    }

    // extract segmented cloud for visualization
//    if (pubSegmentedCloudPure.getNumSubscribers() != 0){
//        for (size_t i = 0; i < N_SCAN; ++i){
//            for (size_t j = 0; j < Horizon_SCAN; ++j){
//                if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
//                    segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
//                    segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
//                }
//            }
//        }
//    }
}



void ImageProjection::labelComponents(int row, int col){
    // use std::queue std::vector std::deque will slow the program down greatly
    float d1, d2, alpha, angle;
    int fromIndX, fromIndY, thisIndX, thisIndY; 
    // bool lineCountFlag[N_SCAN] = {false};
    // bool *lineCountFlag = new bool[N_SCAN];
    bool lineCountFlag[N_SCAN];

    queueIndX[0] = row;
    queueIndY[0] = col;
    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    int allPushedIndSize = 1;
    
    while(queueSize > 0){
        // Pop point
        fromIndX = queueIndX[queueStartInd];
        fromIndY = queueIndY[queueStartInd];
        --queueSize;
        ++queueStartInd;
        // Mark popped point
        labelMat.at<int>(fromIndX, fromIndY) = labelCount;
        // Loop through all the neighboring grids of popped grid
        for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
            // new index
            thisIndX = fromIndX + (*iter).first;
            thisIndY = fromIndY + (*iter).second;
            // index should be within the boundary
            if (thisIndX < 0 || thisIndX >= N_SCAN)
                continue;
            // at range image margin (left or right side)
            if (thisIndY < 0)
                thisIndY = Horizon_SCAN - 1;
            if (thisIndY >= Horizon_SCAN)
                thisIndY = 0;
            // prevent infinite loop (caused by put already examined point back)
            if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                continue;

            d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                            rangeMat.at<float>(thisIndX, thisIndY));
            d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                            rangeMat.at<float>(thisIndX, thisIndY));

            if ((*iter).first == 0)
                alpha = segmentAlphaX;
            else
                alpha = segmentAlphaY;

            angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

            if (angle > segmentTheta){

                queueIndX[queueEndInd] = thisIndX;
                queueIndY[queueEndInd] = thisIndY;
                ++queueSize;
                ++queueEndInd;

                labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                lineCountFlag[thisIndX] = true;

                allPushedIndX[allPushedIndSize] = thisIndX;
                allPushedIndY[allPushedIndSize] = thisIndY;
                ++allPushedIndSize;
            }
        }
    }

    // check if this segment is valid
    bool feasibleSegment = false;
    if (allPushedIndSize >= 30)
        feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidPointNum){
        int lineCount = 0;
        for (size_t i = 0; i < N_SCAN; ++i)
            if (lineCountFlag[i] == true)
                ++lineCount;
        if (lineCount >= segmentValidLineNum)
            feasibleSegment = true;            
    }
    // segment is valid, mark these points
    if (feasibleSegment == true){
        ++labelCount;
    }else{ // segment is invalid, mark these points
        for (size_t i = 0; i < allPushedIndSize; ++i){
            labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
        }
    }
}

}
