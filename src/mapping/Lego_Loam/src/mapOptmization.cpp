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
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include "mapOptmization.h"

namespace lego_loam
{

using namespace gtsam;


mapOptimization::mapOptimization()
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    float filter_size;
    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    filter_size = 0.5; downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    filter_size = 0.3; downSizeFilterSurf.setLeafSize(filter_size, filter_size, filter_size); // default 0.4;
    downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

    filter_size = 0.3; downSizeFilterHistoryKeyFrames.setLeafSize(filter_size, filter_size, filter_size); // default 0.4; for histor key frames of loop closure
    filter_size = 1.0; downSizeFilterSurroundingKeyPoses.setLeafSize(filter_size, filter_size, filter_size); // default 1; for surrounding key poses of scan-to-map optimization

    downSizeFilterGlobalMapKeyPoses.setLeafSize(1.0, 1.0, 1.0); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(0.4, 0.4, 0.4); // for global map visualization

    allocateMemory();
}

void mapOptimization::reset()
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    if(isam)
        delete isam;
    isam = new ISAM2(parameters);

    float filter_size;
    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    filter_size = 0.5; downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    filter_size = 0.3; downSizeFilterSurf.setLeafSize(filter_size, filter_size, filter_size); // default 0.4;
    downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

    filter_size = 0.3; downSizeFilterHistoryKeyFrames.setLeafSize(filter_size, filter_size, filter_size); // default 0.4; for histor key frames of loop closure
    filter_size = 1.0; downSizeFilterSurroundingKeyPoses.setLeafSize(filter_size, filter_size, filter_size); // default 1; for surrounding key poses of scan-to-map optimization

    downSizeFilterGlobalMapKeyPoses.setLeafSize(1.0, 1.0, 1.0); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(0.4, 0.4, 0.4); // for global map visualization

    allocateMemory();
}

void mapOptimization::allocateMemory(){

    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    surroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
    surroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>());        

    laserCloudRaw.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
    laserCloudRawDS.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
    laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization
    laserCloudOutlierLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
    laserCloudOutlierLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner feature set from odoOptimization
    laserCloudSurfTotalLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
    laserCloudSurfTotalLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());

    laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

    
    nearHistoryCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    nearHistoryCornerKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());
    SCnearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    SCnearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

    latestCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    SClatestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    latestSurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

    RSlatestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>()); // giseop
    RSnearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    RSnearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

    kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>());
    globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
    globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
    globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
    globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());

    timeLaserOdometry = 0;

    fullMapCloud.reset(new pcl::PointCloud<PointType>());

    timeLaserCloudCornerLast = 0;
    timeLaserCloudSurfLast = 0;
    timeLaserOdometry = 0;
    timeLaserCloudOutlierLast = 0;
    timeLastGloalMapPublish = 0;

    timeLastProcessing = -1;

    newLaserCloudCornerLast = false;
    newLaserCloudSurfLast = false;

    newLaserOdometry = false;
    newLaserCloudOutlierLast = false;

    for (int i = 0; i < 6; ++i){
        transformLast[i] = 0;
        transformSum[i] = 0;
        transformIncre[i] = 0;
        transformTobeMapped[i] = 0;
        transformBefMapped[i] = 0;
        transformAftMapped[i] = 0;
    }

    imuPointerFront = 0;
    imuPointerLast = -1;

    for (int i = 0; i < imuQueLength; ++i){
        imuTime[i] = 0;
        imuRoll[i] = 0;
        imuPitch[i] = 0;
    }

    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
    priorNoise = noiseModel::Diagonal::Variances(Vector6);
    odometryNoise = noiseModel::Diagonal::Variances(Vector6);

    matA0 = cv::Mat (5, 3, CV_32F, cv::Scalar::all(0));
    matB0 = cv::Mat (5, 1, CV_32F, cv::Scalar::all(-1));
    matX0 = cv::Mat (3, 1, CV_32F, cv::Scalar::all(0));

    matA1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
    matD1 = cv::Mat (1, 3, CV_32F, cv::Scalar::all(0));
    matV1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));

    isDegenerate = false;
    matP = cv::Mat (6, 6, CV_32F, cv::Scalar::all(0));

    laserCloudCornerFromMapDSNum = 0;
    laserCloudSurfFromMapDSNum = 0;
    laserCloudCornerLastDSNum = 0;
    laserCloudSurfLastDSNum = 0;
    laserCloudOutlierLastDSNum = 0;
    laserCloudSurfTotalLastDSNum = 0;

    potentialLoopFlag = false;
    aLoopIsClosed = false;

    latestFrameID = 0;
}

void mapOptimization::transformAssociateToMap()
{
    float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
    float y1 = transformBefMapped[4] - transformSum[4];
    float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

    float x2 = x1;
    float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
    float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

    transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
    transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
    transformIncre[5] = z2;

    float sbcx = sin(transformSum[0]);
    float cbcx = cos(transformSum[0]);
    float sbcy = sin(transformSum[1]);
    float cbcy = cos(transformSum[1]);
    float sbcz = sin(transformSum[2]);
    float cbcz = cos(transformSum[2]);

    float sblx = sin(transformBefMapped[0]);
    float cblx = cos(transformBefMapped[0]);
    float sbly = sin(transformBefMapped[1]);
    float cbly = cos(transformBefMapped[1]);
    float sblz = sin(transformBefMapped[2]);
    float cblz = cos(transformBefMapped[2]);

    float salx = sin(transformAftMapped[0]);
    float calx = cos(transformAftMapped[0]);
    float saly = sin(transformAftMapped[1]);
    float caly = cos(transformAftMapped[1]);
    float salz = sin(transformAftMapped[2]);
    float calz = cos(transformAftMapped[2]);

    float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
                - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
    transformTobeMapped[0] = -asin(srx);

    float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                    - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                    - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                    + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                    + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                    + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
    float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                    - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                    + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                    + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                    - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                    + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
    transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]), 
                                    crycrx / cos(transformTobeMapped[0]));
    
    float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                    - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                    - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                    - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                    + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
    float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                    - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                    - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                    - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                    + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
    transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]), 
                                    crzcrx / cos(transformTobeMapped[0]));

    x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
    y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
    z1 = transformIncre[5];

    x2 = x1;
    y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    transformTobeMapped[3] = transformAftMapped[3] 
                            - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
    transformTobeMapped[4] = transformAftMapped[4] - y2;
    transformTobeMapped[5] = transformAftMapped[5] 
                            - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void mapOptimization::transformUpdate()
{
    if (imuPointerLast >= 0) {
        float imuRollLast = 0, imuPitchLast = 0;
        while (imuPointerFront != imuPointerLast) {
            if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
                break;
            }
            imuPointerFront = (imuPointerFront + 1) % imuQueLength;
        }

        if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
            imuRollLast = imuRoll[imuPointerFront];
            imuPitchLast = imuPitch[imuPointerFront];
        } else {
            int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
            float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack]) 
                                / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod) 
                            / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

            imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
            imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
        }

        transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
        transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
        }

    for (int i = 0; i < 6; i++) {
        transformBefMapped[i] = transformSum[i];
        transformAftMapped[i] = transformTobeMapped[i];
    }
}

void mapOptimization::updatePointAssociateToMapSinCos(){
    cRoll = cos(transformTobeMapped[0]);
    sRoll = sin(transformTobeMapped[0]);

    cPitch = cos(transformTobeMapped[1]);
    sPitch = sin(transformTobeMapped[1]);

    cYaw = cos(transformTobeMapped[2]);
    sYaw = sin(transformTobeMapped[2]);

    tX = transformTobeMapped[3];
    tY = transformTobeMapped[4];
    tZ = transformTobeMapped[5];
}

void mapOptimization::pointAssociateToMap(PointType const * const pi, PointType * const po)
{
    float x1 = cYaw * pi->x - sYaw * pi->y;
    float y1 = sYaw * pi->x + cYaw * pi->y;
    float z1 = pi->z;

    float x2 = x1;
    float y2 = cRoll * y1 - sRoll * z1;
    float z2 = sRoll * y1 + cRoll * z1;

    po->x = cPitch * x2 + sPitch * z2 + tX;
    po->y = y2 + tY;
    po->z = -sPitch * x2 + cPitch * z2 + tZ;
    po->intensity = pi->intensity;
}

void mapOptimization::updateTransformPointCloudSinCos(PointTypePose *tIn){

    ctRoll = cos(tIn->roll);
    stRoll = sin(tIn->roll);

    ctPitch = cos(tIn->pitch);
    stPitch = sin(tIn->pitch);

    ctYaw = cos(tIn->yaw);
    stYaw = sin(tIn->yaw);

    tInX = tIn->x;
    tInY = tIn->y;
    tInZ = tIn->z;
}

pcl::PointCloud<PointType>::Ptr mapOptimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn){
// !!! DO NOT use pcl for point cloud transformation, results are not accurate
    // Reason: unkown
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;
    PointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);

    for (int i = 0; i < cloudSize; ++i){

        pointFrom = &cloudIn->points[i];
        float x1 = ctYaw * pointFrom->x - stYaw * pointFrom->y;
        float y1 = stYaw * pointFrom->x + ctYaw* pointFrom->y;
        float z1 = pointFrom->z;

        float x2 = x1;
        float y2 = ctRoll * y1 - stRoll * z1;
        float z2 = stRoll * y1 + ctRoll* z1;

        pointTo.x = ctPitch * x2 + stPitch * z2 + tInX;
        pointTo.y = y2 + tInY;
        pointTo.z = -stPitch * x2 + ctPitch * z2 + tInZ;
        pointTo.intensity = pointFrom->intensity;

        cloudOut->points[i] = pointTo;
    }
    return cloudOut;
}

pcl::PointCloud<PointType>::Ptr mapOptimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn){

    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;
    PointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);
    
    for (int i = 0; i < cloudSize; ++i){

        pointFrom = &cloudIn->points[i];
        float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
        float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw)* pointFrom->y;
        float z1 = pointFrom->z;

        float x2 = x1;
        float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
        float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll)* z1;

        pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
        pointTo.y = y2 + transformIn->y;
        pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
        pointTo.intensity = pointFrom->intensity;

        cloudOut->points[i] = pointTo;
    }
    return cloudOut;
}

void mapOptimization::mapBuild(FeatureAssociation &FA, const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &mapCornerCloud, pcl::PointCloud<PointType> &mapSurfCloud, float* PoseAftMapped)
{

    initMapData(FA, cloud_in);

    run();

    saveMapCloud(mapCornerCloud, mapSurfCloud);

    savePoseTraj(PoseAftMapped);

}

void mapOptimization::savePoseTraj(float* PoseAftMapped)
{
    PoseAftMapped[0] = transformAftMapped[0];
    PoseAftMapped[1] = transformAftMapped[1];
    PoseAftMapped[2] = transformAftMapped[2];
    PoseAftMapped[3] = transformAftMapped[3];
    PoseAftMapped[4] = transformAftMapped[4];
    PoseAftMapped[5] = transformAftMapped[5];
}

void mapOptimization::initMapData(FeatureAssociation &FA, const pcl::PointCloud<PointType> &cloud_in)
{
    //1. laserCloudOutlierLastHandler
    laserCloudOutlierLast->clear();
    pcl::copyPointCloud(*(FA.outlierCloud), *laserCloudOutlierLast);
    newLaserCloudOutlierLast = true;

    //2. laserCloudRawHandler
    laserCloudRaw->clear();
    pcl::copyPointCloud(cloud_in,*laserCloudRaw);

    //3. laserCloudCornerLastHandler
    laserCloudCornerLast->clear();
    pcl::copyPointCloud(*(FA.laserCloudCornerLast), *laserCloudCornerLast);
    newLaserCloudCornerLast = true;

    //4. laserCloudSurfLastHandler
    laserCloudSurfLast->clear();
    pcl::copyPointCloud(*(FA.laserCloudSurfLast), *laserCloudSurfLast);
    newLaserCloudSurfLast = true;

    //5. laserOdometryHandler
    transformSum[0] = FA.transformSum[0];
    transformSum[1] = FA.transformSum[1];
    transformSum[2] = FA.transformSum[2];
    transformSum[3] = FA.transformSum[3];
    transformSum[4] = FA.transformSum[4];
    transformSum[5] = FA.transformSum[5];
    newLaserOdometry = true;
}

void mapOptimization::saveMap()
{
    if(fullMapCloud->size() != 0)
        fullMapCloud->clear();

    for(int i = 0; i < cornerCloudKeyFrames.size(); i++)
    {
        *fullMapCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
    }

    for(int i = 0; i < surfCloudKeyFrames.size(); i++)
    {
        *fullMapCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
    }
}

void mapOptimization::saveMapCloud(pcl::PointCloud<PointType> &mapCornerCloud, pcl::PointCloud<PointType> &mapSurfCloud)
{

    for(int i = 0; i < cornerCloudKeyFrames.size(); i++)
    {
        mapCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
    }

    for(int i = 0; i < surfCloudKeyFrames.size(); i++)
    {
        mapCornerCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
    }

}



void mapOptimization::imuHandler(){
    double roll, pitch, yaw;
//    tf::Quaternion orientation;
//    tf::quaternionMsgToTF(imuIn->orientation, orientation);
//    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    imuPointerLast = (imuPointerLast + 1) % imuQueLength;
//    imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
}


PointTypePose mapOptimization::trans2PointTypePose(float transformIn[]){
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll  = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw   = transformIn[2];
    return thisPose6D;
}


void mapOptimization::visualizeGlobalMapThread(){

    // save final point cloud
    pcl::io::savePCDFileASCII(fileDirectory+"finalCloud.pcd", *globalMapKeyFramesDS);

    string cornerMapString = "/tmp/cornerMap.pcd";
    string surfaceMapString = "/tmp/surfaceMap.pcd";
    string trajectoryString = "/tmp/trajectory.pcd";

    pcl::PointCloud<PointType>::Ptr cornerMapCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cornerMapCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceMapCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceMapCloudDS(new pcl::PointCloud<PointType>());
    
    for(int i = 0; i < cornerCloudKeyFrames.size(); i++) {
        *cornerMapCloud  += *transformPointCloud(cornerCloudKeyFrames[i],   &cloudKeyPoses6D->points[i]);
        *surfaceMapCloud += *transformPointCloud(surfCloudKeyFrames[i],     &cloudKeyPoses6D->points[i]);
        *surfaceMapCloud += *transformPointCloud(outlierCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
    }

    downSizeFilterCorner.setInputCloud(cornerMapCloud);
    downSizeFilterCorner.filter(*cornerMapCloudDS);
    downSizeFilterSurf.setInputCloud(surfaceMapCloud);
    downSizeFilterSurf.filter(*surfaceMapCloudDS);

    pcl::io::savePCDFileASCII(fileDirectory+"cornerMap.pcd", *cornerMapCloudDS);
    pcl::io::savePCDFileASCII(fileDirectory+"surfaceMap.pcd", *surfaceMapCloudDS);
    pcl::io::savePCDFileASCII(fileDirectory+"trajectory.pcd", *cloudKeyPoses3D);
}

void mapOptimization::startLoopClosure()
{
    m_bLoopThread = true;
    loop = loopthread();
}

void mapOptimization::endLoopClosure()
{
    m_bLoopThread = false;
    loop.join();
}

void mapOptimization::loopClosureThread(){

    if (loopClosureEnableFlag == false)
        return;


    while (m_bLoopThread){
        sleep(1);
        performLoopClosure();
    }
} // loopClosureThread


bool mapOptimization::detectLoopClosure(){

    std::lock_guard<std::mutex> lock(mtx);

    /* 
        * 1. xyz distance-based radius search (contained in the original LeGO LOAM code)
        * - for fine-stichting trajectories (for not-recognized nodes within scan context search) 
        */
    RSlatestSurfKeyFrameCloud->clear();
    RSnearHistorySurfKeyFrameCloud->clear();
    RSnearHistorySurfKeyFrameCloudDS->clear();

    // find the closest history key frame
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(currentRobotPosPoint, historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    
    RSclosestHistoryFrameID = -1;
    int curMinID = 1000000;
    // policy: take Oldest one (to fix error of the whole trajectory)
    for (int i = 0; i < pointSearchIndLoop.size(); ++i){
        int id = pointSearchIndLoop[i];
        {
            // RSclosestHistoryFrameID = id;
            // break;
            if( id < curMinID ) {
                curMinID = id;
                RSclosestHistoryFrameID = curMinID;
            }
        }
    }

    if (RSclosestHistoryFrameID == -1){
        // Do nothing here
        // then, do the next check: Scan context-based search 
        // not return false here;
    }
    else {
        // save latest key frames
        latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
        *RSlatestSurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
        *RSlatestSurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure],   &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
        pcl::PointCloud<PointType>::Ptr RShahaCloud(new pcl::PointCloud<PointType>());
        int cloudSize = RSlatestSurfKeyFrameCloud->points.size();
        for (int i = 0; i < cloudSize; ++i){
            if ((int)RSlatestSurfKeyFrameCloud->points[i].intensity >= 0){
                RShahaCloud->push_back(RSlatestSurfKeyFrameCloud->points[i]);
            }
        }
        RSlatestSurfKeyFrameCloud->clear();
        *RSlatestSurfKeyFrameCloud = *RShahaCloud;

        // save history near key frames
        for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j){
            if (RSclosestHistoryFrameID + j < 0 || RSclosestHistoryFrameID + j > latestFrameIDLoopCloure)
                continue;
            *RSnearHistorySurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[RSclosestHistoryFrameID+j], &cloudKeyPoses6D->points[RSclosestHistoryFrameID+j]);
            *RSnearHistorySurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[RSclosestHistoryFrameID+j],   &cloudKeyPoses6D->points[RSclosestHistoryFrameID+j]);
        }
        downSizeFilterHistoryKeyFrames.setInputCloud(RSnearHistorySurfKeyFrameCloud);
        downSizeFilterHistoryKeyFrames.filter(*RSnearHistorySurfKeyFrameCloudDS);
    }

    /* 
        * 2. Scan context-based global localization 
        */
    SClatestSurfKeyFrameCloud->clear();
    SCnearHistorySurfKeyFrameCloud->clear();
    SCnearHistorySurfKeyFrameCloudDS->clear();

    // std::lock_guard<std::mutex> lock(mtx);        
    latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
    SCclosestHistoryFrameID = -1; // init with -1
    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
    SCclosestHistoryFrameID = detectResult.first;
    yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)

    // if all close, reject
    if (SCclosestHistoryFrameID == -1){ 
        return false;
    }

        // save latest key frames: query ptcloud (corner points + surface points)
        // NOTE: using "closestHistoryFrameID" to make same root of submap points to get a direct relative between the query point cloud (latestSurfKeyFrameCloud) and the map (nearHistorySurfKeyFrameCloud). by giseop
        // i.e., set the query point cloud within mapside's local coordinate
        *SClatestSurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[SCclosestHistoryFrameID]);         
        *SClatestSurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure],   &cloudKeyPoses6D->points[SCclosestHistoryFrameID]); 

    pcl::PointCloud<PointType>::Ptr SChahaCloud(new pcl::PointCloud<PointType>());
    int cloudSize = SClatestSurfKeyFrameCloud->points.size();
    for (int i = 0; i < cloudSize; ++i){
        if ((int)SClatestSurfKeyFrameCloud->points[i].intensity >= 0){
            SChahaCloud->push_back(SClatestSurfKeyFrameCloud->points[i]);
        }
    }
    SClatestSurfKeyFrameCloud->clear();
    *SClatestSurfKeyFrameCloud = *SChahaCloud;

    // save history near key frames: map ptcloud (icp to query ptcloud)
    for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j){
        if (SCclosestHistoryFrameID + j < 0 || SCclosestHistoryFrameID + j > latestFrameIDLoopCloure)
            continue;
        *SCnearHistorySurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[SCclosestHistoryFrameID+j], &cloudKeyPoses6D->points[SCclosestHistoryFrameID+j]);
        *SCnearHistorySurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[SCclosestHistoryFrameID+j],   &cloudKeyPoses6D->points[SCclosestHistoryFrameID+j]);
    }
    downSizeFilterHistoryKeyFrames.setInputCloud(SCnearHistorySurfKeyFrameCloud);
    downSizeFilterHistoryKeyFrames.filter(*SCnearHistorySurfKeyFrameCloudDS);

    // // optional: publish history near key frames
    // if (pubHistoryKeyFrames.getNumSubscribers() != 0){
    //     sensor_msgs::PointCloud2 cloudMsgTemp;
    //     pcl::toROSMsg(*nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
    //     cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    //     cloudMsgTemp.header.frame_id = "/camera_init";
    //     pubHistoryKeyFrames.publish(cloudMsgTemp);
    // }

    return true;
} // detectLoopClosure



void mapOptimization::performLoopClosure( void ) {

    if (cloudKeyPoses3D->points.empty() == true)
        return;

    // try to find close key frame if there are any
    if (potentialLoopFlag == false){
        if (detectLoopClosure() == true){
            std::cout << std::endl;
            potentialLoopFlag = true; // find some key frames that is old enough or close enough for loop closure
            timeSaveFirstCurrentScanForLoopClosure = timeLaserOdometry;
        }
        if (potentialLoopFlag == false){
            return;
        }
    }

    // reset the flag first no matter icp successes or not
    potentialLoopFlag = false;

    // *****
    // Main 
    // *****
    // make common variables at forward
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionCameraFrame;
    float noiseScore = 0.5; // constant is ok...
    gtsam::Vector Vector6(6);
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    constraintNoise = noiseModel::Diagonal::Variances(Vector6);
    robustNoiseModel = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure
        gtsam::noiseModel::Diagonal::Variances(Vector6)
    ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

    bool isValidRSloopFactor = false;
    bool isValidSCloopFactor = false;

        /*
         * 1. RS loop factor (radius search)
         */
        if( RSclosestHistoryFrameID != -1 ) {
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaxCorrespondenceDistance(100);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(0);

            // Align clouds
            icp.setInputSource(RSlatestSurfKeyFrameCloud);
            icp.setInputTarget(RSnearHistorySurfKeyFrameCloudDS);
            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
            icp.align(*unused_result);

            std::cout << "[RS] ICP fit score: " << icp.getFitnessScore() << std::endl;
            if ( icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore ) {
                std::cout << "[RS] Reject this loop (bad icp fit score, > " << historyKeyframeFitnessScore << ")" << std::endl;
                isValidRSloopFactor = false;
            }
            else {
                std::cout << "[RS] The detected loop factor is added between Current [ " << latestFrameIDLoopCloure << " ] and RS nearest [ " << RSclosestHistoryFrameID << " ]" << std::endl;
                isValidRSloopFactor = true;
            }

            if( isValidRSloopFactor == true ) {
                correctionCameraFrame = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
                pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);
                Eigen::Affine3f correctionLidarFrame = pcl::getTransformation(z, x, y, yaw, roll, pitch);
                // transform from world origin to wrong pose
                Eigen::Affine3f tWrong = pclPointToAffine3fCameraToLidar(cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
                // transform from world origin to corrected pose
                Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
                pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
                gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[RSclosestHistoryFrameID]);
                gtsam::Vector Vector6(6);

                std::lock_guard<std::mutex> lock(mtx);
                gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, RSclosestHistoryFrameID, poseFrom.between(poseTo), robustNoiseModel));
                isam->update(gtSAMgraph);
                isam->update();
                gtSAMgraph.resize(0);
            }
        }

        /*
         * 2. SC loop factor (scan context)
         */
        if( SCclosestHistoryFrameID != -1 ) {
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaxCorrespondenceDistance(100);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(0);

            // Align clouds
            // Eigen::Affine3f icpInitialMatFoo = pcl::getTransformation(0, 0, 0, yawDiffRad, 0, 0); // because within cam coord: (z, x, y, yaw, roll, pitch)
            // Eigen::Matrix4f icpInitialMat = icpInitialMatFoo.matrix();
            icp.setInputSource(SClatestSurfKeyFrameCloud);
            icp.setInputTarget(SCnearHistorySurfKeyFrameCloudDS);
            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
            icp.align(*unused_result); 
            // icp.align(*unused_result, icpInitialMat); // PCL icp non-eye initial is bad ... don't use (LeGO LOAM author also said pcl transform is weird.)

            std::cout << "[SC] ICP fit score: " << icp.getFitnessScore() << std::endl;
            if ( icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore ) {
                std::cout << "[SC] Reject this loop (bad icp fit score, > " << historyKeyframeFitnessScore << ")" << std::endl;
                isValidSCloopFactor = false;
            }
            else {
                std::cout << "[SC] The detected loop factor is added between Current [ " << latestFrameIDLoopCloure << " ] and SC nearest [ " << SCclosestHistoryFrameID << " ]" << std::endl;
                isValidSCloopFactor = true;
            }

            if( isValidSCloopFactor == true ) {
                correctionCameraFrame = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
                pcl::getTranslationAndEulerAngles (correctionCameraFrame, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
                gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
                
                std::lock_guard<std::mutex> lock(mtx);
                // gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, closestHistoryFrameID, poseFrom.between(poseTo), constraintNoise)); // original 
                gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, SCclosestHistoryFrameID, poseFrom.between(poseTo), robustNoiseModel)); // giseop
                isam->update(gtSAMgraph);
                isam->update();
                gtSAMgraph.resize(0);
            }
        }

        // just for visualization
        // // publish corrected cloud
        // if (pubIcpKeyFrames.getNumSubscribers() != 0){
        //     pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        //     pcl::transformPointCloud (*latestSurfKeyFrameCloud, *closed_cloud, icp.getFinalTransformation());
        //     sensor_msgs::PointCloud2 cloudMsgTemp;
        //     pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
        //     cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        //     cloudMsgTemp.header.frame_id = "/camera_init";
        //     pubIcpKeyFrames.publish(cloudMsgTemp);
        // }   

    // flagging
    aLoopIsClosed = true;

} // performLoopClosure


Pose3 mapOptimization::pclPointTogtsamPose3(PointTypePose thisPoint){ // camera frame to lidar frame
    return Pose3(Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll), double(thisPoint.pitch)),
                        Point3(double(thisPoint.z),   double(thisPoint.x),    double(thisPoint.y)));
}

Eigen::Affine3f mapOptimization::pclPointToAffine3fCameraToLidar(PointTypePose thisPoint){ // camera frame to lidar frame
    return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y, thisPoint.yaw, thisPoint.roll, thisPoint.pitch);
}

void mapOptimization::extractSurroundingKeyFrames(){

    if (cloudKeyPoses3D->points.empty() == true)
        return;	
    
    if (loopClosureEnableFlag == true){
        // only use recent key poses for graph building
            if (recentCornerCloudKeyFrames.size() < surroundingKeyframeSearchNum){ // queue is not full (the beginning of mapping or a loop is just closed)
                // clear recent key frames queue
                recentCornerCloudKeyFrames. clear();
                recentSurfCloudKeyFrames.   clear();
                recentOutlierCloudKeyFrames.clear();
                int numPoses = cloudKeyPoses3D->points.size();
                for (int i = numPoses-1; i >= 0; --i){
                    int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
                    PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
                    updateTransformPointCloudSinCos(&thisTransformation);
                    // extract surrounding map
                    recentCornerCloudKeyFrames. push_front(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
                    recentSurfCloudKeyFrames.   push_front(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
                    recentOutlierCloudKeyFrames.push_front(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
                    if (recentCornerCloudKeyFrames.size() >= surroundingKeyframeSearchNum)
                        break;
                }
            }else{  // queue is full, pop the oldest key frame and push the latest key frame
                if (latestFrameID != cloudKeyPoses3D->points.size() - 1){  // if the robot is not moving, no need to update recent frames

                    recentCornerCloudKeyFrames. pop_front();
                    recentSurfCloudKeyFrames.   pop_front();
                    recentOutlierCloudKeyFrames.pop_front();
                    // push latest scan to the end of queue
                    latestFrameID = cloudKeyPoses3D->points.size() - 1;
                    PointTypePose thisTransformation = cloudKeyPoses6D->points[latestFrameID];
                    updateTransformPointCloudSinCos(&thisTransformation);
                    recentCornerCloudKeyFrames. push_back(transformPointCloud(cornerCloudKeyFrames[latestFrameID]));
                    recentSurfCloudKeyFrames.   push_back(transformPointCloud(surfCloudKeyFrames[latestFrameID]));
                    recentOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[latestFrameID]));
                }
            }

            for (int i = 0; i < recentCornerCloudKeyFrames.size(); ++i){
                *laserCloudCornerFromMap += *recentCornerCloudKeyFrames[i];
                *laserCloudSurfFromMap   += *recentSurfCloudKeyFrames[i];
                *laserCloudSurfFromMap   += *recentOutlierCloudKeyFrames[i];
            }
    }else{
        surroundingKeyPoses->clear();
        surroundingKeyPosesDS->clear();
        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeSurroundingKeyPoses->radiusSearch(currentRobotPosPoint, (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis, 0);
        for (int i = 0; i < pointSearchInd.size(); ++i)
            surroundingKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchInd[i]]);
        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        // delete key frames that are not in surrounding region
        int numSurroundingPosesDS = surroundingKeyPosesDS->points.size();
        for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i){
            bool existingFlag = false;
            for (int j = 0; j < numSurroundingPosesDS; ++j){
                if (surroundingExistingKeyPosesID[i] == (int)surroundingKeyPosesDS->points[j].intensity){
                    existingFlag = true;
                    break;
                }
            }
            if (existingFlag == false){
                surroundingExistingKeyPosesID.   erase(surroundingExistingKeyPosesID.   begin() + i);
                surroundingCornerCloudKeyFrames. erase(surroundingCornerCloudKeyFrames. begin() + i);
                surroundingSurfCloudKeyFrames.   erase(surroundingSurfCloudKeyFrames.   begin() + i);
                surroundingOutlierCloudKeyFrames.erase(surroundingOutlierCloudKeyFrames.begin() + i);
                --i;
            }
        }
        // add new key frames that are not in calculated existing key frames
        for (int i = 0; i < numSurroundingPosesDS; ++i) {
            bool existingFlag = false;
            for (auto iter = surroundingExistingKeyPosesID.begin(); iter != surroundingExistingKeyPosesID.end(); ++iter){
                if ((*iter) == (int)surroundingKeyPosesDS->points[i].intensity){
                    existingFlag = true;
                    break;
                }
            }
            if (existingFlag == true){
                continue;
            }else{
                int thisKeyInd = (int)surroundingKeyPosesDS->points[i].intensity;
                PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
                updateTransformPointCloudSinCos(&thisTransformation);
                surroundingExistingKeyPosesID.   push_back(thisKeyInd);
                surroundingCornerCloudKeyFrames. push_back(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
                surroundingSurfCloudKeyFrames.   push_back(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
                surroundingOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
            }
        }

        for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i) {
            *laserCloudCornerFromMap += *surroundingCornerCloudKeyFrames[i];
            *laserCloudSurfFromMap   += *surroundingSurfCloudKeyFrames[i];
            *laserCloudSurfFromMap   += *surroundingOutlierCloudKeyFrames[i];
        }
    }
    // Downsample the surrounding corner key frames (or map)
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size();
    // Downsample the surrounding surf key frames (or map)
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size();
}

void mapOptimization::downsampleCurrentScan(){

    laserCloudRawDS->clear();
    downSizeFilterScancontext.setInputCloud(laserCloudRaw);
    downSizeFilterScancontext.filter(*laserCloudRawDS);
    
    laserCloudCornerLastDS->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);
    laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size();
    // std::cout << "laserCloudCornerLastDSNum: " << laserCloudCornerLastDSNum << std::endl;

    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();
    // std::cout << "laserCloudSurfLastDSNum: " << laserCloudSurfLastDSNum << std::endl;

    laserCloudOutlierLastDS->clear();
    downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
    downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
    laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size();

    laserCloudSurfTotalLast->clear();
    laserCloudSurfTotalLastDS->clear();
    *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
    *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
    downSizeFilterSurf.setInputCloud(laserCloudSurfTotalLast);
    downSizeFilterSurf.filter(*laserCloudSurfTotalLastDS);
    laserCloudSurfTotalLastDSNum = laserCloudSurfTotalLastDS->points.size();
}

void mapOptimization::cornerOptimization(int iterCount){

    updatePointAssociateToMapSinCos();
    for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
        pointOri = laserCloudCornerLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel);
        kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
        
        if (pointSearchSqDis[4] < 1.0) {
            float cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 5; j++) {
                cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
            }
            cx /= 5; cy /= 5;  cz /= 5;

            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++) {
                float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                a22 += ay * ay; a23 += ay * az;
                a33 += az * az;
            }
            a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

            matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
            matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
            matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

            cv::eigen(matA1, matD1, matV1);

            if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                float x0 = pointSel.x;
                float y0 = pointSel.y;
                float z0 = pointSel.z;
                float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                            + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float ld2 = a012 / l12;

                float s = 1 - 0.9 * fabs(ld2);

                coeff.x = s * la;
                coeff.y = s * lb;
                coeff.z = s * lc;
                coeff.intensity = s * ld2;

                if (s > 0.1) {
                    laserCloudOri->push_back(pointOri);
                    coeffSel->push_back(coeff);
                }
            }
        }
    }
}

void mapOptimization::surfOptimization(int iterCount){
    updatePointAssociateToMapSinCos();
    for (int i = 0; i < laserCloudSurfTotalLastDSNum; i++) {
        pointOri = laserCloudSurfTotalLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel); 
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        if (pointSearchSqDis[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
                matA0.at<float>(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                matA0.at<float>(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                matA0.at<float>(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
            }
            cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

            float pa = matX0.at<float>(0, 0);
            float pb = matX0.at<float>(1, 0);
            float pc = matX0.at<float>(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps; pb /= ps; pc /= ps; pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
                if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                            pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                            pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                    planeValid = false;
                    break;
                }
            }

            if (planeValid) {
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                        + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1) {
                    laserCloudOri->push_back(pointOri);
                    coeffSel->push_back(coeff);
                }
            }
        }
    }
}

bool mapOptimization::LMOptimization(int iterCount){
    float srx = sin(transformTobeMapped[0]);
    float crx = cos(transformTobeMapped[0]);
    float sry = sin(transformTobeMapped[1]);
    float cry = cos(transformTobeMapped[1]);
    float srz = sin(transformTobeMapped[2]);
    float crz = cos(transformTobeMapped[2]);

    int laserCloudSelNum = laserCloudOri->points.size();
    if (laserCloudSelNum < 50) {
        return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
    for (int i = 0; i < laserCloudSelNum; i++) {
        pointOri = laserCloudOri->points[i];
        coeff = coeffSel->points[i];

        float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                    + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                    + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

        float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                    + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                    + ((-cry*crz - srx*sry*srz)*pointOri.x 
                    + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                    + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                    + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

        matA.at<float>(i, 0) = arx;
        matA.at<float>(i, 1) = ary;
        matA.at<float>(i, 2) = arz;
        matA.at<float>(i, 3) = coeff.x;
        matA.at<float>(i, 4) = coeff.y;
        matA.at<float>(i, 5) = coeff.z;
        matB.at<float>(i, 0) = -coeff.intensity;
    }
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0) {
        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 5; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(
                        pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
                        pow(matX.at<float>(3, 0) * 100, 2) +
                        pow(matX.at<float>(4, 0) * 100, 2) +
                        pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.05 && deltaT < 0.05) {
        return true;
    }
    return false;
}

void mapOptimization::scan2MapOptimization(){

    if (laserCloudCornerFromMapDSNum > 10 && laserCloudSurfFromMapDSNum > 100) {

        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

        for (int iterCount = 0; iterCount < 10; iterCount++) {

            laserCloudOri->clear();
            coeffSel->clear();

            cornerOptimization(iterCount);
            surfOptimization(iterCount);

            if (LMOptimization(iterCount) == true)
                break;              
        }

        transformUpdate();
    }
}


void mapOptimization::saveKeyFramesAndFactor(){

    currentRobotPosPoint.x = transformAftMapped[3];
    currentRobotPosPoint.y = transformAftMapped[4];
    currentRobotPosPoint.z = transformAftMapped[5];

//    std::cout << "X: " << currentRobotPosPoint.x << ", Y: " << currentRobotPosPoint.y << ", Z: " << currentRobotPosPoint.z << std::endl;

    bool saveThisKeyFrame = true;
    if (sqrt((previousRobotPosPoint.x-currentRobotPosPoint.x)*(previousRobotPosPoint.x-currentRobotPosPoint.x)
            +(previousRobotPosPoint.y-currentRobotPosPoint.y)*(previousRobotPosPoint.y-currentRobotPosPoint.y)
            +(previousRobotPosPoint.z-currentRobotPosPoint.z)*(previousRobotPosPoint.z-currentRobotPosPoint.z)) < 0.3){ // save keyframe every 0.3 meter 
        saveThisKeyFrame = false;
    }
    if (saveThisKeyFrame == false && !cloudKeyPoses3D->points.empty())
        return;

    previousRobotPosPoint = currentRobotPosPoint;
    /**
     * update grsam graph
     */
    if (cloudKeyPoses3D->points.empty()){
        gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
                                                            Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])), priorNoise));
        initialEstimate.insert(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
                                                Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])));
        for (int i = 0; i < 6; ++i)
            transformLast[i] = transformTobeMapped[i];
    }
    else{
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
                                            Point3(transformLast[5], transformLast[3], transformLast[4]));
        gtsam::Pose3 poseTo   = Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                            Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4]));
        gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->points.size()-1, cloudKeyPoses3D->points.size(), poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->points.size(), Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                                                            Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])));
    }
    /**
     * update iSAM
     */
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    /**
     * save key poses
     */
    PointType thisPose3D;
    PointTypePose thisPose6D;
    Pose3 latestEstimate;

    isamCurrentEstimate = isam->calculateEstimate();
    latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);

    thisPose3D.x = latestEstimate.translation().y();
    thisPose3D.y = latestEstimate.translation().z();
    thisPose3D.z = latestEstimate.translation().x();
    thisPose3D.intensity = cloudKeyPoses3D->points.size(); // this can be used as index
    cloudKeyPoses3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
    thisPose6D.roll  = latestEstimate.rotation().pitch();
    thisPose6D.pitch = latestEstimate.rotation().yaw();
    thisPose6D.yaw   = latestEstimate.rotation().roll(); // in camera frame
    thisPose6D.time = timeLaserOdometry;
    cloudKeyPoses6D->push_back(thisPose6D);

    /**
     * save updated transform
     */
    if (cloudKeyPoses3D->points.size() > 1){
        transformAftMapped[0] = latestEstimate.rotation().pitch();
        transformAftMapped[1] = latestEstimate.rotation().yaw();
        transformAftMapped[2] = latestEstimate.rotation().roll();
        transformAftMapped[3] = latestEstimate.translation().y();
        transformAftMapped[4] = latestEstimate.translation().z();
        transformAftMapped[5] = latestEstimate.translation().x();

        for (int i = 0; i < 6; ++i){
            transformLast[i] = transformAftMapped[i];
            transformTobeMapped[i] = transformAftMapped[i];
        }
    }

    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisOutlierKeyFrame(new pcl::PointCloud<PointType>());

    pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
    pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);
    pcl::copyPointCloud(*laserCloudOutlierLastDS, *thisOutlierKeyFrame);

    /* 
        Scan Context loop detector 
        - ver 1: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
        - ver 2: using downsampled original point cloud (/full_cloud_projected + downsampling)
        */
    bool usingRawCloud = true;
    if( usingRawCloud ) { // v2 uses downsampled raw point cloud, more fruitful height information than using feature points (v1)
        pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudRawDS,  *thisRawCloudKeyFrame);
        scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
    }
    else { // v1 uses thisSurfKeyFrame, it also works. (empirically checked at Mulran dataset sequences)
        scManager.makeAndSaveScancontextAndKeys(*thisSurfKeyFrame); 
    }
    
    cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);
    outlierCloudKeyFrames.push_back(thisOutlierKeyFrame);
    keyframeStamps.push_back(timeLaserOdometry);
} // saveKeyFramesAndFactor


void mapOptimization::correctPoses(){
    if (aLoopIsClosed == true){
        recentCornerCloudKeyFrames. clear();
        recentSurfCloudKeyFrames.   clear();
        recentOutlierCloudKeyFrames.clear();
        // update key poses
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i){
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().z();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().x();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
            cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
        }

        aLoopIsClosed = false;
    }
}

void mapOptimization::clearCloud(){
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();  
    laserCloudCornerFromMapDS->clear();
    laserCloudSurfFromMapDS->clear();   
}

void mapOptimization::run(){

    if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudOutlierLast && newLaserOdometry)
    {

        newLaserCloudCornerLast = false; newLaserCloudSurfLast = false; newLaserCloudOutlierLast = false; newLaserOdometry = false;

        std::lock_guard<std::mutex> lock(mtx);

        if (1) {


            transformAssociateToMap();

            extractSurroundingKeyFrames();

            downsampleCurrentScan();

            scan2MapOptimization();

            saveKeyFramesAndFactor();

            correctPoses();

            clearCloud();
        }
    }
}

}

