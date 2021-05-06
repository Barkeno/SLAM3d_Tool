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

#ifndef FEATUREASSOCIATION
#define FEATUREASSOCIATION

#include "utility.h"
#include "imageProjection.h"

namespace lego_loam
{

class FeatureAssociation{

public:

    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

    pcl::VoxelGrid<PointType> downSizeFilter;

    double timeScanCur;
    double timeNewSegmentedCloud;
    double timeNewSegmentedCloudInfo;
    double timeNewOutlierCloud;

    bool newSegmentedCloud;
    bool newSegmentedCloudInfo;
    bool newOutlierCloud;

    int systemInitCount;
    bool systemInited;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    int imuPointerFront;
    int imuPointerLast;
    int imuPointerLastIteration;

    float imuRollStart, imuPitchStart, imuYawStart;
    float cosImuRollStart, cosImuPitchStart, cosImuYawStart, sinImuRollStart, sinImuPitchStart, sinImuYawStart;
    float imuRollCur, imuPitchCur, imuYawCur;

    float imuVeloXStart, imuVeloYStart, imuVeloZStart;
    float imuShiftXStart, imuShiftYStart, imuShiftZStart;

    float imuVeloXCur, imuVeloYCur, imuVeloZCur;
    float imuShiftXCur, imuShiftYCur, imuShiftZCur;

    float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
    float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

    float imuAngularRotationXCur, imuAngularRotationYCur, imuAngularRotationZCur;
    float imuAngularRotationXLast, imuAngularRotationYLast, imuAngularRotationZLast;
    float imuAngularFromStartX, imuAngularFromStartY, imuAngularFromStartZ;

    double imuTime[imuQueLength];
    float imuRoll[imuQueLength];
    float imuPitch[imuQueLength];
    float imuYaw[imuQueLength];

    float imuAccX[imuQueLength];
    float imuAccY[imuQueLength];
    float imuAccZ[imuQueLength];

    float imuVeloX[imuQueLength];
    float imuVeloY[imuQueLength];
    float imuVeloZ[imuQueLength];

    float imuShiftX[imuQueLength];
    float imuShiftY[imuQueLength];
    float imuShiftZ[imuQueLength];

    float imuAngularVeloX[imuQueLength];
    float imuAngularVeloY[imuQueLength];
    float imuAngularVeloZ[imuQueLength];

    float imuAngularRotationX[imuQueLength];
    float imuAngularRotationY[imuQueLength];
    float imuAngularRotationZ[imuQueLength];



    int skipFrameNum;
    bool systemInitedLM;

    int laserCloudCornerLastNum;
    int laserCloudSurfLastNum;

    int *pointSelCornerInd;
    float *pointSearchCornerInd1;
    float *pointSearchCornerInd2;

    int *pointSelSurfInd;
    float *pointSearchSurfInd1;
    float *pointSearchSurfInd2;
    float *pointSearchSurfInd3;

    float transformCur[6];
    float transformSum[6];

    float imuRollLast, imuPitchLast, imuYawLast;
    float imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ;
    float imuVeloFromStartX, imuVeloFromStartY, imuVeloFromStartZ;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;


    bool isDegenerate;
    cv::Mat matP;

    int frameCount;

public:
    float startOrientation;
    float endOrientation;
    float orientationDiff;

    vector<int> startRingIndex;
    vector<int> endRingIndex;

    vector<bool> segmentedCloudGroundFlag;
    vector<int>  segmentedCloudColInd;
    vector<float>  segmentedCloudRange;

    void imuHandler();

public:

    FeatureAssociation();

    void initializationValue();

    void updateImuRollPitchYawStartSinCos();


    void ShiftToStartIMU(float pointTime);

    void VeloToStartIMU();

    void TransformToStartIMU(PointType *p);

    void AccumulateIMUShiftAndRotation();

    void adjustDistortion();

    void calculateSmoothness();

    void markOccludedPoints();

    void extractFeatures();

//    void publishCloud();

    void featureOdometry(ImageProjection &imagePro);



    void TransformToStart(PointType const * const pi, PointType * const po);

    void TransformToEnd(PointType const * const pi, PointType * const po);

    void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz, 
                           float alx, float aly, float alz, float &acx, float &acy, float &acz);

    void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, 
                            float &ox, float &oy, float &oz);

    double rad2deg(double radians);

    double deg2rad(double degrees);

    void findCorrespondingCornerFeatures(int iterCount);

    void findCorrespondingSurfFeatures(int iterCount);

    bool calculateTransformationSurf(int iterCount);

    bool calculateTransformationCorner(int iterCount);

    bool calculateTransformation(int iterCount);

    void checkSystemInitialization();

    void updateInitialGuess();

    void updateTransformation();

    void integrateTransformation();

    void publishOdometry();

    void adjustOutlierCloud();

    void publishCloudsLast();

    void runFeatureAssociation();
};

}

#endif
