//#include "utility.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace std;



string pointCloudTopic = "/velodyne_points";
// extern const string pointCloudTopic = "/kitti_scan";
// extern const string pointCloudTopic = "/os1_points";
string imuTopic = "/imu/data";

// Save pcd
string fileDirectory = "/tmp/";

// Using velodyne cloud "ring" channel for image projection (other lidar may have different name for this channel, change "PointXYZIR" below)
bool useCloudRing = false; // if true, ang_res_y and ang_bottom are not used

// VLP-16
// int N_SCAN = 16;
// int Horizon_SCAN = 1800;
// float ang_res_x = 0.2;
// float ang_res_y = 2.0;
// float ang_bottom = 15.0+0.1;
// int groundScanInd = 7;

// VLP-32
// int N_SCAN = 32;
// int Horizon_SCAN = 1800;
// float ang_res_x = 360.0/float(Horizon_SCAN);
// float ang_res_y = 40/float(N_SCAN-1);
// float ang_bottom = 20;
// int groundScanInd = 12;

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// VLS-128
int N_SCAN = 128;
int Horizon_SCAN = 1800;
float ang_res_x = 0.2;
float ang_res_y = 0.3;
float ang_bottom = 25.0;
int groundScanInd = 10;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet (LeGO-LOAM needs 9-DOF IMU), please just publish point cloud data
// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

bool loopClosureEnableFlag = true;
double mappingProcessInterval = 0.3;

float scanPeriod = 0.1;
int systemDelay = 0;
//int imuQueLength = 200;

float sensorMinimumRange = 1.0;
float sensorMountAngle = 0.0;
float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy
int segmentValidPointNum = 5;
int segmentValidLineNum = 3;
float segmentAlphaX = ang_res_x / 180.0 * M_PI;
float segmentAlphaY = ang_res_y / 180.0 * M_PI;


int edgeFeatureNum = 2;
int surfFeatureNum = 4;
int sectionsTotal = 6;
float edgeThreshold = 0.1;
float surfThreshold = 0.1;
float nearestFeatureSearchSqDist = 25;


// Mapping Params
float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
int   surroundingKeyframeSearchNum = 50; // submap size (when loop closure enabled)

// history key frames (history submap for loop closure)
float historyKeyframeSearchRadius = 20.0; // NOT used in Scan Context-based loop detector / default 7.0; key frame that is within n meters from current pose will be considerd for loop closure
int   historyKeyframeSearchNum = 25; // 2n+1 number of history key frames will be fused into a submap for loop closure
float historyKeyframeFitnessScore = 1.5; // default 0.3; the smaller the better alignment

float globalMapVisualizationSearchRadius = 1500.0; // key frames with in n meters will be visualized



