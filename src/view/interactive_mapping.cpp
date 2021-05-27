#include <view/interactive_mapping.hpp>

#include <chrono>
#include <boost/filesystem.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include "imageProjection.h"
#include "featureAssociation.h"
#include "mapOptmization.h"
#include "dumpGraph.h"
#include "wheelOdometry.h"

lego_loam::ImageProjection image;
lego_loam::FeatureAssociation feature;
lego_loam::mapOptimization mapOpt;
lego_loam::WheelOdometry wheelOdm;
std::string file_directory;


InteractiveMapping::InteractiveMapping(){
  running = false;
}

InteractiveMapping::~InteractiveMapping() {
  if (running) {
    running = false;
  }

  if (mapping_thread.joinable()) {
    mapping_thread.join();
  }
}

bool InteractiveMapping::start_mapping()
{
  if (!running) {
      running = true;
      mapping_thread = std::thread([&]() { mapping(); });
    }
  return true;
}

bool InteractiveMapping::stop_mapping()
{
  if (running) {
      running = false;
      
      mapOpt.endLoopClosure();
      std::lock_guard<std::mutex> lock(mapping_mutex);
      dump("/tmp/dump", *(mapOpt.isam), mapOpt.isamCurrentEstimate, mapOpt.keyframeStamps, mapOpt.cornerCloudKeyFrames, mapOpt.surfCloudKeyFrames, mapOpt.outlierCloudKeyFrames);
      mapOpt.allocateMemory();
      image.resetParameters();
      mapping_thread.join();
      std::cout << "end mapping!" << std::endl;
    }
  return true;
}

void InteractiveMapping::mapping()
{
  rosbag::Bag bag;
  bag.open(file_directory, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("/rslidar_points"));
  topics.push_back(std::string("/wheel_speed"));
  topics.push_back(std::string("/wheel_rear_left_dir"));
  topics.push_back(std::string("/wheel_rear_right_dir"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  rosbag::View::iterator it = view.begin();
  
  int keycount = 0;
  mapOpt.startLoopClosure();

  while (running) 
  {
    if(it !=  view.end())
    {
      auto m = *it;
      ++it;
      keycount++;
      std::string topic = m.getTopic();

      if(topic == "/rslidar_points")
      {
          pcl::PCLPointCloud2 *pointCloud2 = new pcl::PCLPointCloud2;
          sensor_msgs::PointCloud2ConstPtr pclmsg = m.instantiate<sensor_msgs::PointCloud2>();
          pcl_conversions::toPCL(*pclmsg, *pointCloud2);
          pcl::PointCloud<pcl::PointXYZI> pcs;
          pcl::fromPCLPointCloud2(*pointCloud2, pcs);

          pcl::PointCloud<pcl::PointXYZI> mapCornerCloud, mapSurfCloud, nullCloud;
          float PoseAftMapped[6];

          image.loadPointCloud(pcs);

          feature.featureOdometry(image);
          mapOpt.mapBuild(feature, pcs, mapCornerCloud, mapSurfCloud, PoseAftMapped);
          
          image.resetParameters();
          for(int i = 0; i < mapCornerCloud.size(); i++)
          {
            float tempx, tempy, tempz;
            tempx = mapCornerCloud[i].x;
            tempy = mapCornerCloud[i].y;
            tempz = mapCornerCloud[i].z;
            mapCornerCloud[i].x = tempz;
            mapCornerCloud[i].y = tempx;
            mapCornerCloud[i].z = tempy;
          }


          std::lock_guard<std::mutex> lock(mapping_mutex);
          // InteractiveKeyFrame::Ptr keyframe = std::make_shared<InteractiveKeyFrame>(mapCornerCloud, PoseAftMapped);
          // mappingkeyframes[0] = keyframe;
          mappingkeyframes[0] = std::make_shared<InteractiveKeyFrame>(mapCornerCloud, PoseAftMapped);
          float fx, fy, fthita;
          wheelOdm.getOdometry(fx, fy, fthita);
          float PoseWheelOdometry[6];
          PoseWheelOdometry[5] = fx;
          PoseWheelOdometry[3] = fy;
          PoseWheelOdometry[4] = PoseAftMapped[4];
          PoseWheelOdometry[0] = PoseAftMapped[0];
          PoseWheelOdometry[1] = PoseAftMapped[1];
          PoseWheelOdometry[2] = PoseAftMapped[2];
          

          // mappingkeyframes[1] = std::make_shared<InteractiveKeyFrame>(nullCloud, PoseWheelOdometry);

          std::cout << "WheelOdom: " << "x: " << fx << ", y: " << fy << ", thita: " << fthita << endl;
          std::cout << "LidarOdom: " << "x: " << PoseAftMapped[5] << ", y: " << PoseAftMapped[3] << endl;

          delete pointCloud2;
      }
      if(topic == "/wheel_speed")
      {
        cout << "wheel_speed " << endl;
        self_msgs::wheel_speed::ConstPtr msg = m.instantiate<self_msgs::wheel_speed>();
        wheelOdm.MsgWheelSpeed(msg);
      }
      if(topic == "/wheel_rear_left_dir")
      {
        cout << "wheel_rear_left_dir " << endl;
        std_msgs::Int32::ConstPtr msg = m.instantiate<std_msgs::Int32>();
        wheelOdm.MsgWheelRLDir(msg);
      }
      if(topic == "/wheel_rear_right_dir")
      {
        cout << "wheel_rear_right_dir " << endl;
        std_msgs::Int32::ConstPtr msg = m.instantiate<std_msgs::Int32>();
        wheelOdm.MsgWheelRRDir(msg);
      }
    }
    usleep(100);
  }
}

bool InteractiveMapping::load_raw_data(const std::string& directory, guik::ProgressInterface& progress) {
  // load graph file
  progress.set_title("Opening " + directory);
  progress.increment();
  progress.set_text("loading dataset");
  file_directory = directory;

  progress.set_maximum(3);
  for(int i = 0; i < 3; i++)
  {
    progress.increment();
    sleep(1);
  } 

  return true;
}
