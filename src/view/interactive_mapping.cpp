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

lego_loam::ImageProjection image;
lego_loam::FeatureAssociation feature;
lego_loam::mapOptimization mapOpt;
std::string file_directory;


InteractiveMapping::InteractiveMapping(){
  
}

InteractiveMapping::~InteractiveMapping() {
  
}

bool InteractiveMapping::start_mapping(guik::ProgressInterface& progress)
{
  rosbag::Bag bag;
  std::vector<std::string> topics;
  topics.push_back(std::string("/rslidar_points"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  rosbag::View::iterator it = view.begin();
  
  mapOpt.startLoopClosure();

  while(it !=  view.end())
  {
    auto m = *it;
    ++it;
    std::string topic = m.getTopic();

    if(topic == "/rslidar_points")
    {
        pcl::PCLPointCloud2 *pointCloud2 = new pcl::PCLPointCloud2;
        sensor_msgs::PointCloud2ConstPtr pclmsg = m.instantiate<sensor_msgs::PointCloud2>();
        pcl_conversions::toPCL(*pclmsg, *pointCloud2);
        pcl::PointCloud<pcl::PointXYZI> pcs;
        pcl::fromPCLPointCloud2(*pointCloud2, pcs);

        pcl::PointCloud<pcl::PointXYZI> mapCornerCloud, mapSurfCloud;
        float PoseAftMapped[6];

        image.loadPointCloud(pcs);

        feature.featureOdometry(image);
        mapOpt.mapBuild(feature, pcs, mapCornerCloud, mapSurfCloud, PoseAftMapped);
        image.resetParameters();


        // InteractiveKeyFrame::Ptr keyframe = std::make_shared<InteractiveKeyFrame>(keyframe_dir, graph.get());
        // if(!keyframe->node) {
        //   std::cerr << "error : failed to load keyframe!!" << std::endl;
        //   std::cerr << "      : " << keyframe_dir << std::endl;
        // } else {
        //   keyframes[keyframe->id()] = keyframe;
        //   progress.increment();
        // }

    }

  }
  
}

bool InteractiveMapping::load_map_data(const std::string& directory, guik::ProgressInterface& progress) {
  // load graph file
  progress.set_title("Opening " + directory);
  progress.increment();
  progress.set_text("loading dataset");
  file_directory = directory;

  progress.set_maximum(10);
  for(int i = 0; i < 10; i++)
  {
    progress.increment();
    sleep(1);
  }
   // load keyframes
  
  progress.set_text("loading keyframes");
  

  // // load keyframes
  // progress.increment();
  // progress.set_text("loading keyframes");

  return true;
}
