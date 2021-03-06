#ifndef INTERACTIVE_KEYFRAME_HPP
#define INTERACTIVE_KEYFRAME_HPP

#include <boost/any.hpp>
#include <view/keyframe.hpp>


struct InteractiveKeyFrame : public KeyFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<InteractiveKeyFrame>;

  InteractiveKeyFrame(const std::string& directory, g2o::HyperGraph* graph);

  InteractiveKeyFrame(pcl::PointCloud<pcl::PointXYZI> mapSurfCloud, float* PoseAftMapped);
  
  virtual ~InteractiveKeyFrame() override;

  std::vector<int> neighbors(const Eigen::Vector3f& pt, double radius);

  pcl::PointCloud<pcl::Normal>::Ptr normals();

public:
  Eigen::Vector3f min_pt;
  Eigen::Vector3f max_pt;

  boost::any kdtree_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
};

#endif