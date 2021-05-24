#ifndef INTERACTIVE_MAPPING_HPP
#define INTERACTIVE_MAPPING_HPP

#include <regex>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <atomic>

#include <guik/progress_interface.hpp>


#include <view/interactive_keyframe.hpp>
#include <view/graph_slam.hpp>
#include <view/parameter_server.hpp>
#include <view/information_matrix_calculator.hpp>

namespace g2o {
class VertexPlane;
class VertexSE3;
class VertexSE3Edge;
}  // namespace g2o


class InformationMatrixCalculator;





/**
 * @brief mapping
 *
 */
class InteractiveMapping : protected GraphSLAM {
public:
  InteractiveMapping();
  virtual ~InteractiveMapping();

  bool load_raw_data(const std::string& directory, guik::ProgressInterface& progress);

  bool start_mapping();
  bool stop_mapping();
  void mapping();

  std::unordered_map<long, InteractiveKeyFrame::Ptr> mappingkeyframes;
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;

  std::mutex mapping_mutex;
  std::thread mapping_thread;

  std::atomic_bool running;

private:
  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  g2o::VertexPlane* floor_node;
};

#endif