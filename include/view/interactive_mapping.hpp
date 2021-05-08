#ifndef INTERACTIVE_MAPPING_HPP
#define INTERACTIVE_MAPPING_HPP

#include <regex>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <guik/progress_interface.hpp>


#include <view/interactive_keyframe.hpp>





/**
 * @brief mapping
 *
 */
class InteractiveMapping{
public:
  InteractiveMapping();
  virtual ~InteractiveMapping();

  bool load_map_data(const std::string& directory, guik::ProgressInterface& progress);

  bool start_mapping();
  bool stop_mapping();
  void mapping();

  std::unordered_map<long, InteractiveKeyFrame::Ptr> mappingkeyframes;

  std::mutex mapping_mutex;
  std::thread mapping_thread;

  std::atomic_bool running;
};

#endif