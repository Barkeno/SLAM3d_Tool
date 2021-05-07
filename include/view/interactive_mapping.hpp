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

  bool start_mapping(guik::ProgressInterface& progress);

  std::unordered_map<long, InteractiveKeyFrame::Ptr> keyframes;

  std::string file_directory;
};

#endif