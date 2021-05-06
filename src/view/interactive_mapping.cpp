#include <view/interactive_mapping.hpp>

#include <chrono>
#include <boost/filesystem.hpp>





InteractiveMapping::InteractiveMapping(){
  
}

InteractiveMapping::~InteractiveMapping() {
  
}

bool InteractiveMapping::load_map_data(const std::string& directory, guik::ProgressInterface& progress) {
  // load graph file
  progress.set_title("Opening " + directory);
  progress.set_text("loading dataset");
  std::cout << "Load dataset : " << directory << std::endl;
  

  // // load keyframes
  // progress.increment();
  // progress.set_text("loading keyframes");

  return true;
}
