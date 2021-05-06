#include <view/interactive_mapping.hpp>

#include <chrono>
#include <boost/filesystem.hpp>





InteractiveMapping::InteractiveMapping(){
  
}

InteractiveMapping::~InteractiveMapping() {
  
}

bool InteractiveMapping::start_mapping(guik::ProgressInterface& progress)
{
    while(1)
    {
      std::cout << "mapping.." << std::endl;
    }
}

bool InteractiveMapping::load_map_data(const std::string& directory, guik::ProgressInterface& progress) {
  // load graph file
  progress.set_title("Opening " + directory);
  progress.increment();
  
  progress.set_text("loading dataset");
  std::cout << "Load dataset : " << directory << std::endl;
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
