#ifndef HDL_GRAPH_SLAM_GRAPH_EDIT_WINDOW_HPP
#define HDL_GRAPH_SLAM_GRAPH_EDIT_WINDOW_HPP

#include <memory>

#include <imgui.h>
#include <guik/gl_canvas.hpp>
#include <view/interactive_graph_view.hpp>


class GraphEditWindow {
public:
  GraphEditWindow(std::shared_ptr<InteractiveGraphView>& graph);
  ~GraphEditWindow();

  void show() { show_window = true; }
  void draw_ui();

private:
  bool show_window;
  int selected_vertex;
  std::shared_ptr<InteractiveGraphView>& graph;
};

#endif