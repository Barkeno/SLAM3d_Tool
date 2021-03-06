#ifndef HDL_GRAPH_SLAM_PLANE_ALIGNMENT_MODAL_HPP
#define HDL_GRAPH_SLAM_PLANE_ALIGNMENT_MODAL_HPP

#include <memory>

#include <imgui.h>
#include <guik/gl_canvas.hpp>
#include <view/window/robust_kernels.hpp>
#include <view/vertex_plane_view.hpp>
#include <view/interactive_graph_view.hpp>

namespace g2o {
class VertexPlane;
}

class PlaneAlignmentModal {
public:
  PlaneAlignmentModal(std::shared_ptr<InteractiveGraphView>& graph);
  ~PlaneAlignmentModal();

  bool set_begin_plane(int plane_id);
  bool set_end_plane(int plane_id);

  bool has_begin_plane();

  bool run();

  void close();

  void draw_gl(glk::GLSLShader& shader);

private:
  const std::shared_ptr<InteractiveGraphView>& graph;

  std::unique_ptr<VertexPlaneView> plane_begin;
  std::unique_ptr<VertexPlaneView> plane_end;

  int plane_mode;
  float information_scale;

  RobustKernels robust_kernel;
};

#endif