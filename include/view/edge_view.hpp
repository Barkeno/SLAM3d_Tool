#ifndef HDL_GRAPH_SLAM_EDGE_VIEW_HPP
#define HDL_GRAPH_SLAM_EDGE_VIEW_HPP

#include <Eigen/Core>

#include <glk/glsl_shader.hpp>
#include <view/graph_slam.hpp>
#include <view/line_buffer.hpp>
#include <view/drawable_object.hpp>

class EdgeView : public DrawableObject {
public:
  using Ptr = std::shared_ptr<EdgeView>;

  static EdgeView::Ptr create(g2o::HyperGraph::Edge* edge, LineBuffer& line_buffer);

  EdgeView(g2o::HyperGraph::Edge* edge, LineBuffer& line_buffer);
  virtual ~EdgeView();

  long id() const;
  virtual Eigen::Vector3f representative_point() const = 0;

  virtual void context_menu(bool& do_delete);

private:
  EdgeView();

protected:
  LineBuffer& line_buffer;

public:
  g2o::HyperGraph::Edge* edge;
};

#endif
