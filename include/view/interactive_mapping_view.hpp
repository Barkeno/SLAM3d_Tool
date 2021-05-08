#ifndef INTERACTIVE_MAPPING_VIEW_HPP
#define INTERACTIVE_MAPPING_VIEW_HPP

#include <mutex>
#include <unordered_map>
#include <glk/glsl_shader.hpp>

#include <view/interactive_mapping.hpp>

#include <view/edge_view.hpp>
#include <view/vertex_view.hpp>
#include <view/keyframe_view.hpp>
#include <view/line_buffer.hpp>
#include <view/drawable_object.hpp>
#include <view/mapping_keyframe_view.hpp>


class InteractiveMappingView : public InteractiveMapping {
public:
  InteractiveMappingView() { }
  virtual ~InteractiveMappingView() override {}

  void init_gl() { line_buffer.reset(new LineBuffer()); }

  void update_view() {
    bool keyframe_inserted = false;
    std::cout << "mappingKeyframes: " << mappingkeyframes.size() << std::endl;
    for (const auto& key_item : mappingkeyframes) {
      auto& keyframe = key_item.second;
      // auto found = keyframes_view_map.find(keyframe);
      // if (found == keyframes_view_map.end()) {
        keyframe_inserted = true;
        keyframes_view.push_back(std::make_shared<MappingKeyFrameView>(keyframe));
        keyframes_view_map[keyframe] = keyframes_view.back();

        // vertices_view.push_back(keyframes_view.back());
        // vertices_view_map[keyframe->id()] = keyframes_view.back();

        drawables.push_back(keyframes_view.back());
      // }
    }

    // if (keyframe_inserted) {
    //   std::sort(keyframes_view.begin(), keyframes_view.end(), [=](const KeyFrameView::Ptr& lhs, const KeyFrameView::Ptr& rhs) { return lhs->lock()->id() < rhs->lock()->id(); });
    // }

  }

    

  void draw(const DrawFlags& flags, glk::GLSLShader& shader) {
    std::lock_guard<std::mutex> lock(mapping_mutex);
    update_view();
    line_buffer->clear();

    for (auto& drawable : drawables) {
      if (drawable->available()) {
        drawable->draw(flags, shader);
      }
    }

    line_buffer->draw(shader);
  }

  

public:
  std::unique_ptr<LineBuffer> line_buffer;

  std::vector<MappingKeyFrameView::Ptr> keyframes_view;
  std::unordered_map<InteractiveKeyFrame::Ptr, MappingKeyFrameView::Ptr> keyframes_view_map;

  std::vector<VertexView::Ptr> vertices_view;
  std::unordered_map<long, VertexView::Ptr> vertices_view_map;

  std::vector<EdgeView::Ptr> edges_view;
  std::unordered_map<g2o::HyperGraph::Edge*, EdgeView::Ptr> edges_view_map;

  std::vector<DrawableObject::Ptr> drawables;
};


#endif