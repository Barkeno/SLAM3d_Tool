cmake_minimum_required(VERSION 2.8.3)
project(interactive_slam)

set (IS_VERSION_MAJOR 1)
set (IS_VERSION_MINOR 0)
set (IS_VERSION_REVISION 5)


if("$ENV{ROS_DISTRO}" STRGREATER "melodic")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# find_package(catkin REQUIRED COMPONENTS roscpp ndt_omp hdl_graph_slam roslib)

find_package(GLM REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED filesystem program_options)

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport
  rosbag_storage
  rosbag

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)


###########
## Build ##
###########

include_directories(
  include
  thirdparty/gl3w
  thirdparty/imgui
  thirdparty/imgui/examples
  thirdparty/portable-file-dialogs
  thirdparty/ndt_omp/include
  include/mapping/Lego_Loam/include
  /opt/ros/melodic/include
  ${GLM_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

###########
## Build ##
###########

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_library(ndt_omp STATIC
  thirdparty/ndt_omp/src/pclomp/gicp_omp.cpp
  thirdparty/ndt_omp/src/pclomp/ndt_omp.cpp
  thirdparty/ndt_omp/src/pclomp/voxel_grid_covariance_omp.cpp
)

# imgui
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)

###########
## Build ##
###########

add_library(imgui STATIC
  thirdparty/gl3w/gl3w.c
  thirdparty/imgui/imgui.cpp
  thirdparty/imgui/imgui_demo.cpp
  thirdparty/imgui/imgui_draw.cpp
  thirdparty/imgui/imgui_widgets.cpp
  thirdparty/imgui/examples/imgui_impl_glfw.cpp
  thirdparty/imgui/examples/imgui_impl_opengl3.cpp
)
target_link_libraries(imgui
  ${OPENGL_LIBRARIES}
  glfw
  dl
)


add_library(guik STATIC
  src/glk/mesh.cpp
  src/glk/lines.cpp
  src/glk/colormap.cpp
  src/glk/glsl_shader.cpp
  src/glk/frame_buffer.cpp
  src/glk/pointcloud_buffer.cpp
  src/glk/primitives/primitives.cpp
  src/glk/loaders/ply_loader.cpp
  src/guik/gl_canvas.cpp
  src/guik/model_control.cpp
  src/guik/camera_control.cpp
  src/guik/projection_control.cpp
  src/guik/imgui_application.cpp
)

###########
## Build ##
###########

add_library(lego_loam STATIC
  src/mapping/Lego_Loam/src/featureAssociation.cpp
  src/mapping/Lego_Loam/src/imageProjection.cpp
  src/mapping/Lego_Loam/src/mapOptmization.cpp 
  src/mapping/Lego_Loam/src/Scancontext.cpp 
  src/mapping/Lego_Loam/src/utility.cpp
)

target_link_libraries(lego_loam
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    gtsam
    Eigen3::Eigen
)

add_executable(interactive_slam
  src/interactive_slam.cpp
  src/view/edge_view.cpp
  src/view/graph_slam.cpp
  src/view/interactive_graph.cpp
  src/view/interactive_mapping.cpp
  src/view/keyframe.cpp
  src/view/vertex_view.cpp
  src/view/information_matrix_calculator.cpp
  src/view/interactive_keyframe.cpp
  src/g2o/robust_kernel_io.cpp
  src/view/window/registration_methods.cpp
  src/view/window/version_modal.cpp
  src/view/window/graph_edit_window.cpp
  src/view/window/edge_refinement_window.cpp
  src/view/window/plane_detection_window.cpp
  src/view/window/plane_alignment_modal.cpp
  src/view/window/manual_loop_close_modal.cpp
  src/view/window/automatic_loop_close_window.cpp
)
target_link_libraries(interactive_slam
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
  ${OPENGL_LIBRARIES}
  ${catkin_LIBRARIES}
  glfw
  guik
  imgui
  ndt_omp
  lego_loam
)

file(COPY data DESTINATION .)
