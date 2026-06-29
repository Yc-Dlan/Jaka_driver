#ifndef ESDF_TOOLS_INCLUDE_PARAMETERS_H_
#define ESDF_TOOLS_INCLUDE_PARAMETERS_H_
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#define PROBABILISTIC
//#define HASH_TABLE
#define BLOCK
#define BITWISE
#define DEBUG
//#define SIGNED_NEEDED

namespace fiesta {

// Connectivity used in BFS
const static int num_dirs_ = 24; // faces 2 steps
const Eigen::Vector3i dirs_[num_dirs_] = {Eigen::Vector3i(-1, 0, 0), Eigen::Vector3i(1, 0, 0),
                                          Eigen::Vector3i(0, -1, 0), Eigen::Vector3i(0, 1, 0),
                                          Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, 1),

                                          Eigen::Vector3i(-1, -1, 0), Eigen::Vector3i(1, 1, 0),
                                          Eigen::Vector3i(0, -1, -1), Eigen::Vector3i(0, 1, 1),
                                          Eigen::Vector3i(-1, 0, -1), Eigen::Vector3i(1, 0, 1),
                                          Eigen::Vector3i(-1, 1, 0), Eigen::Vector3i(1, -1, 0),
                                          Eigen::Vector3i(0, -1, 1), Eigen::Vector3i(0, 1, -1),
                                          Eigen::Vector3i(1, 0, -1), Eigen::Vector3i(-1, 0, 1),

                                          Eigen::Vector3i(-2, 0, 0), Eigen::Vector3i(2, 0, 0),
                                          Eigen::Vector3i(0, -2, 0), Eigen::Vector3i(0, 2, 0),
                                          Eigen::Vector3i(0, 0, -2), Eigen::Vector3i(0, 0, 2)};


struct Parameters {
  // resolution
  double resolution_;
  // hash table implementation only
  int reserved_size_;
  // array implementation only
  Eigen::Vector3d l_cornor_, r_cornor_, map_size_;
  // intrinsic parameters of camera, only used if your input is depth image
  double center_x_, center_y_, focal_x_, focal_y_;
  // parameters of probabilistic occupancy updating
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;
  //depth filter
  bool use_depth_filter_;
  double depth_filter_max_dist_, depth_filter_min_dist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  // ray cast parameters
  double min_ray_length_, max_ray_length_;
  // visualization
  double slice_vis_max_dist_;
  int slice_vis_level_, vis_lower_bound_, vis_upper_bound_;
  // frequency of updating
  double update_esdf_every_n_sec_;
  // frequency of visualization
  int visualize_every_n_updates_;
  // number of thread used
  int ray_cast_num_thread_;
  // local map
  bool global_vis_, global_update_, global_map_;
  Eigen::Vector3d radius_;
  // transforms
  Eigen::Matrix4d T_B_C_, T_D_B_;

  // 修改：接收 ROS 2 Node 指针
  void SetParameters(rclcpp::Node::SharedPtr node);
};
}
#endif //ESDF_TOOLS_INCLUDE_PARAMETERS_H_