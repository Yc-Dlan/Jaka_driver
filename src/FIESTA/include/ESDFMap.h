//
// Created by tommy on 12/17/18.
// Modified for ROS 2
//

#ifndef ESDF_MAP_H
#define ESDF_MAP_H

#include <Eigen/Eigen>
#include <iostream>
#include <queue>
#include <vector>
#include <unordered_map>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>

// ROS 2 Includes
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "parameters.h"

namespace fiesta {
#ifdef HASH_TABLE
// Eigen::Matrix hashing function
template<typename transform_>
struct MatrixHash : std::unary_function<transform_, size_t> {
  std::size_t operator()(transform_ const &matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename transform_::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};
#endif
class ESDFMap {
  struct QueueElement {
    Eigen::Vector3i point_;
    double distance_;
    bool operator<(const QueueElement &element) const {
      return distance_ > element.distance_;
    }
  };

 private:
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;
  const double Logit(const double &x) const;
  bool Exist(const int &idx);
  double Dist(Eigen::Vector3i a, Eigen::Vector3i b);

  bool PosInMap(Eigen::Vector3d pos);
  bool VoxInRange(Eigen::Vector3i vox, bool current_vec = true);
  void Vox2Pos(Eigen::Vector3i vox, Eigen::Vector3d &pos);
  int Vox2Idx(Eigen::Vector3i vox);
  int Vox2Idx(Eigen::Vector3i vox, int sub_sampling_factor);
  void Pos2Vox(Eigen::Vector3d pos, Eigen::Vector3i &vox);
  Eigen::Vector3i Idx2Vox(int idx);

#ifdef HASH_TABLE
  void IncreaseCapacity(int &old_size, int new_size);
  int FindAndInsert(Eigen::Vector3i hash_key);
  std::unordered_map<Eigen::Vector3i, int, MatrixHash<Eigen::Vector3i>> hash_table_;
  int count, reserve_size_;
#ifdef BLOCK
  int block_, block_size_, block_bit_;
#endif
  std::vector<Eigen::Vector3i> vox_buffer_;
#else 
  Eigen::Vector3d map_size_;
  Eigen::Vector3d min_range_, max_range_; 
  Eigen::Vector3i grid_size_;             
  int grid_size_yz_;
#endif

#ifdef PROBABILISTIC
  std::vector<double> occupancy_buffer_; 
#else
  std::vector<unsigned char> occupancy_buffer_; 
#endif
  std::vector<double> distance_buffer_;
  std::vector<int> num_hit_, num_miss_;
  std::vector<Eigen::Vector3i> closest_obstacle_;
  std::vector<int> head_, prev_, next_;

  std::queue<QueueElement> insert_queue_;
  std::queue<QueueElement> delete_queue_;
  std::queue<QueueElement> update_queue_;
  std::queue<QueueElement> occupancy_queue_;

  Eigen::Vector3d origin_;
  int reserved_idx_4_undefined_;
  int total_time_ = 0;
  int infinity_, undefined_;
  double resolution_, resolution_inv_;
  Eigen::Vector3i max_vec_, min_vec_, last_max_vec_, last_min_vec_;

  void DeleteFromList(int link, int idx);
  void InsertIntoList(int link, int idx);

 public:
#ifdef HASH_TABLE
  ESDFMap(Eigen::Vector3d origin, double resolution, int reserve_size = 0);
#else
  int grid_total_size_;
  ESDFMap(Eigen::Vector3d origin, double resolution, Eigen::Vector3d map_size);
#endif

  ~ESDFMap() {}

#ifdef PROBABILISTIC
  void SetParameters(double p_hit, double p_miss, double p_min, double p_max, double p_occ);
#endif

  bool CheckUpdate();
  bool UpdateOccupancy(bool global_map);
  void UpdateESDF();

  int SetOccupancy(Eigen::Vector3d pos, int occ);
  int SetOccupancy(Eigen::Vector3i vox, int occ);
  int GetOccupancy(Eigen::Vector3d pos);
  int GetOccupancy(Eigen::Vector3i pos_id);

  double GetDistance(Eigen::Vector3d pos);
  double GetDistance(Eigen::Vector3i vox);
  double GetDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d &grad);

  // ROS 2 Modification: Changed sensor_msgs::PointCloud to sensor_msgs::msg::PointCloud2
  void GetPointCloud(sensor_msgs::msg::PointCloud2 &m, int vis_lower_bound, int vis_upper_bound);
  // ROS 2 Modification: Changed visualization_msgs::Marker to visualization_msgs::msg::Marker
  void GetSliceMarker(visualization_msgs::msg::Marker &m, int slice, int id, Eigen::Vector4d color, double max_dist);

  void SetUpdateRange(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos, bool new_vec = true);
  void SetOriginalRange();

#ifndef PROBABILISTIC
  void SetAway();
  void SetAway(Eigen::Vector3i left, Eigen::Vector3i right);
  void SetBack();
  void SetBack(Eigen::Vector3i left, Eigen::Vector3i right);
#endif

#ifdef DEBUG
  bool CheckConsistency();
  bool CheckWithGroundTruth();
#endif
};
}

#endif //ESDF_MAP_H