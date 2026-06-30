//
// ESDFServer.h — 轻量封装，供 CBF 控制器直接内嵌使用
// 避免走 ROS 2 topic 通信，建图和查询在同一进程内存中完成
//

#ifndef FIESTA_ESDF_SERVER_H_
#define FIESTA_ESDF_SERVER_H_

#include "ESDFMap.h"
#include "raycast.h"
#include "parameters.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>
#include <vector>

#ifdef HASH_TABLE
#include <unordered_set>
#endif

namespace fiesta {

class ESDFServer {
public:
    ESDFServer(const Parameters &params);
    ~ESDFServer();

    // ========== 建图接口 ==========

    /// 插入世界坐标系点云（不做 raycasting，直接标记为占据）
    void insertPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud);

    /// 插入点云并做 raycasting（从相机原点向每个点发射射线）
    /// @param cloud     相机坐标系下的点云
    /// @param origin    相机在世界坐标系中的位置
    void insertPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                          const Eigen::Vector3d &origin);

    /// 触发 ESDF 增量更新（通常在插入几帧后调用一次）
    void update();

    /// 获取当前地图更新的中心位置
    void setUpdateCenter(const Eigen::Vector3d &center);

    // ========== 查询接口 ==========

    /// 查询世界坐标系 (x,y,z) 处的距离和梯度（三线性插值，最精确）
    /// @return 到最近障碍物的距离，正=外部，负=内部
    double getDistanceAndGradient(const Eigen::Vector3d &pos,
                                  Eigen::Vector3d &grad);

    /// 查询最近距离（不带梯度）
    double getDistance(const Eigen::Vector3d &pos);

    /// 获取体素占据状态：1=占据, 0=自由, -1=未知
    int getOccupancy(const Eigen::Vector3d &pos);

    // ========== 参数接口 ==========

    const Parameters &parameters() const { return params_; }
    double resolution() const { return params_.resolution_; }

private:
    Parameters params_;
    ESDFMap *map_;

    Eigen::Vector3d cur_center_;

    // 上次 ESDF 更新的体素范围
    Eigen::Vector3i last_update_min_, last_update_max_;
    int esdf_update_count_ = 0;

#ifdef PROBABILISTIC
    // 射线投射
    void raycastMultiThread(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                            const Eigen::Vector3d &origin);

    Eigen::Vector3d raycast_origin_;
#ifdef HASH_TABLE
    std::unordered_set<int> set_free_, set_occ_;
#else
    std::vector<int> set_free_, set_occ_;
    int tot_ = 0;
#endif
#endif
};

// ====================== 实现 ======================

inline ESDFServer::ESDFServer(const Parameters &params) : params_(params) {
#ifdef HASH_TABLE
    map_ = new ESDFMap(Eigen::Vector3d(0, 0, 0),
                       params_.resolution_,
                       params_.reserved_size_);
#else
    map_ = new ESDFMap(params_.l_cornor_,
                       params_.resolution_,
                       params_.map_size_);
#endif

#ifdef PROBABILISTIC
    map_->SetParameters(params_.p_hit_, params_.p_miss_,
                        params_.p_min_, params_.p_max_,
                        params_.p_occ_);
#endif

#ifdef PROBABILISTIC
#ifndef HASH_TABLE
    set_free_.resize(map_->grid_total_size_);
    set_occ_.resize(map_->grid_total_size_);
    std::fill(set_free_.begin(), set_free_.end(), 0);
    std::fill(set_occ_.begin(), set_occ_.end(), 0);
#endif
#endif

    cur_center_ = Eigen::Vector3d::Zero();
}

inline ESDFServer::~ESDFServer() {
    delete map_;
}

// ---- 非 raycasting 模式：直接标记占据 ----
inline void ESDFServer::insertPointCloud(
        const pcl::PointCloud<pcl::PointXYZ> &cloud) {
#ifdef PROBABILISTIC
    // 概率模式仍需 raycasting，走重载版本
    insertPointCloud(cloud, Eigen::Vector3d::Zero());
    return;
#else
    map_->SetUpdateRange(cur_center_ - params_.radius_,
                         cur_center_ + params_.radius_, false);
    map_->SetAway();
    Eigen::Vector3d tmp_pos;
    for (const auto &pt : cloud.points) {
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
            continue;
        tmp_pos = Eigen::Vector3d(pt.x, pt.y, pt.z);
        map_->SetOccupancy(tmp_pos, 1);
    }
    map_->SetBack();
#endif
}

// ---- raycasting 模式 ----
inline void ESDFServer::insertPointCloud(
        const pcl::PointCloud<pcl::PointXYZ> &cloud,
        const Eigen::Vector3d &origin) {
#ifndef PROBABILISTIC
    // 非概率模式忽略 origin，直接调简单版本
    insertPointCloud(cloud);
#else
    raycastMultiThread(cloud, origin);
#endif
}

// ---- 触发 ESDF 更新 ----
inline void ESDFServer::update() {
    if (!map_->CheckUpdate())
        return;

    if (params_.global_update_)
        map_->SetOriginalRange();
    else
        map_->SetUpdateRange(cur_center_ - params_.radius_,
                             cur_center_ + params_.radius_);

    map_->UpdateOccupancy(params_.global_update_);
    map_->UpdateESDF();
    esdf_update_count_++;
}

// ---- 设置更新中心 ----
inline void ESDFServer::setUpdateCenter(const Eigen::Vector3d &center) {
    cur_center_ = center;
}

// ---- 距离+梯度查询 ----
inline double ESDFServer::getDistanceAndGradient(
        const Eigen::Vector3d &pos, Eigen::Vector3d &grad) {
    return map_->GetDistWithGradTrilinear(pos, grad);
}

// ---- 距离查询 ----
inline double ESDFServer::getDistance(const Eigen::Vector3d &pos) {
    return map_->GetDistance(pos);
}

// ---- 占据查询 ----
inline int ESDFServer::getOccupancy(const Eigen::Vector3d &pos) {
    return map_->GetOccupancy(pos);
}

// ====================== 概率模式 raycasting ======================
#ifdef PROBABILISTIC

inline void ESDFServer::raycastMultiThread(
        const pcl::PointCloud<pcl::PointXYZ> &cloud,
        const Eigen::Vector3d &origin) {

    raycast_origin_ = origin;
#ifdef HASH_TABLE
    set_free_.clear();
    set_occ_.clear();
#endif
    int tt = ++tot_;

    map_->SetUpdateRange(cur_center_ - params_.radius_,
                         cur_center_ + params_.radius_, false);

    Eigen::Vector3d half(0.5, 0.5, 0.5);
    std::vector<Eigen::Vector3d> ray_output;

    for (size_t idx = 0; idx < cloud.points.size(); ++idx) {
        const auto &pt = cloud.points[idx];
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
            continue;

        Eigen::Vector3d point(pt.x, pt.y, pt.z);

        double length = (point - raycast_origin_).norm();
        if (length < params_.min_ray_length_)
            continue;

        int tmp_idx;
        if (length > params_.max_ray_length_) {
            point = (point - raycast_origin_) / length
                    * params_.max_ray_length_ + raycast_origin_;
            tmp_idx = map_->SetOccupancy(point, 0);
        } else {
            tmp_idx = map_->SetOccupancy(point, 1);
        }

        if (tmp_idx != -10000) {
#ifdef HASH_TABLE
            if (set_occ_.find(tmp_idx) != set_occ_.end())
                continue;
            set_occ_.insert(tmp_idx);
#else
            if (set_occ_[tmp_idx] == tt)
                continue;
            set_occ_[tmp_idx] = tt;
#endif
        }

        // 射线投射：从相机光心到障碍点
        Raycast(raycast_origin_ / params_.resolution_,
                point / params_.resolution_,
                params_.l_cornor_ / params_.resolution_,
                params_.r_cornor_ / params_.resolution_,
                &ray_output);

        // 标记射线路径上的体素为 free
        int hit_cnt = 0;
        for (int i = static_cast<int>(ray_output.size()) - 2; i >= 0; --i) {
            Eigen::Vector3d tmp = (ray_output[i] + half) * params_.resolution_;
            double len = (tmp - raycast_origin_).norm();
            if (len < params_.min_ray_length_)
                break;
            if (len > params_.max_ray_length_)
                continue;

            tmp_idx = map_->SetOccupancy(tmp, 0);
            if (tmp_idx != -10000) {
#ifdef HASH_TABLE
                if (set_free_.find(tmp_idx) != set_free_.end()) {
                    if (++hit_cnt >= 1) break;
                } else {
                    set_free_.insert(tmp_idx);
                    hit_cnt = 0;
                }
#else
                if (set_free_[tmp_idx] == tt) {
                    if (++hit_cnt >= 1) break;
                } else {
                    set_free_[tmp_idx] = tt;
                    hit_cnt = 0;
                }
#endif
            }
        }
        ray_output.clear();
    }
}

#endif // PROBABILISTIC

} // namespace fiesta

#endif // FIESTA_ESDF_SERVER_H_
