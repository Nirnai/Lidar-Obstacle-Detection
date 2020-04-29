#pragma once

#include <pcl/point_cloud.h>
#include <unordered_set>
#include "KDTree.hpp"

struct Label {
  /// Use an enum class for additional type safety
  enum class PointType { UNDEFINED = 0, NOISE, CLUSTERED };
  PointType point_type_ = PointType::UNDEFINED;
  size_t cluster_id_ = 0;
};

template <typename PointT>
class DensityCluster {
 public:
  DensityCluster(float epsilon, int min_points)
      : epsilon_(epsilon), min_points_(min_points) {}

  std::vector<typename pcl::PointCloud<PointT>::Ptr> cluster(
      typename pcl::PointCloud<PointT>::ConstPtr cloud) {
    // Build Tree
    kdtree_ = std::make_shared<KdTree<PointT>>();
    {
      for (int i = 0; i < cloud->size(); ++i) {
        kdtree_->insert(cloud->points[i], i);
      }
    }

    // Output
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Actual Algorithm
    int cluster_id = 0;
    std::vector<Label> labels(cloud->size());

    for (int i = 0; i < cloud->size(); i++) {
      if (labels[i].point_type_ != Label::PointType::UNDEFINED) continue;
      auto neighbours = kdtree_->search(cloud->points[i], epsilon_);
      std::vector<int> cluster_set = neighbours;
      if (neighbours.size() < min_points_) {
        labels[i].point_type_ = Label::PointType::NOISE;
      } else {
        cluster_id++;
        typename pcl::PointCloud<PointT>::Ptr cluster(
            new pcl::PointCloud<PointT>);
        cluster->push_back(cloud->points[i]);
        labels[i].point_type_ = Label::PointType::CLUSTERED;
        labels[i].cluster_id_ = cluster_id;
        expandCluster(cloud, labels, cluster, cluster_set, cluster_id);
        clusters.push_back(cluster);
      }
    }

    return clusters;
  }

  void expandCluster(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                     std::vector<Label>& labels,
                     typename pcl::PointCloud<PointT>::Ptr cluster,
                     std::vector<int>& cluster_set, int cluster_id) {
    while (cluster_set.size() > 0) {
      int n = cluster_set.front();
      if (labels[n].point_type_ == Label::PointType::NOISE) {
        labels[n].point_type_ = Label::PointType::CLUSTERED;
        labels[n].cluster_id_ = cluster_id;
        cluster->push_back(cloud->points[n]);
      }
      if (labels[n].point_type_ != Label::PointType::UNDEFINED) 
      {
        cluster_set.erase(cluster_set.begin());
        continue;
      }
      labels[n].point_type_ = Label::PointType::CLUSTERED;
      labels[n].cluster_id_ = cluster_id;
      cluster->push_back(cloud->points[n]);
      auto new_neighbours = kdtree_->search(cloud->points[n], epsilon_);
      if (new_neighbours.size() > min_points_) {
        for (auto new_n : new_neighbours) {
          if (labels[new_n].point_type_ == Label::PointType::UNDEFINED) {
            cluster_set.push_back(new_n);
          }
        }
      }
      cluster_set.erase(cluster_set.begin());
    }
  }

 private:
  /* data */
  std::shared_ptr<KdTree<PointT>> kdtree_;
  float epsilon_;
  int min_points_;
};
