#pragma once

#include <pcl/common/distances.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>

template <typename PointT>
class HDBSCAN {
 public:
  std::vector<typename pcl::PointCloud<PointT>::Ptr> cluster(
      typename pcl::PointCloud<PointT>::ConstPtr cloud) {
    // Build Tree
    octree_ = std::make_shared<pcl::octree::OctreePointCloudSearch<PointT>>(
        octree_resolution_);
    octree_->setInputCloud(cloud);
    octree_->addPointsFromInputCloud();

    std::vector<float> core_distance(cloud->points.size());
    Eigen::MatrixXd pairwise_distances(cloud->points.size());

#pragma omp parallel for
    for (int i = 0; i < cloud->points.size(); i++) {
      for (int j = i; j < cloud->points.size(); j++) {
        float distance =
            pcl::euclideanDistance(cloud->points[i], cloud->points[j]);

        std::vector<int> indices;
        std::vector<float> distances;
        octree_->nearestKSearch(i, k_, indices, distances);
        float core_distance_i = distances.back();
        octree_->nearestKSearch(j, k_, indices, distances);
        float core_distance_j = distances.back();
        // float mutual_reachability_distance = ;
      }
    }

    // Output
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    return clusters;
  }

 private:
  std::shared_ptr<pcl::octree::OctreePointCloudSearch<PointT>> octree_ =
      nullptr;
  float octree_resolution_ = 0.5f;
  int k_ = 10;
};