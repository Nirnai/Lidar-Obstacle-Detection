#ifndef POINTPROCESSOR_H_
#define POINTPROCESSOR_H_

#include <vector>
#include <string>
#include <random>
#include <unordered_set>
#include <set>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

template<typename PointT> using PointCloud = typename pcl::PointCloud<PointT>::Ptr;
template<typename PointT> using PointCloudPair = std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>;


template<typename PointT>
class PointProcessor
{
    public:
        // Processing
        PointCloud<PointT> filterCloud(PointCloud<PointT> inputCloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
        PointCloudPair<PointT> segmentCloud(PointCloud<PointT> inputCloud, int maxIterations, float distanceThreshold);

        // PCD Loading and Streaming
        PointCloud<PointT> loadPCD(std::string file);
        std::vector<boost::filesystem::path> streamPCD(std::string dataPath);

    private:
        std::vector<int> ransac(PointCloud<PointT> cloud, int maxIterations, float distanceTol);
        std::vector<float> planeModel(PointT p1, PointT p2, PointT p3);
        PointCloudPair<PointT> seperateClouds(std::vector<int> inliers, PointCloud<PointT> cloud);
};

#endif