#pragma once

#include <vector>
#include <string>
#include <random>
#include <unordered_set>
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

// Create Scoped Timer
// Namespace instead of class
// Struct for Plane Model (return model as struct)
// Convert Pointcloud to Matrix and multiply in one step
// pragma once instead ifndef ...
// Templates functions do not need to specify type if deducable


template<typename PointT> using PointCloud = typename pcl::PointCloud<PointT>::Ptr;
template<typename PointT> using PointCloudPair = std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>;


template<typename PointT>
class PointProcessor
{
    public:
        // Processing
        typename pcl::PointCloud<PointT>::Ptr filterCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
        SegmentedCloud segmentCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud, int maxIterations, float distanceThreshold);
        std::vector<typename pcl::PointCloud<PointT>::Ptr inputCloud> clusterCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud inputCloud, float clusterTolerance, int minSize, int maxSize);

        // PCD Loading and Streaming
        PointCloud<PointT> loadPCD(std::string file);
        std::vector<boost::filesystem::path> streamPCD(std::string dataPath);

    private:
        PointCloudPair<PointT> ransac(PointCloud<PointT> cloud, int maxIterations, float distanceTol);
};


