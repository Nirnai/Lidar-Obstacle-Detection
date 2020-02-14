#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include "utils/Profiler.hpp"
#include "ransac/Ransac.hpp"
#include "euclideanCluster/EuclideanCluster.hpp"

template<typename PointT>
class PointProcessor
{
    public:
        // PCD Loading and Streaming
        typename pcl::PointCloud<PointT>::Ptr loadPCD(std::string file)
        {
            PROFILE_FUNCTION();
            typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file \n");
            }
            return cloud;
        }

        std::vector<boost::filesystem::path> streamPCD(std::string dataPath)
        {
            std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});
            sort(paths.begin(), paths.end());
            return paths;
        }

        // Processing
        typename pcl::PointCloud<PointT>::Ptr filterCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
        {
            PROFILE_FUNCTION();
            // voxel grid point reduction and region based filtering
            typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
            pcl::VoxelGrid<PointT> filter;
            filter.setInputCloud(inputCloud);
            filter.setLeafSize(filterRes, filterRes, filterRes);
            filter.filter(*cloudFiltered);

            typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
            pcl::CropBox<PointT> region(true);
            region.setMin(minPoint);
            region.setMax(maxPoint);
            region.setInputCloud(cloudFiltered);
            region.filter(*cloudRegion);

            // Roof Points
            std::vector<int> indices;
            pcl::CropBox<PointT> roof(true);
            region.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
            region.setMax(Eigen::Vector4f(2.6,1.7,-.4,1));
            region.setInputCloud(cloudRegion);
            region.filter(indices);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            for(int point: indices)
                inliers->indices.push_back(point);
            
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloudRegion);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*cloudRegion);

            return cloudRegion;
        }

        SegmentedCloud<PointT> segmentCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud, int maxIterations, float distanceThreshold)
        {
            PROFILE_FUNCTION();
            static Ransac<PointT> segmenter(maxIterations, distanceThreshold);
            return segmenter.segment(inputCloud);
        }
        
        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud, float clusterTolerance)
        {
            PROFILE_FUNCTION();
            EuclideanCluster<PointT> clusterer(clusterTolerance);
            return clusterer.estimate(inputCloud);
        }
};