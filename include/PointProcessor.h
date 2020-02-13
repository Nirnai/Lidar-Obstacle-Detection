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
        PointCloud<PointT> filterCloud(PointCloud<PointT> inputCloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
        PointCloudPair<PointT> segmentCloud(PointCloud<PointT> inputCloud, int maxIterations, float distanceThreshold);
        std::vector<PointCloud<PointT>> clusterCloud(PointCloud<PointT> inputCloud, float clusterTolerance, int minSize, int maxSize);

        // PCD Loading and Streaming
        PointCloud<PointT> loadPCD(std::string file);
        std::vector<boost::filesystem::path> streamPCD(std::string dataPath);

    private:
        PointCloudPair<PointT> ransac(PointCloud<PointT> cloud, int maxIterations, float distanceTol);
};


template<typename PointT>
class PlaneModel
{
    public:
        PlaneModel(const std::vector<PointT>& points)
        {
            auto p1 = points[0];
            auto p2 = points[1];
            auto p3 = points[2];

            m_a = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
            m_b = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
            m_c = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
            m_d = - (m_a * p1.x + m_b * p1.y + m_c * p1.z);
            m_n = sqrt( m_a*m_a + m_b*m_b + m_c*m_c );
        }

        std::pair<float, PointCloudPair<PointT>> evaluate(const PointCloud<PointT> cloud, float distanceTol)
        {
            PointCloud<PointT> inliers(new pcl::PointCloud<PointT>);
            PointCloud<PointT> outliers(new pcl::PointCloud<PointT>);
            int nPoints = cloud->points.size();
            int nInliers = 0;
            for( auto p : cloud->points )
            {
                if(computeDistance(p) < distanceTol)
                {
                    inliers->points.push_back(p);
                    nInliers++;
                }else
                {
                    outliers->points.push_back(p);
                }   
            }
            float inlierFraction = (float)nInliers/(float)nPoints;
            PointCloudPair<PointT> result(inliers, outliers);
            return std::make_pair(inlierFraction, result);
        }

    protected:
        float computeDistance( PointT p )
        {
            return fabs(m_a * p.x + m_b * p.y + m_c * p.z + m_d)/m_n;
        }

    private:
        float m_a;
        float m_b;
        float m_c;
        float m_d;
        float m_n;

};