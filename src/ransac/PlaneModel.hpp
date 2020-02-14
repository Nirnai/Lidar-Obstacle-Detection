#pragma once
#include <vector>
#include <cmath>
#include <pcl/point_cloud.h>

template<typename PointT>
struct SegmentedCloud
{   
    // Inlier Fraction
    float fraction;
    // Resulting Clouds
    typename pcl::PointCloud<PointT>::Ptr inlier;
    typename pcl::PointCloud<PointT>::Ptr outlier;
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
            m_n = std::sqrt( m_a*m_a + m_b*m_b + m_c*m_c );
        }

        SegmentedCloud<PointT> evaluate(const typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol)
        {
            typename pcl::PointCloud<PointT>::Ptr inlier(new pcl::PointCloud<PointT>);
            typename pcl::PointCloud<PointT>::Ptr outlier(new pcl::PointCloud<PointT>);
            int nPoints = cloud->points.size();
            int nInliers = 0;
            for( auto p : cloud->points )
            {
                if(computeDistance(p) < distanceTol)
                {
                    inlier->points.push_back(p);
                    nInliers++;
                }else
                {
                    outlier->points.push_back(p);
                }   
            }
            float inlierFraction = (float)nInliers/(float)nPoints;
            return SegmentedCloud<PointT>{inlierFraction, inlier, outlier};
        }

    protected:
        float computeDistance( PointT p )
        {
            return std::fabs(m_a * p.x + m_b * p.y + m_c * p.z + m_d)/m_n;
        }

    private:
        float m_a;
        float m_b;
        float m_c;
        float m_d;
        float m_n;

};