#include <pcl/point_cloud.h>
#include "KDTree.hpp"


template<typename PointT>
class EuclideanCluster
{
    public:
        EuclideanCluster(float distanceTol)
            :m_distanceTol(distanceTol)
        {}

        std::vector<typename pcl::PointCloud<PointT>::Ptr> estimate(typename pcl::PointCloud<PointT>::Ptr cloud)
        {
            std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
            std::vector<bool> processed(cloud->points.size(), false);
            m_tree = std::make_shared<KdTree<PointT>>();
            for(int i = 0; i < cloud->points.size(); ++i)
            {
                m_tree->insert(cloud->points[i], i);
            }

            for(int i = 0; i < cloud->size(); ++i)
            {
                if(processed[i])
                {
                    i++;
                    continue;
                }
                typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
                clusterRecursive(i, cloud, cluster, processed);
                clusters.push_back(cluster);
                i++;
                
            }

            return clusters;
        }


        void clusterRecursive(int i, typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr cluster, std::vector<bool> processed)
        {
            processed[i] = true;
            cluster->push_back(cloud->points[i]);
            std::vector<int> nearest = m_tree->search(cloud->points[i], m_distanceTol);
            for( int idx : nearest )
            {
                if(!processed[idx])
                    clusterRecursive(idx, cloud, cluster, processed);
            }
        }


    private:
        std::shared_ptr<KdTree<PointT>> m_tree;
        float m_distanceTol;
}; 