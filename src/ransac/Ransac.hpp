#pragma once 

#include <random>
#include <omp.h>
#include "PlaneModel.hpp"

template<typename PointT>
class Ransac
{
    public:
        Ransac(int maxIterations, float distanceTol)
            :m_maxIterations(maxIterations), m_distanceTol(distanceTol), m_results(maxIterations)
        {
            m_nThreads = std::max(1, omp_get_max_threads());
            omp_set_dynamic(0);
            omp_set_num_threads(m_nThreads);

            for(int i = 0; i < m_nThreads; ++i)
            {
                std::random_device seedDevice;
                m_randomEngines.push_back(std::mt19937(seedDevice()));
            }
        }

        SegmentedCloud<PointT> segment(typename pcl::PointCloud<PointT>::Ptr cloud)
        {
            std::uniform_int_distribution<int> dist(0, cloud->points.size());
            SegmentedCloud<PointT> bestResult{0, nullptr, nullptr};
            #pragma omp parallel for
            for(int i = 0; i < m_maxIterations; i++)
            {
                std::vector<PointT> randSamples;
                while(randSamples.size() < 3)
                {
                    randSamples.push_back(cloud->points[dist(m_randomEngines[omp_get_thread_num()])]);
                }
                auto randomModel = std::make_shared<PlaneModel<PointT>>(randSamples);
                m_results[i] = randomModel->evaluate(cloud, m_distanceTol);
            }
            for(int i = 0; i < m_maxIterations; ++i)
            {
                if(m_results[i].fraction > bestResult.fraction)
                {
                    bestResult = m_results[i];
                }
            }
            return bestResult;
        }

    private:
        int m_nThreads;
        int m_maxIterations;
        float m_distanceTol;

        std::vector<std::mt19937> m_randomEngines;
        std::vector<SegmentedCloud<PointT>> m_results;
        
};