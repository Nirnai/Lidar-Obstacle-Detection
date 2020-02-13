#include "Profiler.hpp"
#include "KDTree.hpp"
#include "PointProcessor.h"

#include <omp.h>



template<typename PointT>
PointCloud<PointT> PointProcessor<PointT>::filterCloud(PointCloud<PointT> inputCloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    PROFILE_FUNCTION();
    // voxel grid point reduction and region based filtering
    PointCloud<PointT> cloudFiltered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(inputCloud);
    filter.setLeafSize(filterRes, filterRes, filterRes);
    filter.filter(*cloudFiltered);

    PointCloud<PointT> cloudRegion(new pcl::PointCloud<PointT>);
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

template<typename PointT>
PointCloudPair<PointT> PointProcessor<PointT>::ransac(PointCloud<PointT> cloud, int maxIterations, float distanceTol)
{
    // OpenMP
    int nThreads = std::max(1, omp_get_max_threads());
    omp_set_dynamic(0);
    omp_set_num_threads(nThreads);

    // Create random engine for each thread
    std::vector<std::mt19937> randomEngines;
    // std::cout << "[ INFO ]: Maximum usable threads: " << nThreads << std::endl;
    for(int i = 0; i < nThreads; ++i)
    {
        std::random_device seedDevice;
        randomEngines.push_back(std::mt19937(seedDevice()));
    }

    // Init Containers
    PointCloudPair<PointT> bestPair;
    std::vector<PointCloudPair<PointT>> inOutPair(maxIterations);
    std::vector<std::shared_ptr<PlaneModel<PointT>>> sampledModels(maxIterations);
    std::vector<float> inlierFractionAccum(maxIterations);
    std::shared_ptr<PlaneModel<PointT>> bestModel;
    float bestModelScore=0;
    std::uniform_int_distribution<int> dist(0, cloud->points.size()-1);

    #pragma omp parallel for
    for(int i = 0; i < maxIterations; ++i)
    {
        std::vector<PointT> randSamples;
        while(randSamples.size() < 3)
        {
            randSamples.push_back(cloud->points[dist(randomEngines[omp_get_thread_num()])]);
        }

        std::shared_ptr<PlaneModel<PointT>> randomModel = std::make_shared<PlaneModel<PointT>>(randSamples);
        auto result = randomModel->evaluate(cloud, distanceTol);
        inlierFractionAccum[i] = result.first;
        inOutPair[i] = result.second;
        sampledModels[i] = randomModel;
    }
    for(int i = 0; i < maxIterations; ++i)
    {
        if(inlierFractionAccum[i] > bestModelScore)
        {
            bestModelScore = inlierFractionAccum[i];
            bestModel = sampledModels[i];
            bestPair = inOutPair[i];
        }
    }
    return bestPair;
}

template<typename PointT>
PointCloudPair<PointT> PointProcessor<PointT>::segmentCloud(PointCloud<PointT> inputCloud, int maxIterations, float distanceThreshold)
{
    PROFILE_FUNCTION();
    return ransac(inputCloud, maxIterations, distanceThreshold);
}

template<typename PointT>
std::vector<PointCloud<PointT>> PointProcessor<PointT>::clusterCloud(PointCloud<PointT> inputCloud, float clusterTolerance, int minSize, int maxSize)
{
    std::shared_ptr<KdTree> tree = std::make_shared<KdTree>();
    for (int i=0; i<inputCloud->points.size(); i++) 
    	tree->insert(inputCloud->points[i],i); 

}



template<typename PointT>
PointCloud<PointT> PointProcessor<PointT>::loadPCD(std::string file)
{
    PROFILE_FUNCTION();
    PointCloud<PointT> cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> PointProcessor<PointT>::streamPCD(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});
    sort(paths.begin(), paths.end());
    return paths;
}