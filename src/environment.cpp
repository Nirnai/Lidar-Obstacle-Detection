#include <iostream>
#include "utils/Profiler.hpp"

#define PROFILING 1
#if PROFILING 
    #define PROFILE_SCOPE(name) ProfilerTimer timer##__LINE__(name);
    #define PROFILE_FUNCTION() PROFILE_SCOPE(__FUNCTION__)
#else
    #define PROFILE_SCOPE(name)
    #define PROFILE_FUNCTION()
#endif

#include "render.h"
#include "PointProcessor.hpp"

int main()
{
    std::cout<<"Starting Environment!"<<std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    initCamera(FPS, viewer);

    PointProcessor<pcl::PointXYZI> pointProcessor;
    auto stream = pointProcessor.streamPCD("../data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Profile One Frame of processing
        Profiler::get().beginSession("Profile");
        // Load pcd and run obstacle detection process
        cloud = pointProcessor.loadPCD((*streamIterator).string());
        // Voxel Filter and Region of Interest
        auto filteredCloud = pointProcessor.filterCloud(cloud, 0.1f, Eigen::Vector4f (-10, -5, -3, 1), Eigen::Vector4f (30, 7, 1, 1) );
        // Segment Plane
        auto segmentedCloud = pointProcessor.segmentCloud(filteredCloud, 100, 0.2);
        renderPointCloud(viewer, segmentedCloud.inlier, "Road", Color(0,1,0));
        auto clusters = pointProcessor.clusterCloud(segmentedCloud.outlier, 0.3, 20, 1500);
        {
            PROFILE_SCOPE("BoundingBox")
            int clusterId = 0;
            std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,1,1),Color(0,0,1), Color(1,0,1)};
            for(auto cluster: clusters)
            {
                std::cout << cluster->size() << std::endl;
                renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%3]);
                Box box = pointProcessor.boundingBox(cluster);
                renderBox(viewer, box, clusterId);
                ++clusterId;
            }
        }
        Profiler::get().endSession();

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }

    
    return 0;
}