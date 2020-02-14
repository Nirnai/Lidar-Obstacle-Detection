#include "utils/Profiler.hpp"
#define PROFILING 1
#if PROFILING 
    #define PROFILE_SCOPE(name) ProfilerTimer timer##__LINE__(name);
    #define PROFILE_FUNCTION() PROFILE_SCOPE(__FUNCTION__)
#else
    #define PROFILE_SCOPE(name)
    #define PROFILE_FUNCTION()
#endif

// #include "euclideanCluster/KDTree.hpp"
#include "PointProcessor.hpp"



int main()
{
    std::cout << "Start Test!" << std::endl;
    PointProcessor<pcl::PointXYZI> processor;
    auto cloud = processor.loadPCD("../data/pcd/data_1/0000000000.pcd");
    std::cerr << "Loaded " << cloud->points.size () << " data points" << std::endl;


}