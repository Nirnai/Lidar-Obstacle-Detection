#include <iostream>
#include "render.h"
#include "PointProcessor.h"
#include "PointProcessor.cpp"

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


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

        // Load pcd and run obstacle detection process
        cloud = pointProcessor.loadPCD((*streamIterator).string());
        // cloud = pointProcessor.loadPCD("../data/pcd/data_1/0000000000.pcd");
        // Voxel Filter and Region of Interest
        cloud = pointProcessor.filterCloud(cloud, 0.1f, Eigen::Vector4f (-10, -5, -3, 1), Eigen::Vector4f (30, 7, 1, 1) );
        // Segment Plane
        auto segmented = pointProcessor.segmentCloud(cloud, 100, 0.2);
        renderPointCloud(viewer, segmented.first, "Road", Color(0,1,0));
        renderPointCloud(viewer, segmented.second, "Rest", Color(1,0,0));
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }

    
    return 0;
}