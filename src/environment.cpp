#include <iostream>
#include "PointProcessor.hpp"
#include "render.h"

int main() {
  std::cout << "Starting Environment!" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer);
  initCamera(XY, viewer);

  PointProcessor<pcl::PointXYZI> pointProcessor;
  auto stream = pointProcessor.streamPCD("data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

  while (!viewer->wasStopped()) {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // LOAD
    cloud = pointProcessor.loadPCD((*streamIterator).string());
    // END LOAD

    // VOXEL FILTER
    auto filteredCloud =
        pointProcessor.filterCloud(cloud, 0.1f, Eigen::Vector4f(-10, -5, -3, 1),
                                   Eigen::Vector4f(30, 7, 1, 1));
    // END VOXEL FILTER

    // SEGMENT PLANE
    auto segmentedCloud = pointProcessor.segmentCloud(filteredCloud, 100, 0.2);
    renderPointCloud(viewer, segmentedCloud.inlier, "Road", Color(0, 1, 0));
    // END SEGMENT PLANE

    // CLUSTERING
    // auto clusters =
    //     pointProcessor.clusterCloud(segmentedCloud.outlier, 0.3, 30, 1500);
    // segmentedCloud.outlier->width = 1;
    // segmentedCloud.outlier->height = segmentedCloud.outlier->points.size();

    // pcl::io::savePCDFileASCII("../data/test_pcd.pcd",
    // *segmentedCloud.outlier); std::cerr << "Saved " <<
    // segmentedCloud.outlier->points.size()
    //           << " data points to test_pcd.pcd." << std::endl;

    auto clusters =
        pointProcessor.clusterCloud(segmentedCloud.outlier, 0.5, 10);
    std::cout << "Number of Clusters: " << clusters.size() << std::endl;
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 1, 1),
                                 Color(0, 0, 1), Color(1, 0, 1)};
    for (auto cluster : clusters) {
      renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                       colors[clusterId % 3]);
      Box box = pointProcessor.boundingBox(cluster);
      renderBox(viewer, box, clusterId);
      ++clusterId;
    }
    // ENF CLUSTERING

    // streamIterator++;
    if (streamIterator == stream.end()) streamIterator = stream.begin();

    viewer->spinOnce();
  }

  return 0;
}