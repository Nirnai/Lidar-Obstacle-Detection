#include "PointProcessor.h"


template<typename PointT>
PointCloud<PointT> PointProcessor<PointT>::filterCloud(PointCloud<PointT> inputCloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
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
std::vector<float> PointProcessor<PointT>::planeModel(PointT p1, PointT p2, PointT p3)
{
    float a1 = p2.x - p1.x;
    float b1 = p2.y - p1.y;
    float c1 = p2.z - p1.z;

    float a2 = p3.x - p1.x;
    float b2 = p3.y - p1.y;
    float c2 = p3.z - p1.z;

    float a = b1 * c2 - b2 * c1; 
    float b = a2 * c1 - a1 * c2;
    float c = a1 * b2 - b1 * a2;
    float d = (- a * p1.x - b * p1.y - c * p1.z);

    std::vector<float> model{a,b,c,d};
    return model;
}

template<typename PointT>
std::vector<int> PointProcessor<PointT>::ransac(PointCloud<PointT> cloud, int maxIterations, float distanceTol)
{
    std::vector<int> inliersResult;
    std::random_device rd;
    std::mt19937 gen(rd());

    while (maxIterations--)
    {
        
        std::unordered_set<int> modelPoints;


        // Efficient Random Sampling: https://stackoverflow.com/a/28287865
        for(int r = cloud->points.size() - 3; r < cloud->points.size(); ++r)
        {
            int v = std::uniform_int_distribution<>(1, r)(gen);
            if(!modelPoints.insert(v).second)
            {
                modelPoints.insert(r);
            }
        }
        
        std::vector<int> inliers(modelPoints.begin(), modelPoints.end());

        // Compute Plane Model
        auto it = inliers.begin();
        auto model = planeModel(cloud->points[*it], cloud->points[*std::next(it, 1)], cloud->points[*std::next(it, 2)]);
        float a, b, c, d;
        a = model[0];
        b = model[1];
        c = model[2];
        d = model[3];
        float n = sqrt(a*a + b*b + c*c);

        // Measure distance between every point and fitted plane
        for( int idx = 0; idx < cloud->points.size(); idx++)
		{
			if(modelPoints.count(idx) > 0)
				continue;
			float distance = fabs(a * cloud->points[idx].x + b * cloud->points[idx].y + c * cloud->points[idx].z + d)/n;
			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol)
			{
				inliers.push_back(idx);
			}
		}
        // Return indicies of inliers from fitted line with most inliers
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
    }
    return inliersResult;   
}

template<typename PointT>
PointCloudPair<PointT> PointProcessor<PointT>::seperateClouds(std::vector<int> inliers, PointCloud<PointT> cloud)
{
    PointCloud<PointT> cloudInliers(new pcl::PointCloud<PointT>());
	PointCloud<PointT> cloudOutliers(new pcl::PointCloud<PointT>());

    std::sort(inliers.begin(), inliers.end());
    auto it = inliers.begin();
    for(int idx = 0; idx < cloud->points.size(); idx++)
    {
        PointT point = cloud->points[idx];
        if( idx < *it )
        {
            cloudOutliers->points.push_back(point);
        } else 
        {
            cloudInliers->points.push_back(point);
            it++;
        }
    } 
    PointCloudPair<PointT> cloudPair(cloudInliers, cloudOutliers);
    return cloudPair;
}

template<typename PointT>
PointCloudPair<PointT> PointProcessor<PointT>::segmentCloud(PointCloud<PointT> inputCloud, int maxIterations, float distanceThreshold)
{
    pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(inputCloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud {new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr obstCloud {new pcl::PointCloud<PointT>};
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(inputCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*planeCloud);
    extract.setNegative(true);
    extract.filter(*obstCloud);
    PointCloudPair<PointT> segResult(planeCloud, obstCloud);

    // std::vector<int> inliers = ransac(inputCloud, maxIterations, distanceThreshold);
    // auto segResult = seperateClouds(inliers, inputCloud);

    return segResult;
}

template<typename PointT>
PointCloud<PointT> PointProcessor<PointT>::loadPCD(std::string file)
{
    PointCloud<PointT> cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> PointProcessor<PointT>::streamPCD(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});
    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());
    return paths;
}