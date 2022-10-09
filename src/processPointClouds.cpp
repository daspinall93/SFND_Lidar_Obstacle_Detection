// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Apply voxel grid
    typename pcl::PointCloud<PointT>::Ptr voxelFilteredCloud (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.setInputCloud(cloud);
    vg.filter(*voxelFilteredCloud);

    // Apply region of interest
    typename pcl::PointCloud<PointT>::Ptr cropBoxFilteredCloud (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cropBox(true);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(voxelFilteredCloud);
    cropBox.filter(*cropBoxFilteredCloud);

    // Remove lidar returns from top of car
    // std::vector<int> indices;
    // pcl::CropBox<PointT> roofCropBox(true);
    // // roofCropBox.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    // // roofCropBox.setMax(Eigen::Vector4f(2.6, 1.7, -4, 1));
    // roofCropBox.setMin(Eigen::Vector4f (-3.0, -3.0, -2, 1));
    // roofCropBox.setMax(Eigen::Vector4f(4.0, 4.0, 4, 1));
    // roofCropBox.setInputCloud(cropBoxFilteredCloud);
    // roofCropBox.filter(indices);

    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // for (int point : indices)
    //     inliers->indices.push_back(point);

    // pcl::ExtractIndices<PointT> extract;
    // extract.setInputCloud(cropBoxFilteredCloud);
    // extract.setIndices(inliers);
    // extract.setNegative(true);
    // extract.filter(*cropBoxFilteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cropBoxFilteredCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obsCloud(new pcl::PointCloud<PointT>());

    // Extract obstacle
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*planeCloud);

    // Extract obstacles
    extract.setNegative(true);
    extract.filter(*obsCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsCloud, planeCloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Use ransac to determine plane
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    const auto points = Ransac(cloud, maxIterations, distanceThreshold);
    inliers->indices.reserve(points.size());
    inliers->indices.insert(inliers->indices.begin(), points.begin(), points.end());

    // Segment the largest planar component from the remaining cloud
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Create KdTree and insert points from cloud
    KdTree3<PointT> tree;
    for (int i = 0; i < cloud->points.size(); i++) 
    	tree.insert(cloud->points.at(i), i); 
    
    // Carry out euclidean clustering to determine obstacles
    const auto clusterPoints = euclideanCluster(cloud, tree, clusterTolerance);
    std::vector<pcl::PointIndices> clusterIndices(clusterPoints.size());
    for (const auto& cluster : clusterPoints)
    {
        const auto clusterSize = cluster.size();
        if (clusterSize < minSize || clusterSize > maxSize)
            continue;

        pcl::PointIndices indices;
        indices.indices = cluster;
        clusterIndices.emplace_back(indices);
    }

    // Convert cluster indexes to point clouds
    for (const auto &cluster : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for (const auto &idx : cluster.indices)
        {
            cloudCluster->push_back((*cloud)[idx]);
        }
        cloudCluster->width = cloudCluster->size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
	srand(time(NULL));

	for (int i = 0; i < maxIterations; i++)
	{		
		// Select to random points
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
			inliers.insert(rand() % cloud->points.size());

		// Calculate the line 
		auto itr = inliers.begin();
		const auto x1 = cloud->points.at(*itr).x;
		const auto y1 = cloud->points.at(*itr).y;
		const auto z1 = cloud->points.at(*itr).z;
		itr++;
		const auto x2 = cloud->points.at(*itr).x;
		const auto y2 = cloud->points.at(*itr).y;
		const auto z2 = cloud->points.at(*itr).z;
		itr++;
		const auto x3 = cloud->points.at(*itr).x;
		const auto y3 = cloud->points.at(*itr).y;
		const auto z3 = cloud->points.at(*itr).z;

		const auto A = (y2-y1) * (z3-z1) - (z2-z1) * (y3-y1);
		const auto B = (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1);
		const auto C = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1);
		const auto D = -(A * x1 + B * y1 + C * z1);
		const auto E = sqrt(A * A + B * B + C * C);		

		// Find inliers and outliers
		for (int idx = 0; idx < cloud->points.size(); idx++)
		{
			const auto point = cloud->points.at(idx);

			// Determine inliers
			const auto distance = fabs(A * point.x + B * point.y + C * point.z + D) / E;
			if (distance < distanceTol)
				inliers.insert(idx);
		}

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;

	}

	return inliersResult;
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, const KdTree3<PointT>& tree, float distanceTol)
{
	// Cluster all points which haven't been processed yet
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processedPoints(cloud->points.size(), false);
	for (int iPoint = 0; iPoint < cloud->points.size(); iPoint++)
	{
		if (!processedPoints[iPoint])
		{
			std::vector<int> cluster;
			clusteringHelper(iPoint, cloud, tree, distanceTol, processedPoints, cluster);
			clusters.push_back(cluster);
		}		
	}
 
	return clusters;
}

template <typename PointT>
void ProcessPointClouds<PointT>::clusteringHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, const KdTree3<PointT>& tree, float distanceTol, std::vector<bool>& processedPoints, std::vector<int>& cluster)
{
    cluster.push_back(idx);
	processedPoints[idx] = true;

	const auto pointsInClusterIdx = tree.search(cloud->points[idx], distanceTol);

	// Iterate through nearest points and add to cluster if not yet processed
	for (const auto& pointIdx : pointsInClusterIdx)
	{
		if (!processedPoints[pointIdx])
			clusteringHelper(pointIdx, cloud, tree, distanceTol, processedPoints, cluster);
	}
}