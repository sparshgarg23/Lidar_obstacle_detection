// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int point_idx, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distancetol) {
	processed[point_idx] = true;
	cluster.push_back(point_idx);
	for (int idx : tree->search(points[point_idx], distancetol)) {
		if (!processed[idx]) {
			proximity(idx, points, cluster, processed, tree, distancetol);
		}
	}
}
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideancluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distancetol) {
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);
	for (int i = 0; i < points.size(); i++) {
		if (!processed[i]) {
			std::vector<int> cluster;
			proximity(i, points, cluster, processed, tree, distancetol);
			clusters.push_back(cluster);
		}
	}
	return clusters;
}
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
	typename pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloud_nearremoved(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
	//Voxel filter
	typename pcl::VoxelGrid<PointT> g;
	g.setInputCloud(cloud);
	g.setLeafSize(filterRes, filterRes, filterRes);
	g.filter(*cloud_downsampled);
	//ROI Filter
	typename pcl::CropBox<PointT> remove_nearneighbor;
	typename pcl::CropBox<PointT> roi;

	remove_nearneighbor.setInputCloud(cloud_downsampled);
	remove_nearneighbor.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
	remove_nearneighbor.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
	remove_nearneighbor.setNegative(true);
	remove_nearneighbor.filter(*cloud_nearremoved);

	roi.setInputCloud(cloud_nearremoved);
	roi.setMin(minPoint);
	roi.setMax(maxPoint);
	roi.filter(*cloud_filtered);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr  ground_cloud(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());

	for (int idx : inliers->indices) {
		ground_cloud->points.push_back(cloud->points[idx]);
	}
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*obstacle_cloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(ground_cloud, obstacle_cloud);

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    //pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxIterations);
	seg.setDistanceThreshold(distanceThreshold);

	//perform segmentation
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coeff);
	if (inliers->indices.size() == 0) {
		std::cout << "Could not estimate planar model." << std::endl;
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int max_iter, float distancetol) {
	std::unordered_set<int> inlier_result;
	srand(time(NULL));
	while (max_iter--) {
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)inliers.insert(rand() % (cloud->points.size()));
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto iter = inliers.begin();
		x1 = cloud->points[*iter].x;
		y1 = cloud->points[*iter].y;
		z1 = cloud->points[*iter].z;
		iter++;
		x2 = cloud->points[*iter].x;
		y2 = cloud->points[*iter].y;
		z2 = cloud->points[*iter].z;
		iter++;
		x3 = cloud->points[*iter].x;
		y3 = cloud->points[*iter].y;
		z3 = cloud->points[*iter].z;

		float a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		float b = (z2 - z1)*(x3 - x1) - (z3 - z1)*(x2 - x1);
		float c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		float d = -(a*x1 + b*y1 + c*z1);

		for (int index = 0; index < cloud->points.size(); index++) {
			if (inliers.count(index) > 0) {
				continue;
			}
			PointT pt = cloud->points[index];
			float x4 = pt.x;
			float y4 = pt.y;
			float z4 = pt.z;
			float dist = fabs(a*x4 + b*y4 + c*z4) / sqrt(a*a + b*b + c*c);
			if (dist <= distancetol)inliers.insert(index);
		}
		if (inliers.size() > inlier_result.size())inlier_result = inliers;
	}
	typename pcl::PointCloud<PointT>::Ptr cloud_inlier(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloud_outlier(new pcl::PointCloud<PointT>());
	for (int index = 0; index < cloud->points.size(); index++) {
		PointT pt = cloud->points[index];
		if (inlier_result.count(index)) {
			cloud_inlier->points.push_back(pt);
		}
		else {
			cloud_outlier->points.push_back(pt);
		}
	}
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segresult(cloud_inlier, cloud_outlier);
	return segresult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<std::vector<float>> points;
	KdTree* tree = new KdTree;
	for (int i = 0; i < cloud->points.size(); i++) {
		auto point = cloud->points[i];
		tree->insert({ point.x,point.y,point.z }, i);
		points.push_back({ point.x,point.y,point.z });
	}


    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	std::vector<std::vector<int>> indices = euclideancluster(points, tree, clusterTolerance);
	for (std::vector<int> v : indices) {
		typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
		if (v.size() >= minSize && v.size() <= maxSize) {
			for (int cluster_idx : v) {
				cluster->points.push_back(cloud->points[cluster_idx]);
			}
			clusters.push_back(cluster);
		}
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}



template<typename PointT>
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


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
