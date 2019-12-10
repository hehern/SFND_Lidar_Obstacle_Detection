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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr range_cloud (new typename pcl::PointCloud<PointT>);
    // get the interested region
    pcl::CropBox<PointT> crop_range;
    crop_range.setInputCloud(cloud);
    crop_range.setMin(minPoint);
    crop_range.setMax(maxPoint);
    crop_range.setNegative(false);//false为默认值表示切掉矩形外面的,true表示切掉矩形里面的
    crop_range.filter(*range_cloud);
    typename pcl::PointCloud<PointT>::Ptr _vehicl_cloud (new typename pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> crop_vehicle;
    crop_vehicle.setInputCloud(range_cloud);
    crop_vehicle.setMin(Eigen::Vector4f(-2.0,-1.6,-10,1));
    crop_vehicle.setMax(Eigen::Vector4f(3,1.6,10,1));
    crop_vehicle.setNegative(true);//false为默认值表示切掉矩形外面的,true表示切掉矩形里面的
    crop_vehicle.filter(*_vehicl_cloud);
    // Create the filtering object
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud (new typename pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (_vehicl_cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*filtered_cloud);



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filtered_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new typename pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new typename pcl::PointCloud<PointT>);
    // Create the filtering object
    pcl::ExtractIndices<VPoint> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planeCloud);
    // Extract the obstacles
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // TODO:: Fill in this function to find inliers for the cloud.
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if(inliers->indices.size() == 0){
        std::cout << "Could not estimate a plannar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // std::unordered_set<int> inliersResult;
    pcl::PointIndices::Ptr inliersResult (new pcl::PointIndices);
	
	while(maxIterations--)
	{
		// std::unordered_set<int> inliers;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		while(inliers->indices.size() < 3)
		{
			inliers->indices.push_back(rand()%(cloud->points.size()));
		}
		float x1,y1,z1,x2,y2,z2,x3,y3,z3;
		auto itr = inliers->indices.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr ++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr ++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float A,B,C,D;
		A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
        C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		D = -(A*x1+B*y1+C*z1);
        
		for(int i = 0; i < cloud->points.size(); i++){
			if((i == inliers->indices[0]) || (i == inliers->indices[1]) || (i == inliers->indices[2])){
				continue;
			}
			float x4 = cloud->points[i].x;
			float y4 = cloud->points[i].y;
			float z4 = cloud->points[i].z;
			float distance = fabs(A*x4 + B*y4 + C*z4 + D) / sqrt(A*A + B*B + C*C);
			if(distance <= distanceThreshold) inliers->indices.push_back(i);
		}
		if(inliers->indices.size() > inliersResult->indices.size()) inliersResult = inliers;
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<VPoint>::Ptr tree (new pcl::search::KdTree<VPoint>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<VPoint> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);


    for(int i = 0; i < cluster_indices.size(); i++){
        pcl::ExtractIndices<VPoint> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (pcl::PointIndices::Ptr(&cluster_indices[i], [](pcl::PointIndices *){}));//ptr
        extract.setNegative (false);
        pcl::PointCloud<VPoint>::Ptr cluster(new pcl::PointCloud<VPoint>);
        extract.filter (*cluster);//object
        clusters.push_back(cluster);
    }

    /* for(pcl::PointIndices getIndices: cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(int index: getIndices.indices)
            cluster->points.push_back(cloud->points[index]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }*/
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelp(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);
	std::vector<int> nearest = tree->search(cloud->points[indice], distanceTol);
	for(int id : nearest)
	{
		if(!processed[id])
			clusterHelp(id, cloud, cluster, processed, tree, distanceTol);
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    KdTree* tree = new KdTree;
    for (int i=0; i<cloud->points.size(); i++)
    {
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree->insert(point,i); 
    }
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<int>> clusters_indices;
	std::vector<bool> processed(cloud->points.size(),false);

	int i = 0;
	while(i < cloud->points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}
		std::vector<int> cluster_indice;
		clusterHelp(i, cloud, cluster_indice, processed, tree, clusterTolerance);
		clusters_indices.push_back(cluster_indice);
		i++;
	}
    for(int i = 0; i < clusters_indices.size(); i++){
        if((clusters_indices[i].size() >= minSize) && (clusters_indices[i].size() <= maxSize)){
        pcl::PointCloud<VPoint>::Ptr cluster(new pcl::PointCloud<VPoint>);
        for(int index: clusters_indices[i])
            cluster->points.push_back(cloud->points[index]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
        }
    }
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