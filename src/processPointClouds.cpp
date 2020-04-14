// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    //std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr voxel_filteredCloud(new pcl::PointCloud<PointT>);
    // create filter - set parameters and filter to output cloud
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*voxel_filteredCloud);

    // box crop filter
    pcl::CropBox<PointT> cropBox;
    typename pcl::PointCloud<PointT>::Ptr box_filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
    cropBox.setMax(maxPoint);
    cropBox.setMin(minPoint);
    cropBox.setInputCloud(voxel_filteredCloud);
    cropBox.filter(*box_filteredCloud);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    typename pcl::PointCloud<PointT>::Ptr roof_out(new pcl::PointCloud<pcl::PointXYZI>);
    roof.setMin(Eigen::Vector4f (-1.5,-2,-2,1));
    roof.setMax(Eigen::Vector4f (2.6,2,0.,1));
    roof.setInputCloud(box_filteredCloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point: indices)
        inliers->indices.push_back(point);


    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(box_filteredCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*box_filteredCloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    // return voxel_filteredCloud;

    return box_filteredCloud;
}

// input: cloud of all data and list of indecies of the plane in that cloud
// output: 2 clouds of data, one for plane and other for obstacles
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>());

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the the obstacles points - we could you this to extract also plane points
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);
    // extracting plane using extract object
    extract.setNegative(false);
    extract.filter(*plane);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, plane);
    return segResult;
}
template <typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    uint cloud_size = cloud->points.size();

    // For max iterations
    while (maxIterations--)
    {
        std::unordered_set<int> inliers;
        // Randomly sample subset and fit thw plane
        while (inliers.size() < 3)
        {
            inliers.insert(rand() % (cloud_size));
        }
        float x[3], y[3], z[3];
        auto itr = inliers.begin();
        for (auto i = 0; i < 3; i++)
        {
            x[i] = cloud->points[*itr].x;
            y[i] = cloud->points[*itr].y;
            z[i] = cloud->points[*itr].z;
            itr++;
        }
        float a, b, c, d, delta;

        a = (y[1] - y[0]) * (z[2] - z[0]) - (z[1] - z[0]) * (y[2] - y[1]);
        b = (z[1] - z[0]) * (x[2] - x[0]) - (x[1] - x[0]) * (z[2] - z[1]);
        c = (x[1] - x[0]) * (y[2] - y[0]) - (y[1] - y[0]) * (x[2] - x[1]);
        d = -1. * (a * x[0] + b * y[0] + c * z[0]);

        delta = sqrt(a * a + b * b + c * c);

        for (int index = 0; index < cloud_size; index++)
        {
            if (inliers.count(index) > 0)
                continue;

            PointT point = cloud->points[index];
            float xx = cloud->points[index].x;
            float yy = cloud->points[index].y;
            float zz = cloud->points[index].z;
            float dd = fabs(a * xx + b * yy + c * zz + d);

            if (dd <= distanceTol * delta)
            {
                inliers.insert(index);
            }
            if (inliers.size() > inliersResult.size())
            {
                inliersResult = inliers;
            }
        }
    }

    return inliersResult;
}
// input: cloud of all points
// output: pair of clouds where one belongs to obstacles and another to the plane
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    //auto startTime = std::chrono::steady_clock::now();
    // inliers - indecies of the plane
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

    std::unordered_set<int> inliersRes = Ransac<PointT>(cloud, maxIterations, distanceThreshold);
    
    for(auto itr = inliersRes.begin();itr!= inliersRes.end();itr++)
        inliers->indices.push_back(*itr);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                          float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    // return
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = My_Clustering( cloud, clusterTolerance, minSize, maxSize);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

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
void ProcessPointClouds<PointT>::proximity(const std::vector<PointT, Eigen::aligned_allocator<PointT>>& points, std::vector<int>& cluster,std::vector<bool>& processed,
int current_id, float distanceTol,	 KdTree* tree)
{
	processed[current_id] = true;
	cluster.push_back(current_id);
	std::vector<int> neighbors = tree->search(std::vector<float>{points[current_id].x,points[current_id].y,points[current_id].z},distanceTol);

	for(auto neighbor_id : neighbors)
	{
		if(!processed[neighbor_id])
		{
			proximity(points,cluster, processed,neighbor_id,distanceTol,tree);
		}
	}
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<PointT, Eigen::aligned_allocator<PointT>>& points, KdTree* tree, float distanceTol)
{

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);
	int i = 0;
	while( i < points.size() )
	{
		if(processed[i] )
			{
				i++;
				continue;
			}

		std::vector<int> cluster;
		proximity(points,cluster, processed,i ,distanceTol,tree);
		clusters.push_back(cluster);
		i++ ;
	}

 
	return clusters;

}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::My_Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                          float clusterTolerance, int minSize, int maxSize)
{
	KdTree* tree = new KdTree;
  
    for (int i=0; i<cloud->points.size(); i++) 
    	tree->insert(std::vector<float>{cloud->points[i].x,cloud->points[i].y,cloud->points[i].z},i); 
  	auto startTime = std::chrono::steady_clock::now();
  	//
    typename std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  	std::vector<std::vector<int>> clusters_indicies = euclideanCluster(cloud->points, tree, 3.0);
  	//
    for(std::vector<int> cluster_indices : clusters_indicies)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
  		for(int index: cluster_indices)
  			clusterCloud->points.push_back(PointT(cloud->points[index]));
        clusterCloud->is_dense = true;
        clusters.push_back(clusterCloud);
  	}
   return clusters;
}
