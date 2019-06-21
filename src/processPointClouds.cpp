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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr road_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
    for (int index : inliers->indices){
        road_cloud->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(road_cloud, obstacle_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    std::vector<int> res;

    res = RansacPlane(cloud,maxIterations,distanceThreshold);
    inliers->indices = res;
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);
    for(pcl::PointIndices getIndices : clusterIndices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices){
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
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

template<typename PointT>
std::vector<float> ProcessPointClouds<PointT>::crossProd(std::vector<float> const& v1, std::vector<float> const& v2){
    std::vector<float> result (v1.size());
    result = {v1[1]*v2[2] - v1[2]*v2[1],
            v1[2]*v2[0] - v1[0]*v2[2],
            v1[0]*v2[1] - v1[1]*v2[0]};
    return result;
}

template<typename PointT>
std::vector<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol){
    auto startTime = std::chrono::steady_clock::now();
    srand(time(NULL));
    pcl::PointIndices* bestSet(new pcl::PointIndices ());
    pcl::PointIndices* tempSet(new pcl::PointIndices ());
    // TODO: Fill in this function
    pcl::PointXYZ *point1(new pcl::PointXYZ());
    pcl::PointXYZ *point2(new pcl::PointXYZ());
    pcl::PointXYZ *point3(new pcl::PointXYZ());

    float coefficients[4] = {0.0,0.0,0.0,0.0};

    std::vector<float> v1 {0.0, 0.0, 0.0};
    std::vector<float> v2 {0.0, 0.0, 0.0};
    std::vector<float> v3 {0.0, 0.0, 0.0};

    int cloudSize = cloud->points.size();
    // For max iterations
    for (int i=0; i<maxIterations; i++){
    // Randomly sample subset and fit line
        *point1 = cloud->points[rand()%cloudSize];
        //do{
        *point2 = cloud->points[rand()%cloudSize];
        *point3 = cloud->points[rand()%cloudSize];
        //}
        //while(point2->x == point1->x && point2->y == point1->y);
        v1 = {point2->x - point1->x, point2->y - point1->y, point2->z - point1->z};
        v2 = {point3->x - point1->x, point3->y - point1->y, point3->z - point1->z};
        //std::cout << "X1: " << point1.x << "\t Y1: " << point1.y << std::endl;
    
        v3 = crossProd(v1,v2);
        coefficients[0] = v3[0];
        coefficients[1] = v3[1];
        coefficients[2] = v3[2];
        coefficients[3] = -(v3[0]*v1[0] + v3[1]*v1[1] + v3[2]*v1[2]);
        //std::cout << "X: " << v3[0] << "\t Y: " << v3[1] << "\t Z: " << v3[2] << std::endl;

        // Measure distance between every point and fitted line
        for(int j = 0; j<cloudSize; j++){
            if(abs(coefficients[0]*cloud->points[j].x + coefficients[1]*cloud->points[j].y +
            coefficients[2]*cloud->points[j].z + coefficients[3])/sqrt(pow(coefficients[0],2)+pow(coefficients[1],2) + pow(coefficients[2],2))<distanceTol){
                tempSet->indices.push_back(j);
            }
        }
    // If distance is smaller than threshold count it as inlier
        if(tempSet->indices.size()>bestSet->indices.size()){
            bestSet->indices = tempSet->indices;
            //std::cout << "BestSet Dim: " << bestSet->indices.size() << std::endl;
        }
        tempSet->indices.clear();

    }

    // Return indicies of inliers from fitted line with most inliers
    std::vector<int> inliersResult(bestSet->indices.begin(),bestSet->indices.end());
    //std::cout << "Inliers Dim : " << inliersResult.size() << std::endl;
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "ransac took " << elapsedTime.count() << " milliseconds" << std::endl;
    delete point1;
    delete point2;
    delete point3;
    delete bestSet;
    delete tempSet;
    
    return inliersResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, uint index, typename pcl::PointCloud<PointT>::Ptr cluster, std::vector<bool>& procPointIndexes, KdTree* tree, float distanceTol){
	procPointIndexes[index] = true;
	cluster->points.push_back(cloud->points[index]);
	std::vector<int> neighborhood = tree->search({cloud->points[index].x,cloud->points[index].y,cloud->points[index].z},distanceTol);

	for(int n : neighborhood){
		if(!procPointIndexes[n]){
			proximity(cloud, n, cluster, procPointIndexes, tree, distanceTol);
		}
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::customClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, uint minSize, uint maxSize){
    KdTree* tree = new KdTree;
    
    for (int i=0; i<cloud->points.size(); i++){
    	tree->insert({cloud->points[i].x,cloud->points[i].y,cloud->points[i].z},i);
    }

    std::vector<bool> procPointIndexes(cloud->points.size(),false);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    for(int p=0; p<cloud->points.size(); p++){
		if(!procPointIndexes[p])
		{
			typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
			proximity(cloud,p,cluster,procPointIndexes,tree,distanceTol);
            if(cluster->points.size()>minSize && cluster->points.size()<maxSize){
			    clusters.push_back(cluster);
            }
		}
	}

    return clusters;

}