/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

template<class T, class U> std::vector<T> crossProd(std::vector<T> const& v1, std::vector<U> const& v2){
	std::vector<T> result (v1.size());
	result = {v1[1]*v2[2] - v1[2]*v2[1],
			  v1[2]*v2[0] - v1[0]*v2[2],
			  v1[0]*v2[1] - v1[1]*v2[0]};
	return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	
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
	std::unordered_set<int> inliersResult(bestSet->indices.begin(),bestSet->indices.end());
	//std::cout << "Inliers Dim : " << inliersResult.size() << std::endl;
	delete point1;
	delete point2;
	delete point3;
	delete bestSet;
	delete tempSet;
	
	return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
