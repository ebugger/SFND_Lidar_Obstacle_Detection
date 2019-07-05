/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
	auto startTime = std::chrono::steady_clock::now();
	int find_loops = 0;
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	while (maxIterations--){
		//hash set with no order, with unique element
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) {
			//rand() will generate very large number and we got a MOD operator, so the result will be in 0 and the points size.
			inliers.insert(rand() % (cloud->points.size()));
		}

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;	
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;	
		z3 = cloud->points[*itr].z;		 

		float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		float d = -(a * x1 + b * y1 + c * z1);


		for (int i=0;i<cloud->points.size();i++) {
			//count on a set to check if the element is in the set, not zero means it contains
			if(inliers.count(i) > 0)
				//do nothing
				continue;
			
			pcl::PointXYZ point = cloud->points[i];
			float x0 = point.x;
			float y0 = point.y;
			float z0 = point.z;
			//fabs and sqrt
			float dist = fabs(a * x0 + b * y0 + c * z0 + d) / sqrt(a * a + b * b + c * c);
			if(dist <= distanceTol)
				inliers.insert(i);

		}
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
			find_loops = maxIterations;
	}
	cout<<"Loop #" <<find_loops<<"Find max plane: "<< inliersResult.size()<<"points"<<endl;
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac Plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
	return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.25);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
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
