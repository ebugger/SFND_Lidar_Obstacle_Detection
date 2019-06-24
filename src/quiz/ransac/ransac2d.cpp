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

/* 
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	while(maxIterations--){
		//hash set with no order, with unique element
		std::unordered_set<int> inliers;
		while(inliers.size() < 2) {
			//rand() will generate very large number and we got a MOD operator, so the result will be in 0 and the points size.
			inliers.insert(rand() % (cloud->points.size()));
		}

		float x1, x2, y1, y2;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;	

		float A = (y1 - y2);
		float B = (x2 - x1);
		float C = (x1 * y2 - x2 * y1);

		for (int i=0;i<cloud->points.size();i++) {
			//count on a set to check if the element is in the set, not zero means it contains
			if(inliers.count(i) > 0)
				//do nothing
				continue;
			
			pcl::PointXYZ point = cloud->points[i];
			float x0 = point.x;
			float y0 = point.y;
			//fabs and sqrt
			float dist = fabs(A * x0 + B * y0 + C ) / sqrt(A*A + B*B);

			if(dist <= distanceTol)
				inliers.insert(i);

		}
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;

	}
	return inliersResult;

}
*/

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	while(maxIterations--){
		//hash set with no order, with unique element
		std::unordered_set<int> inliers;
		while(inliers.size() < 3) {
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

		/* 
		float A = (y1 - y2);
		float B = (x2 - x1);
		float C = (x1 * y2 - x2 * y1);
		*/
		float i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float j = (z2 - z1) * (x2 - x1) - (x2 - x1) * (x3 - z1);
		float k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		float d = -(i * x1 + j * y1 + k * z1);
		//float i = (y2−y1) * (z3−z1) − (z2−z1) * (y3−y1);
		//float j = (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1);
		//float k = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1);
		//float d = -(i * x1 + j * y1 + k * z1);

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
			//float dist = fabs(A * x0 + B * y0 + C ) / sqrt(A*A + B*B);
			float dist = fabs(i * x0 + j * y0 + k * z0 + d) / sqrt(i*i + j*j + k*k);
			if(dist <= distanceTol)
				inliers.insert(i);

		}
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;

	}
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
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.2);

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
