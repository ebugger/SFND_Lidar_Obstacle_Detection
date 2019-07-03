/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"
#include <unordered_set>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
Box BoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    pcl::PointXYZI minPoint;
	pcl::PointXYZI maxPoint;
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
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, window.z_min, window.z_max, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZI point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = points[i][2];
		point.intensity = .0f;
  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%3==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, window.z_min),pcl::PointXYZ(node->point[0], window.y_max, window.z_max),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else if(depth%3==1)
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], window.z_min),pcl::PointXYZ(window.x_max, node->point[1], window.z_max),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		else if(depth%3==2)
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, window.y_min, node->point[2]),pcl::PointXYZ(window.x_max, window.y_max, node->point[2]),1,1,0,"line"+std::to_string(iteration));
			lowerWindow.z_max = node->point[2];
			upperWindow.z_min = node->point[2];
		}		
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}

void Proximity(const std::vector<std::vector<float>>& points, std::vector<int>& cluster, int idx, std::unordered_set<int> &process_list,  KdTree* tree, float distanceTol){
	std::vector<int> nearby_points;
	process_list.insert(idx);
	cluster.push_back(idx);
	nearby_points = tree->search(points[idx], distanceTol);
	for(int nearby_id : nearby_points) {
		if(process_list.count(nearby_id) == 0){
			Proximity(points, cluster, nearby_id, process_list, tree, distanceTol);
		}
	}

}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, KdTree* tree, float distanceTol)
{
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters_;
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<std::vector<float>> points;
	//or use std::vector<bool> process_list(points.size(), false) and check if(process_list[i]) in future;
	std::unordered_set<int> process_list {};
	std::vector<pcl::PointIndices> cluster_indices;

	std::vector<float> temp;
	for (int i=0;i<cloud->points.size();i++ ){
	        std::vector<float> temp;	
		temp.push_back(cloud->points[i].x);
		temp.push_back(cloud->points[i].y);
		temp.push_back(cloud->points[i].z);
		points.push_back(temp);
	}
	for (int i=0;i<points.size();i++) {
		if(process_list.count(i) == 0) {
			pcl::PointIndices cluster;
			Proximity(points, cluster.indices, i, process_list, tree, distanceTol);
			//if((cluster.indices.size() >= minSize) && (cluster.indices.size() <= maxSize));
			cluster_indices.push_back(cluster);
		}
	}
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
            cloud_cluster->points.push_back (cloud->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters_.push_back(cloud_cluster);
    }	
	return clusters_;

}

int main ()
{

	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min = -10;
  	window.z_max =  10;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7,8}, {-6.3,8.4,-8}, {-5.2,7.1,-0.6}, {-5.7,6.3, 0.4}, {7.2,6.1,1}, {8.0,5.3,-1}, {7.2,7.1,0.8}, {0.2,-7.1,-0.6}, {1.7,-6.9,0.4}, {-1.2,-7.2,-6.2}, {2.2,-8.9,8.7} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	std::vector< std::vector<float> > points_;
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = CreateData(points);
	//renderPointCloud(viewer,cloud,"data");

	KdTree* tree = new KdTree;
	
	//optimaized for binary tree(extract the median element in the dimention)
	int p_size = points.size();
    for (int i=0; i<p_size; i++) {
		int idx = i % 3;
		//get the median element on rolling x->y->z
        std::nth_element(points.begin(), points.begin() + points.size()/2, points.end(),
            [idx](std::vector<float> const &lhs,
             std::vector<float> const &rhs) { return lhs[idx] < rhs[idx]; });
		tree->insert(points[points.size()/2],i);
		//reorder the the points by the insert order
		points_.push_back(points[points.size()/2]);
		//remove the median(swap to end and popback)
		iter_swap(points.begin()+points.size()/2,points.end()-1);
		points.pop_back();		
	}
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = CreateData(points_);
  	//int it = 0;
  	//render2DTree(tree->root,viewer,window, it);
  
  	std::cout << "Test Search" << std::endl;
  	std::vector<int> nearby = tree->search({-6,7,-0.6},3.0);
  	for(int index : nearby) {
      		std::cout << index << ",";
  		std::cout << std::endl;
	}
  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//from the reordered points
  	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = euclideanCluster(cloud, tree, 2.0);
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;
    int clusterId = 0;
    //yellow for (1,1,0)
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters) {
        std::cout << "cluster size ";
        std::cout << cluster->points.size() << std::endl;
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        //using bbox
        Box box = BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
	/* 
  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points_[indice][0],points_[indice][1],points_[indice][2]));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
	  */
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
}


