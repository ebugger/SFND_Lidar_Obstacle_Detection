/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "./render/render.h"
#include "./render/box.h"
#include <chrono>
#include <string>
#include "./quiz/cluster/kdtree.h"
#include <unordered_set>

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol) {
	auto startTime = std::chrono::steady_clock::now();
	int find_loops = 0;
	std::unordered_set<int> inliersResult;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());
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
			
			pcl::PointXYZI point = cloud->points[i];
			float x0 = point.x;
			float y0 = point.y;
			float z0 = point.z;
			//fabs and sqrt
			float dist = fabs(a * x0 + b * y0 + c * z0 + d) / sqrt(a * a + b * b + c * c);
			if(dist <= distanceTol)
				inliers.insert(i);

		}
		if(inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
			find_loops = maxIterations;
		}
	}
	cout<<"Loop #" <<find_loops<<"Find max plane: "<< inliersResult.size()<<"points"<<endl;
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac Plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
	//return inliersResult;

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult(cloudOutliers, cloudInliers);

    return segResult;

}

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

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanCluster( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{
	auto startTime = std::chrono::steady_clock::now();
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters_;
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<std::vector<float>> points;
    std::vector< std::vector<float> > points_;
	//or use std::vector<bool> process_list(points.size(), false) and check if(process_list[i]) in future;
	std::unordered_set<int> process_list {};
	std::vector<pcl::PointIndices> cluster_indices;
    
    KdTree* tree = new KdTree;
	
	//optimaized for binary tree(extract the median element in the dimention)
	int p_size = cloud->points.size();
	for (int i=0;i<p_size;i++ ){
	    std::vector<float> temp;	
		temp.push_back(cloud->points[i].x);
		temp.push_back(cloud->points[i].y);
		temp.push_back(cloud->points[i].z);
		points.push_back(temp);
	}
    for (int i=0; i<p_size; i++) {
		int idx = i % 3;
		//get the median element on rolling x->y->z
        std::nth_element(points.begin(), points.begin() + points.size()/2, points.end(),
            [idx](std::vector<float> const &lhs,
             std::vector<float> const &rhs) { return lhs[idx] < rhs[idx]; });
		tree->insert(points[points.size()/2],i);
		//reorder the the points by the insert order
		cloud->points[i].x = points[points.size()/2][0];
                cloud->points[i].y = points[points.size()/2][1];
                cloud->points[i].z = points[points.size()/2][2];
		points_.push_back(points[points.size()/2]);
		//remove the median(swap to end and popback)
		iter_swap(points.begin()+points.size()/2,points.end()-1);
		points.pop_back();		
	}

	for (int i=0;i<points_.size();i++) {
		if(process_list.count(i) == 0) {
			pcl::PointIndices cluster;
			//std::vector<int> cluster {};
			Proximity(points_, cluster.indices, i, process_list, tree, distanceTol);
			if((cluster.indices.size() >= minSize) && (cluster.indices.size() <= maxSize))
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
	auto endTime = std::chrono::steady_clock::now();
    	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    	std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters_.size() << " clusters" << std::endl;	
	return clusters_;

}
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    //view the point cloud without obstacles when set to false
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Complete:: Create lidar sensor 
    // The lidar arguments are necessary for modeling ray collisions. The Lidar object is going to be holding 
    //point cloud data which could be very large. By instatinating on the heap, we have more memory to work with
    // than the 2MB on the stack. However, it takes longer to look up objects on the heap, while stack lookup 
    // is very fast.
    Lidar* lidarSensor = new Lidar(cars, .0);
    //The Ptr type from PointCloud indicates that the object is actually a pointer - a 32 bit integer that 
    //contains the memory address of your point cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr clouds = lidarSensor->scan();
    //render points and shapes to the pcl viewer.render your lidar rays as line segments in the viewer.
    //renderRays(viewer, lidarSensor->position, clouds);
    //renderPointCloud(viewer, clouds, "test", Color(1,1,1));

    // TODO:: Create point processor on the stack
    ProcessPointClouds<pcl::PointXYZ>  processor_PC;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> my_seg_cloud = processor_PC.SegmentPlane(clouds, 100, 0.2);
    //Color(R,G,B)
    renderPointCloud(viewer, my_seg_cloud.first, "obstacle_cloud", Color(1,0,0));
    renderPointCloud(viewer, my_seg_cloud.second, "plnae_cloud", Color(0,1,0));

    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor_PC.Clustering(my_seg_cloud.first, 1.0, 3, 30);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor_PC.Clustering(my_seg_cloud.first, 1.0, 3, 30);
    int clusterId = 0;
    //yellow for (1,1,0)
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        processor_PC.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        //using bbox
        Box box = processor_PC.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer) {
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI> ();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, inputcloud, "inputcloud" );
    //from topdown view, up for positive x, left for positive y, and the origin for z is on the top of the car.
    filter_cloud = pointProcessorI->FilterCloud(inputcloud, 0.15f, Eigen::Vector4f (-10, -4, -2, 1), Eigen::Vector4f (32, 7, 1, 1));
    //renderPointCloud(viewer, filter_cloud, "filterCould");
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> my_seg_cloud = pointProcessorI->SegmentPlane(filter_cloud, 100, 0.2);
    //Color(R,G,B)
    //renderPointCloud(viewer, my_seg_cloud.first, "obstacle_cloud", Color(1,0,0));
    renderPointCloud(viewer, my_seg_cloud.second, "plnae_cloud", Color(0,1,0));
    //Max is so important as big objects always contains much more points.
    Box roof_box = {-2.6, -1.7, -1, 2.6, 1.7, -.4};
    renderBox(viewer, roof_box, 33265, Color(128,0,128));
    int clusterId = 0;
    //yellow for (1,1,0)
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(my_seg_cloud.first, 0.4, 80, 1000);
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        //using bbox
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }

}

void newcityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputcloud) {
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud (new pcl::PointCloud<pcl::PointXYZI> ());

    //renderPointCloud(viewer, inputcloud, "inputcloud" );
    //from topdown view, up for positive x, left for positive y, and the origin for z is on the top of the car.
    filter_cloud = pointProcessorI->FilterCloud(inputcloud, 0.15f, Eigen::Vector4f (-10, -4, -2, 1), Eigen::Vector4f (32, 7, 1, 1));
    //renderPointCloud(viewer, filter_cloud, "filterCould");
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> my_seg_cloud = pointProcessorI->SegmentPlane(filter_cloud, 100, 0.25);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> my_seg_cloud = RansacPlane(filter_cloud, 100, 0.25);
    //Color(R,G,B)
    //renderPointCloud(viewer, my_seg_cloud.first, "obstacle_cloud", Color(1,0,0));
    renderPointCloud(viewer, my_seg_cloud.second, "plnae_cloud", Color(0,1,0));
    //Max is so important as big objects always contains much more points.
    Box roof_box = {-2.6, -1.7, -1, 2.6, 1.7, -.4};
    //renderBox(viewer, roof_box, 33265, Color(128,0,128));
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(my_seg_cloud.first, 0.4, 80, 1000);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = euclideanCluster(my_seg_cloud.first, 0.4, 80, 1000);
    int clusterId = 0;
    //yellow for (1,1,0)
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        std::cout << cluster->points.size() << std::endl;
        //pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        //using bbox
        Box box = BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        //Box box = pointProcessorI->BoundingBox(cluster);
        //renderBox(viewer, box, clusterId);
        ++clusterId;
    }

}
//Another way to Create point processor on the heap(main memory)
//ProcessPointClouds<pcl::PointXYZ>*  porce_PC = new ProcessPointClouds<pcl::PointXYZ>();

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-6, 0, -0.6, -0.2, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
    //std::vector<pcl::visualization::Camera> cam; 
    //The viewer is used to handle all your visualization of objects on the screen
    //viewer is usually passed in as a reference. That way the process is more streamlined because 
    //something doesn't need to get returned.
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //CameraAngle setAngle = XY;
    CameraAngle setAngle = FPS;
    //set up different viewing angles in your window, 
    //XY, TopDown, Side, and FPS. XY gives a 45 degree angle view, while FPS is First Person Sense and gives 
    //the sensation of being in the car’s driver seat.
    initCamera(setAngle, viewer);
    //impleHighway(viewer);
    //cityBlock(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI> ();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloudI;
    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        inputcloudI = pointProcessorI->loadPcd((*streamIterator).string());
        newcityBlock(viewer, pointProcessorI, inputcloudI);
	//viewer->getCameras(cam);
	//cout << "Cam: " << endl 
        //     << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl 
        //     << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl; 

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}


