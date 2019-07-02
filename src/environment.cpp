/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(my_seg_cloud.first, 0.4, 80, 1000);
    int clusterId = 0;
    //yellow for (1,1,0)
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
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
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
    //The viewer is used to handle all your visualization of objects on the screen
    //viewer is usually passed in as a reference. That way the process is more streamlined because 
    //something doesn't need to get returned.
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //CameraAngle setAngle = XY;
    CameraAngle setAngle = TopDown;
    //set up different viewing angles in your window, 
    //XY, TopDown, Side, and FPS. XY gives a 45 degree angle view, while FPS is First Person Sense and gives 
    //the sensation of being in the car’s driver seat.
    initCamera(setAngle, viewer);
    //impleHighway(viewer);
    cityBlock(viewer);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}
