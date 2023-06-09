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
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
	Lidar* lidar = new Lidar(cars, 0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr points = lidar->scan();
	ProcessPointClouds<pcl::PointXYZ> point_process;
	std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_cloud = point_process.RANSAC(points, 100, 0.2);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cluster = point_process.Clustering(segment_cloud.first, 1.0, 3, 30);
	int cluster_index = 0;
	std::vector<Color> colors = { Color(1,0,0),Color(0,1,0),Color(0,0,1),Color(1,0,1) };
	for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloud_cluster) {
		renderPointCloud(viewer, cluster, "car" + std::to_string(cluster_index), colors[cluster_index%colors.size()]);
		Box box = point_process.BoundingBox(cluster);
		renderBox(viewer, box, cluster_index);
		++cluster_index;
	}


    // TODO:: Create point processor
  
}
void city_representation(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* process_point, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input) {
	//1.) read input cloud and perfrom filtering
	cloud_input = process_point->FilterCloud(cloud_input, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 1, 1));
	//2. Perform RANSAC based planar segmentation
	std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = process_point->RANSAC(cloud_input, 100, 0.2);
	//View obstacles
	renderPointCloud(viewer, segment_cloud.second, "plane", Color(1, 1, 0));
	//3.perform clustering
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = process_point->Clustering(segment_cloud.first, 0.53, 20, 125);
	int cluster_index = 0;
	std::vector<Color> colors = { Color(1,0,0),Color(0,0,1),Color(1,1,0) };
	for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters) {
		renderPointCloud(viewer, cluster, "car" + std::to_string(cluster_index), colors[cluster_index%colors.size()]);
		Box box = process_point->BoundingBox(cluster);
		renderBox(viewer, box, cluster_index);
		cluster_index++;
	}
}

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

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
	auto streamIterator = stream.begin();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

	while (!viewer->wasStopped ())
	{
		// Clear viewer
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		// Load pcd and run obstacle detection process
		inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
		city_representation(viewer, pointProcessorI, inputCloudI);

		streamIterator++;
		if(streamIterator == stream.end())
			streamIterator = stream.begin();

		viewer->spinOnce ();
    } 
}
