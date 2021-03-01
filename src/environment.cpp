/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
//#include "processPointClouds.h"
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

// uncoment next line to streaming process
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)

// uncoment next line to static process   
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  //ProcessPointClouds<pcl::PointXYZI> pointProcessor;
  
 // uncoment next line to static process  
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  
  
 // uncoment this block to streaming process
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud1;
  inputCloud1 = pointProcessor.FilterCloud(inputCloud, 0.25 , Eigen::Vector4f (-10,-5,-2,1), Eigen::Vector4f (30 , 8 , 1 , 1));

  
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud2;
  inputCloud2 = pointProcessor.FilterCloud(inputCloud, 0.65 , Eigen::Vector4f (-10,-5,-2,1), Eigen::Vector4f (30 , 8 , 1 , 1));
  
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> RansacSegmentCloud1 = pointProcessor.RANSAC_SegmentPlane(inputCloud1, 150 , 0.2);

	std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> RansacSegmentCloud2 = pointProcessor.RANSAC_SegmentPlane(inputCloud2, 100 , 0.2); 

  renderPointCloud(viewer,RansacSegmentCloud2.second, "planeCloud" , Color(1,0,1));
  
  //renderPointCloud(viewer,inputCloud,"inputCloud");
  
  //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(RansacSegmentCloud.first,0.25,50,600); //uncomente this line to cluster objects from PCL library

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering_NoPCL(RansacSegmentCloud1.first,0.25,50,600);
 
  
  int clusterId = 0;

  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
    std::cout << "cluster size";
    pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
     
    Box box = pointProcessor.BoundingBox(cluster);
    renderBox(viewer, box , clusterId); 
    
     ++clusterId;      
  }
   
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    //bool renderScene = true; chance to false in point cloud visualization.
    bool renderScene = false;  
  
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
  	Lidar* lidar = new Lidar(cars,0);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
  	//renderRays(viewer, lidar->position, inputCloud);
  	//renderPointCloud(viewer , inputCloud, "inputcloud");

    // TODO:: Create point processor
  	ProcessPointClouds <pcl::PointXYZ> pointProcessor;
  	std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);

  //renderPointCloud(viewer,segmentCloud.first, "obstCloud" , Color(1,0,0));
  renderPointCloud(viewer,segmentCloud.second, "planeCloud" , Color(0,1,0));
  
  
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0 , 3.0 , 30); //uncoment for clustering with PCL library
   
   
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  
  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
    
    
    //Box box = pointProcessor.BoundingBox(cluster);
    //renderBox(viewer, box , clusterId); 
    
     ++clusterId;  
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
    //simpleHighway(viewer);
    //cityBlock (viewer); // coment this line for streaming process
  
        // uncoment this block to streaming process
        ///*  
  
    //Create point CLoud Processor;
  
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
  
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
  
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;  
    
    //*/

    while (!viewer->wasStopped ())
    {
        // uncoment this block to streaming process
        ///* 
        //Clear viewer
        viewer -> removeAllPointClouds();
        viewer -> removeAllShapes();
      
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI , inputCloudI);
      
        streamIterator++;
        if (streamIterator == stream.end())
          streamIterator = stream.begin();
          
         //*/ 
        viewer->spinOnce ();
    } 
}