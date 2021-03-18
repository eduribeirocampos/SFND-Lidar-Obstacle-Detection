[//]: # (Image References)
 
[image1]: ./images/pcdstream_1.gif
[image2]: ./images/ransac-linie-animiert.gif
[image3]: ./images/Ransac2D.jpg
[image4]: ./images/kdtree5.jpg
[image5]: ./images/KdTree2D.jpg
[image6]: ./images/final_video.jpg



## Eduardo Ribeiro de Campos - March 2021.

# Project: Lidar Obstacle Detection.
## Sensor Fusion Nanodegree Program.

![alt text |width=450px | align="middle"][image1]


Throughout the Lidar course, we learned perspectives about Lidar with [Michael Maile](https://www.linkedin.com/in/michael-maile-ab7a078/).(Director, Sensor Fusion & Localization) and the software approach was teached by [Aaron Brown](https://www.linkedin.com/in/awbrown90/) (Senior AV Software Engineer). Both professionals from [MBRDNA](https://www.mbrdna.com/) (Mercedes-Benz Reasearch & Development North America, Inc) team.

It was studied the best practice to develop a pipeline to work with Lidar data.For more details please check the [Velodyne website](https://velodynelidar.com/products/hdl-64e/) and the [Point Cloud Library](https://pointclouds.org/).

The whole process consider the next 4 main steps.

1.Segmentation (RANSAC).<br/>
2.Clustering (KD-Tree).<br/>
3.Bounding boxes.<br/>
4.Filtering and Downsampling data. (Voxel Grid - Region of Interest).<br/>


## 1. Segmentation.

We want to be able to locate obstacles in the scene. However, some objects in our scene are not obstacles. What would be objects that appear in the pcd but are not obstacles? For the most part, any free space on the road is not an obstacle, and if the road is flat it’s fairly straightforward to pick out road points from non-road points. To do this we will use a method called Planar Segmentation which uses the `RANSAC` (random sample consensus) algorithm.

The next gif file shows the RANSAC method applied for line fitting with outliers.

![alt text |width=450px | align="middle"][image2]


One type of RANSAC version selects the smallest possible subset of points to fit. For a line, that would be two points, and for a plane three points. Then the number of inliers are counted, by iterating through every remaining point and calculating its distance to the model. The points that are within a certain distance to the model are counted as inliers. The iteration that has the highest number of inliers is then the best model. This algorithm is availabe on the file [line_ransac2d.cpp](./src/quiz/ransac/line_ransac2d.cpp). To run This simulation follow the next steps in `linux terminal`:

```
* Go to ..src/quiz/ransac/build
* execute ./quizRansac
```
Below the result.

![alt text |width=450px | align="middle"][image3]

The Ransac Function used in the main [code](/src/) is available on file [processPointClouds.cpp](src/processPointClouds.cpp) from lines 134 to 222.


## 2. Clustering.

You have a way to segment points and recognize which ones represent obstacles for your car. It would be great to break up and group those obstacle points, especially if you want to do multiple object tracking with cars, pedestrians, and bicyclists, for instance. One way to do that grouping and cluster point cloud data is called euclidean clustering.

### KD-Tree and Euclidean Clustering.

The idea is you associate groups of points by how close together they are. To do a nearest neighbor search efficiently, you use a KD-Tree data structure which, on average, speeds up your look up time from O(n) to O(log(n)). This is because the tree allows you to better break up your search space. By grouping points into regions in a KD-Tree, you can avoid calculating distance for possibly thousands of points just because you know they are not even considered in a close enough region.

Once points are able to be inserted into the tree, the next step is being able to search for nearby points inside the tree compared to a given target point. Points within a distance of distanceTol are considered to be nearby. The KD-Tree is able to split regions and allows certain regions to be completely ruled out, speeding up the process of finding nearby neighbors.

A KD-Tree is a binary tree that splits points between alternating axes. By separating space by splitting regions, nearest neighbor search can be made much faster when using an algorithm like euclidean clustering. Below the image showing a schematic tree to make easy undertand the concept. The new point (7.2, 6.1) must inserted to D node.

![alt text |width=450px | align="middle"][image4]

Once the KD-Tree method for searching for nearby points is implemented, its not difficult to implement a euclidean clustering method that groups individual cluster indices based on their proximity

This algorithm is availabe on the file [cluster.cpp](./src/quiz/cluster/cluster.cpp). To run the simulation follow the next steps in `linux terminal`:

```
* Go to ..src/quiz/cluster/build
* execute ./quizCluster
```
Below the result.

![alt text |width=450px | align="middle"][image5]

The Cluster code in the main code is available from lines xxxx to xxxx.


The implementation of `Clustering Points` was constructed in 3 Function, `clusterHelper` , `euclideanCluster` and `Clustering_NoPCL`. In the main code is available on file [processPointClouds.cpp](./src/processPointClouds.cpp) from lines 268 to 343.


## 3.Bounding boxes.

according to Michael Maile [video](https://www.youtube.com/watch?v=kk39stQPG84&feature=emb_logo):

"Bounding boxes are relatively easy way of dealing with objects because then you just associate the points to a bounding box or now if you say, Okay, this object is in the bounding box, Okay, this is now pedestrian , this point is not on the boundingbox, this is something else.It is relatively easy way of dealing with it. The issue though is that you are making model assumptions about the size of the bounding box that you assign to let say a car, or to a pedestrian, and this violates the assumptions of the bounding box,then things tend to look somehow strange".

The Bounding box function is available on file: [processPointClouds.cpp](./src/processPointClouds.cpp)  from lines 345 to 362.

## 4.Filtering and Downsampling data

### Voxel Grid.
Voxel grid filtering will create a cubic grid and will filter the cloud by only leaving a single point per voxel cube, so the larger the cube length the lower the resolution of the point cloud.

### Region of Interest
A boxed region is defined and any points outside that box are removed.

The arguments to this function will be your input cloud, voxel grid size, and min/max points representing your region of interest. The function will return the downsampled cloud with only points that were inside the region specified. To get started check out the documentation from PCL for [voxel grid filtering](https://pointclouds.org/documentation/tutorials/voxel_grid.html) and [region of interest](https://pointclouds.org/documentation/classpcl_1_1_statistical_multiscale_interest_region_extraction.html).

# Goals.

The goals of the project is available in the [project Rubric](https://review.udacity.com/#!/rubrics/2529/view) webpage.

# Final result.

The `Main Code` is the file [environment.cpp](./src/environment.cpp). To run the simulation follow the next steps in `linux terminal`:

```
* Go to ..src/build
* execute ./environment
```
click on the next picture to open the video to show the results:

[![alt text |width=450px | align="middle"][image6]](https://youtu.be/1fXyvsJBaxU)

## Support resources.

Some images and texts from this readme file are from the lessons class of the Sensor fusion - Udacity  Nanodegree program.
