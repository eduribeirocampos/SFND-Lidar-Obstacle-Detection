# Lidar-Obstacle-Detection
Udacity -Sensor Fusion - Project 1

[//]: # (Image References)
 
[image1]: ./support_imgs/final-project-ros-graph-v2.jpg
[image2]: ./support_imgs/tl-detector-ros-graph.jpg
[image3]: ./support_imgs/Object_detection.jpg
[image4]: ./support_imgs/waypoint-updater-ros-graph.jpg
[image5]: ./support_imgs/dbw-node-ros-graph.jpg
[image6]: ./support_imgs/Traffic_light_detection_test.jpg 
[image7]: ./support_imgs/Simulator.jpg 


## Eduardo Ribeiro de Campos - October 2019


# CarND- Capstone Project. 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. the Udacity project official repo is [here](https://github.com/udacity/CarND-Capstone). The simulator could be found [here](https://github.com/udacity/CarND-Capstone/releases).

### System requirements.

Because ROS is used, you will need to use Ubuntu to develop and test your project code. You may use:

- Ubuntu 14.04 with ROS Indigo
- Ubuntu 16.04 with ROS Kinetic

More details regarding `systems requirements` and `project procedures` was included at the end of this README file.

**System architecture diagram showing the ROS nodes and topics used in the project.**

![alt text][image1]

To accomplish the goals, it was necessary work on 3 nodes of the above architecture. `Traffic light detector` , `Waypoints updater` and `twist controller`.


## Traffic light detector.

This package contains the traffic light detection node: `tl_detector.py`. This node takes in data from the `/image_color`, `/current_pose`, and `/base_waypoints` topics and publishes the locations to stop for red traffic lights to the `/traffic_waypoint` topic.

The `/current_pose` topic provides the vehicle's current position, and `/base_waypoints provides` a complete list of waypoints the car will be following.

It was builded both a traffic light detection node and a traffic light classification node. Traffic light detection should take place within tl_detector.py, whereas traffic light classification should take place within `../tl_detector/light_classification_model/tl_classfier.py`.

![alt text][image2]


About the `tl_classifier`, was used a pretrained model [ssd_inception_v2_coco](http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_2018_01_28.tar.gz), and it was retrained the model considering 2 differents image scenarios `Simulator` and `real`, the pictures used are available in this [Dataset](https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view)(Thanks to [Anthony Sarkis](https://medium.com/@anthony_sarkis) for making this data set available).

To retrain the model it is necessary work with the `object_detection` from [tensorflow API](https://github.com/tensorflow/models/tree/master/research/object_detection).

The picture below was generated using the [object_detection_tutorial.ipynb](https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb)

![alt text][image3]

To retrain the models, this [repository](https://github.com/smasoudn/traffic_light_detection) is very helpful, describing a pipeline to generate the output files. 


## waypoint_updater.

This package contains the waypoint updater node: `waypoint_updater.py`. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the `/base_waypoints`, `/current_pose`, `/obstacle_waypoint`, and `/traffic_waypoint` topics, and publish a list of waypoints ahead of the car with target velocities to the `/final_waypoints` topic.

![alt text][image4]


## Twist_controller.

Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node `dbw_node.py` and the file `twist_controller.py`, along with a pid and lowpass filter that you can use in your implementation. The `dbw_node` subscribes to the `/current_velocity` topic along with the `/twist_cmd` topic to receive target linear and angular velocities. Additionally, this node will subscribe to `/vehicle/dbw_enabled`, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and `/vehicle/steering_cmd` topics.

![alt text][image5]


## Order of Project Development

1. Waypoint Updater Node (Partial): Complete a partial waypoint updater which subscribes to /base_waypoints and /current_pose and publishes to /final_waypoints.

2. DBW Node: Once your waypoint updater is publishing /final_waypoints, the waypoint_follower node will start publishing messages to the/twist_cmd topic. At this point, you have everything needed to build the dbw_node. After completing this step, the car should drive in the simulator, ignoring the traffic lights.

3. Traffic Light Detection: This can be split into 2 parts:
    - Detection: Detect the traffic light and its color from the `/image_color`. The topic `/vehicle/traffic_lights` contains the exact location and status of all traffic lights in simulator, so you can test your output.
    - Waypoint publishing: Once you have correctly identified the traffic light and determined its position, you can convert it to a waypoint index and publish it.
    
4. Waypoint Updater (Full): Use `/traffic_waypoint` to change the waypoint target velocities before publishing to `/final_waypoints`. Your car should now stop at red traffic lights and move when they are green.


## Goals.

The [rubric](https://review.udacity.com/#!/rubrics/1969/view) for this project is fairly simple - does the vehicle successfully navigate the track?

Running locally , to get good results is necessary a minimum of hardware performance, some latency will provide some issues.

To check the code, I've Run the code in 2 conditions:

1. `Manual mode ON` with the `Camera ON` , in order to check if the code is able to identify the traffic lights.The pictures below is the warn message checking the traffic light status.</br>


![alt text][image6]


2. `Manual mode OFF` with the `Camera OFF` , in this condition the car could drives following the waypoints controlled by the DBW system.


[![alt text][image7]](https://youtu.be/DqzTQGOfPWs)



## About Installation.

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.

--------------------------------------------------
### Gratitude
----------------------------------------------------

1. For all professionals and users from the strong community, that have provided alternative solutions for so many issues that  we face on the projects.</br>
2. Many thanks to [Arjun D](https://github.com/ArjunDeshmukh) , Great teammate, and student from Udacity. 
