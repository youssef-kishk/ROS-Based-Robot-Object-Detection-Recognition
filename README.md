# ROS Based Robot Object detection and Recognition Task
This repository contains a description of the Object detection and recognition Task for (**Benzo**), a personal assistance robot based on **ROS (Robot Operating System)** that is able to perform multiple tasks such as:
- Handling voice commands
- Map Building
- Objects Detection and recognition
- Navigation and Localization

You can check the project summary from the following **video** (https://youtu.be/AEgZd6wD7dk)


## Introduction
In the Object detection and recognition task, we are dealing with a **Microsoft xbox 360 Kinect camera RGB D** to have a real time image of the environment of the robot, which is then used to detect different objects using **You Only Look Once (YOLO) version 2** model, pretrained on a dataset of 80 different object categories [Common Objects in Context (COCO) dataset](https://cocodataset.org/).

## Task Description
The following are the detailed task steps:
1) Extracting the real time image from the Kinect Camera
2) Detect and Recognize different objects in each image frame from the camera
3) Check distance between the robot camera and each of the detected objects
4) Each of the detected objects within a certain distance from the robot camera is stored with its equivelant position on the envirnoment map extracted from the Rtabmap current odometry published topic
5) While performing the above 4 steps, the robot keep waiting for any voice command sent from the user using the mobile app including the name of an object, in order to start navigating towards the required it if exists in the stored dictionary of previously detected objects and their location, as shown in the figure below.

<p align="center">
<img src="https://github.com/youssef-kishk/ROS-Based-Robot-Object-Detection-Recognition-Module/blob/master/images/image.png" width="600" height="300" />
 </p>
 
## Task Steps
### Extracting the real time image from the Kinect Camera
Using the [cv_bridge](http://wiki.ros.org/cv_bridge) ROS package which converts between ROS Image messages the kinect camera sends on the ROS Topic `/camera/rgb/image_color` to OpenCV images so we can deal with in our detection and recognition process.

<p align="center">
<img src="http://wiki.ros.org/cv_bridge?action=AttachFile&do=get&target=cvbridge.png" width="300" height="300" />
 </p>
 
 ### Detect and Recognize objects
 Using [**YOLO**](https://pjreddie.com/darknet/yolo/) pretrained on COCO dataset,
 We were able to detect different objects in each of the opencv images
 
 <p align="center">
<img src="https://github.com/youssef-kishk/ROS-Based-Robot-Object-Detection-Recognition-Module/blob/master/images/image2.png" width="600" height="400" />
 </p>
 
  You can check YOLO details from its official papers at:
 - https://arxiv.org/pdf/1506.02640.pdf
 - https://arxiv.org/pdf/1612.08242.pdf
 - https://arxiv.org/pdf/1804.02767.pdf
 
### Check distance between the robot camera and each of the objects
One of the main challenges of the task was to determine the distance between the robot and the different objects detected, so we can store each of the objects with its approximated exact position on the map in order to navigate towards the object successfully then.

Using the Kinect 3D depth sensor cameras we were able to extract a 2D array for the depth from the Robot on each pixel of the detection image containing multiple objects as the one shown in the previous figure.
As shown in below figure, Assume **a,b,c,d** is the frame of a detected object so we can get the depth values from the 2D depth array at the edges of that frame which is values: *a, b, c, d, e* in mm
 <p align="center">
<img src="https://github.com/youssef-kishk/ROS-Based-Robot-Object-Detection-Recognition-Module/blob/master/images/image3.png" width="300" height="300" />
 </p>
 
- `a = depth_2D_array[y,x])`
- `b = depth_2D_array[y,x+w])`
- `c = depth_2D_array[y+h,x])`
- `d = depth_2D_array[y+h,x+w])`
- `e = depth_2D_array[y+ (h/2), x+ (w/2)])`

By checking those depth values we can make an accurate estimation of how far the object is from the Robot and accept it if within an acceptable range and close enough.

### Store Objects and their equivelant depth values
For the objects accepted within the range from the previous step, we store each detected object name as a string associated with its position on the map captured from the ROS topic `/rtabmap/odom` , so it can be used later when the user wants the robot to navigate to any specific object by simply extracting the position value from the map and pass it to the GO-TO module.


| Object Name   | position.x    | position.y    | position.z    | orientation.x | orientation.y | orientation.z | orientation.w |
| ------------- |:-------------:|:-------------:|:-------------:|:-------------:|:-------------:|:-------------:|:-------------:|
|               |               |               |               |               |               |               |               |

### Mobile App and Navigating
While performing the above steps the robot keeps subscribing to `/app_to_benzo` ROS Topic, waiting for the user using the mobile app to send a voice message of an object name. The name is converted to a string and sent to the robot on the topic. If the object is found in the live updated dictionary, the stored position of the object is sent to the GO-TO module on the `/goto_position` ROS Topic, so the robot can start navigating towards it.

## Summary
This repository shows how the Object detection and Recognition task can be performed for a personal assistance robot based on **ROS (Robot Operating System)**.

The whole project can be found in this youtube [video](https://youtu.be/AEgZd6wD7dk)
