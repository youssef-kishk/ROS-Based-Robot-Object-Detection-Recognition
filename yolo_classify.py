#!/usr/bin/env python
import ssl
ssl._create_default_https_context = ssl._create_unverified_context

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

from cv_bridge import CvBridge
import csv



CONFIDENCE = 0.5
SCORE_THRESHOLD = 0.5
IOU_THRESHOLD = 0.5
config_path = "/home/youssef/ROS_ws/src/yolo_classify/yolo_files/yolov2.cfg"
weights_path = "/home/youssef/ROS_ws/src/yolo_classify/yolo_files/yolov2.weights"
font_scale = 1
thickness = 1

LABELS = open("/home/youssef/ROS_ws/src/yolo_classify/yolo_files/coco.names").read().strip().split("\n")
colors = np.random.randint(0, 255, size=(len(LABELS), 3), dtype="uint8")

net = cv2.dnn.readNetFromDarknet(config_path, weights_path)

ln = net.getLayerNames()
ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]


rospy.init_node('yolo_classify', anonymous=True)

pub_pose_msg = rospy.Publisher('goto_position', Pose, queue_size = 1)




bridge = CvBridge()
msg_string = String()
current_odom = Odometry()

#objects pose on map
objects_locations = {} 
flag_file_read = True   


#Funcion for Yolo detection of objects
def yolo_function(image):
    h, w = image.shape[:2]
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    layer_outputs = net.forward(ln)
    boxes, confidences, class_ids = [], [], []

    # loop over each of the layer outputs
    for output in layer_outputs:
        # loop over each of the object detections
        for detection in output:
            # extract the class id (label) and confidence (as a probability) of
            # the current object detection
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            # discard weak predictions by ensuring the detected
            # probability is greater than the minimum probability
            if confidence > CONFIDENCE:
                # scale the bounding box coordinates back relative to the
                # size of the image, keeping in mind that YOLO actually
                # returns the center (x, y)-coordinates of the bounding
                # box followed by the boxes' width and height
                box = detection[:4] * np.array([w, h, w, h])
                
                (centerX, centerY, width, height) = box.astype("int")

                # use the center (x, y)-coordinates to derive the top and
                # and left corner of the bounding box
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))

                # update our list of bounding box coordinates, confidences,
                # and class IDs
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # perform the non maximum suppression given the scores defined before
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, SCORE_THRESHOLD, IOU_THRESHOLD)
    font_scale = 1
    thickness = 1

    # ensure at least one detection exists
    if len(idxs) > 0:
        # loop over the indexes we are keeping
        for i in idxs.flatten():
            # extract the bounding box coordinates
            x, y = boxes[i][0], boxes[i][1]
            #published_msg = "x= %d : y= %d" %(x,y)
            #pub.publish(published_msg)
            w, h = boxes[i][2], boxes[i][3]
            
            #published_msg = "w= %d : h= %d" %(w,h)
            #pub.publish(published_msg)
            # draw a bounding box rectangle and label on the image
            color = [int(c) for c in colors[class_ids[i]]]
            cv2.rectangle(image, (x, y), (x + w, y + h), color=color, thickness=thickness)
            text = "%s: %.2f" % (LABELS[class_ids[i]][:-1],confidences[i])
            
            # calculate text width & height to draw the transparent boxes as background of the text
            (text_width, text_height) = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale, thickness=thickness)[0]
            text_offset_x = x
            text_offset_y = y - 5
            box_coords = ((text_offset_x, text_offset_y), (text_offset_x + text_width + 2, text_offset_y - text_height))
            overlay = image.copy()
            cv2.rectangle(overlay, box_coords[0], box_coords[1], color=color, thickness=cv2.FILLED)
            # add opacity (transparency to the box)
            image = cv2.addWeighted(overlay, 0.6, image, 0.4, 0)

            # now put the text (label: confidence %)
            cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=font_scale, color=(0, 0, 0), thickness=thickness)
            
            
            
            #update objects locations
            if(check_distance(x,y,w,h)):
                objects_locations[LABELS[class_ids[i]][:-1]] =  current_odom.pose.pose
            
            
    cv2.imshow("image", image)
    #print(objects_locations.keys())
 
def check_distance(x,y,w,h):

    if((depth_image[y,x]/10) <60.0 and (x+w<640 and (depth_image[y,x+w]/10)<60.0) and (y+h<480 and (depth_image[y+h,x]/10)<60.0)
                and (depth_image[y+h,x+w]/10)<60.0 and (depth_image[y+(h/2),x+(w/2)]/10)<60.0):
                    return True
    else:
        return False
    
      
#Send Pose to robot to move             
def send_goto_msg(object_string):
    #extract object name from msg [Data: "objectname"]
    object_name =  str(object_string)[7:-1]

    if(object_name in objects_locations.keys()):
        print "Benzo is going to Object: %s" % (object_name)
        pub_pose_msg.publish(objects_locations[object_name])
        
    else:
         print ("Object Not Found")

        
 
#Callback function to get odometry from /rtabmap/odom'  
def get_odmetry(odom):
       global current_odom
       current_odom = odom
       
depth_image =0    
def get_distance(img):
       global depth_image
       bridge=CvBridge()
       depth_image = bridge.imgmsg_to_cv2(img, desired_encoding="32FC1")




#Write object:Pose in a csv file
fieldnames = ["object", "positionX", "positionY", "positionZ", "orientationX", "orientationY", "orientationZ", "orientationW"]
def write_objects_to_file():
    with open('out.csv', 'w') as outcsv:
        writer = csv.DictWriter(outcsv, fieldnames)
        #writer.writeheader()
        
        for object_name in objects_locations.keys():
            pose_current_object = objects_locations[object_name]
            writer.writerow({"object":object_name,"positionX":pose_current_object.position.x, "positionY":pose_current_object.position.y, "positionZ":pose_current_object.position.z,
                                   "orientationX":pose_current_object.orientation.x, "orientationY":pose_current_object.orientation.y,
                                   "orientationZ":pose_current_object.orientation.z, "orientationW":pose_current_object.orientation.w})

#Read object:Pose from csv file             
def read_objects_from_file():
    try:
        with open('out.csv') as csvfile:
            readCSV = csv.reader(csvfile, delimiter=',')
            for row in readCSV:
                update_object_map_from_file(row)
    except:
        print(objects_locations.keys())
        
        
def update_object_map_from_file(object_pose):
        new_msg_pose = Pose()
        new_msg_pose.position.x = float(object_pose[1])
        new_msg_pose.position.y = float(object_pose[2])
        new_msg_pose.position.z = float(object_pose[3])

        new_msg_pose.orientation.x = float(object_pose[4])
        new_msg_pose.orientation.y = float(object_pose[5])
        new_msg_pose.orientation.z = float(object_pose[6])
        new_msg_pose.orientation.w = float(object_pose[7])
        objects_locations[object_pose[0]] =  new_msg_pose
   


          
#True in exploration phase, False in navigation phase             
UPDATE_OBJECTS_MAP = True
    
    
def callback(image_msg):
    global flag_file_read
    if flag_file_read:
        read_objects_from_file()
        flag_file_read = False   
    #First convert the image to OpenCV image 
    if UPDATE_OBJECTS_MAP:
        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        yolo_function(cv_image)
        cv2.waitKey(1)
        
        #update objects:pose csv file
        write_objects_to_file()
    
    
    
    
rospy.Subscriber("/camera/rgb/image_color", Image, callback, queue_size = 1, buff_size = 16777216)

rospy.Subscriber('/camera/depth/image_raw',Image,get_distance)

#position of the object in the map
rospy.Subscriber('/rtabmap/odom', Odometry, get_odmetry)

#message from mobile app
rospy.Subscriber('/app_to_benzo', String, send_goto_msg)        

while not rospy.is_shutdown():
  rospy.spin()
