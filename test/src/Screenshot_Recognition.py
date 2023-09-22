#!/usr/bin/env python3

import apriltag
import cv2
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Initialize the camera capture object
cam_port = 0

# Capture an image from the camera
cam = cv2.VideoCapture(cam_port)
result, image = cam.read()

# If a valid image was captured:
if result:
    cv2.imshow("AR_Tag", image)
    cv2.imwrite("AR_Tag.png", image)

    # Conversion to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply a threshhold to detect the tag from its surrounding
    ret, thresh1 = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY + 
                                                cv2.THRESH_OTSU)    

    print("[INFO] detecting AprilTags...")

    # Detecting apriltag and retrieving it ID
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(thresh1)
    AR_Tag_id = results[0].tag_id

    #cv2.waitKey(0)

# If captured image is corrupted:
else:
	print("No image detected. Please! try again")

# Function to send the nav_goal to move_base action server according to ARtag ID
def movebase_client():
    global AR_Tag_id

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    if AR_Tag_id == 20 :
        goal.target_pose.pose.position.x = 0.5
        goal.target_pose.pose.orientation.w = 1.0
    elif AR_Tag_id == 21 :
        goal.target_pose.pose.position.x = 0.3
        goal.target_pose.pose.orientation.w = 0.25
    elif AR_Tag_id == 22 :
        goal.target_pose.pose.position.x = -0.75
        goal.target_pose.pose.orientation.w = -0.75
    else: 
        goal.target_pose.pose.position.x = 0.0
        goal.target_pose.pose.orientation.w = 0.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    
    
    rospy.init_node('movebase_client_py')
    result = movebase_client()
    if result:
        rospy.loginfo("Reached goal")
    



