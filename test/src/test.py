#!/usr/bin/env python3
# license removed for brevity

import cv2
import apriltag
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

path = r"/home/ilef/Downloads/tags_test (2)/tags_test/ar_tag_2.JPG"

image = cv2.imread(path)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

ret, thresh1 = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY + 
                                            cv2.THRESH_OTSU)    

detector = apriltag.Detector()
results = detector.detect(thresh1)
AR_Tag_id = results[0].tag_id
cv2.waitKey(0)

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
    
