#!/usr/bin/env python3
import rospy
import cv2
import apriltag
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Define the path to the desired image 
path = r"/home/ilef/Downloads/tags_test (2)/tags_test/ar_tag_2.JPG"
print("[INFO] loading image...")
# Load the image
image = cv2.imread(path)

# Conversion to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply a threshhold to detect the tag from its surrounding
ret, thresh1 = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY + 
                                            cv2.THRESH_OTSU)    

# Detecting apriltag and retrieving it ID
print("[INFO] detecting AprilTags...")
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
results = detector.detect(thresh1)
AR_Tag_id = results[0].tag_id

print(AR_Tag_id)

# Draw the tag limits
for r in results:
	# extract the bounding box (x, y)-coordinates for the AprilTag
	(ptA, ptB, ptC, ptD) = r.corners
	ptB = (int(ptB[0]), int(ptB[1]))
	ptC = (int(ptC[0]), int(ptC[1]))
	ptD = (int(ptD[0]), int(ptD[1]))
	ptA = (int(ptA[0]), int(ptA[1]))
        
	# draw the bounding box of the AprilTag detection
	cv2.line(image, ptA, ptB, (0, 255, 0), 2)
	cv2.line(image, ptB, ptC, (0, 255, 0), 2)
	cv2.line(image, ptC, ptD, (0, 255, 0), 2)
	cv2.line(image, ptD, ptA, (0, 255, 0), 2)
	
# Saving the image
filename = "savedARtag2.jpg"
cv2.imwrite(filename, image)

# Function to send the nav_goal to move_base action server
def movebase_client():
    global AR_Tag_id
    
    # Create an action client for the move_base action server
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

	# Create a MoveBaseGoal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
	# Setting the position and orientation base on the detected tag
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

	# sending the goal
    client.send_goal(goal)
    wait = client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    
    
    rospy.init_node('movebase_client_py')
    result = movebase_client()
    if result:
        rospy.loginfo("Reached goal")


cv2.waitKey(0)