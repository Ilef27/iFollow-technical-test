#!/usr/bin/env python3
import rospy
import json
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist

# MQTT broker configuration
broker_address = "mqtt.eclipseprojects.io"
topic = "velocity_topic"    

# Callback functions for incoming MQTT messages 
def on_message(client, userdata, message):
    global json_data
    Twist_message = Twist()

    # Decode the binary message as utf-8 
    payload = message.payload.decode('utf-8')
    
    # Load the json message
    try:
        json_data = json.loads(payload)
        print("Received message:", json_data)
    except json.JSONDecodeError as e:
        print("Error decoding JSON:", e)

    # Retrieve Angular and Linear velocities from the json message 
    Twist_message.linear.x = json_data['linear']['x']
    Twist_message.linear.y = json_data['linear']['y']
    Twist_message.linear.z = json_data['linear']['z']
    Twist_message.angular.x = json_data['angular']['x']
    Twist_message.angular.y = json_data['angular']['y']
    Twist_message.angular.z = json_data['angular']['z']

    # Publish to  /cmd_web
    pub.publish(Twist_message)

# Initializing ROS node
rospy.init_node('MQTT_listener', anonymous=True)
# Initializing publisher to  cmd_web topic
pub = rospy.Publisher('cmd_web', Twist, queue_size=10)
rate = rospy.Rate(10) 

# create MQTT client and set the callback function
client = mqtt.Client()
client.on_message = on_message

# Connect to the MQTT broker, subscribe to the topic, and start the loop
client.connect(broker_address)
client.subscribe(topic)
client.loop_forever()

