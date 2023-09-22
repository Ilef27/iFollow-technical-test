#!/usr/bin/env python3
import json
from pynput import keyboard
import paho.mqtt.client as mqtt

####### VARIABLES #######

#Maximum velocities
MaxLinearVelocity = 0.22
MaxAngularVelocity = 2.84

# Step value
LinearIncrement = 0.01
AngularIncrement = 0.1

# Initializing current velocities
CurrentLinearVelocity = 0.0
CurrentAngularVelocity = 0.0

# MQTT broker configuration
broker_address = "mqtt.eclipseprojects.io" 
port = 1883  
topic = "velocity_topic"  

# Screen output explaining keyboard controls
Explanation = """ Use your keyboard to teleoperate your BURGER Model Robot

{UP, DOWN} : to increase/decrease linear velocity
{LEFT, RIGHT} : to increase/decrease angular velocity
{ESC} : to put back both velocities to 0.0
Maximum Linear Velocity is 0.22
Maximum Angular Velocity is 2.84

"""
print (Explanation)

######### FUNCTIONS ########

# Function that publishes current velocities to MQTT broker
def publish_velocity():
    global CurrentLinearVelocity
    global CurrentAngularVelocity
    global client

    # dictionary to represent the Twist message
    Twist_Dict = {
        "linear": {
            "x": CurrentLinearVelocity,
            "y": 0.0,
            "z": 0.0,
        },
        "angular": {
            "x": 0.0,
            "y": 0.0,
            "z": CurrentAngularVelocity,
        },
    }

    # serialize the message to json 
    serialized_twist = json.dumps(Twist_Dict)

    # connecting to broker and publishing the velocity message
    client.connect(broker_address, port, keepalive=1)
    client.publish(topic, serialized_twist)
    print(f"Published message to topic '{topic}': {serialized_twist}")

    client.disconnect()

# Function to handle keyboard presses
def on_key_press(key):
    global CurrentLinearVelocity
    global CurrentAngularVelocity

    if (key == keyboard.Key.up) and (CurrentLinearVelocity < MaxLinearVelocity):
        CurrentLinearVelocity += LinearIncrement
        if CurrentLinearVelocity > MaxLinearVelocity:
            CurrentLinearVelocity = MaxLinearVelocity
        publish_velocity()

    elif (key == keyboard.Key.down) and (CurrentLinearVelocity > -MaxLinearVelocity):
        CurrentLinearVelocity -= LinearIncrement
        if CurrentLinearVelocity < -MaxLinearVelocity:
            CurrentLinearVelocity = -MaxLinearVelocity
        publish_velocity()

    elif (key == keyboard.Key.left) and (CurrentAngularVelocity < MaxAngularVelocity):
        CurrentAngularVelocity += AngularIncrement
        if CurrentAngularVelocity > MaxAngularVelocity:
            CurrentAngularVelocity = MaxAngularVelocity
        publish_velocity()

    elif key == keyboard.Key.right and CurrentAngularVelocity > -MaxAngularVelocity:
        CurrentAngularVelocity -= AngularIncrement
        if CurrentAngularVelocity < -MaxAngularVelocity:
            CurrentAngularVelocity = -MaxAngularVelocity
        publish_velocity()

    elif key == keyboard.Key.esc:
        CurrentLinearVelocity = 0.0
        CurrentAngularVelocity = 0.0
        publish_velocity()

# Function to handle keyboard release
def on_key_release(key):
    global CurrentLinearVelocity
    global CurrentAngularVelocity

    if (    key == keyboard.Key.up
            or key == keyboard.Key.down
            or key == keyboard.Key.left
            or key == keyboard.Key.right
            or key == keyboard.Key.esc):
        
        print('Linear Velocity = ', CurrentLinearVelocity, '   , Angular Velocity = ', CurrentAngularVelocity)

# Creating the MQTT Client
client = mqtt.Client()

# Launch the listener for keyboard events
with keyboard.Listener(on_press=on_key_press, on_release=on_key_release) as listener:
    listener.join()
