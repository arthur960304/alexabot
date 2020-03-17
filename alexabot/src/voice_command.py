#!/usr/bin/env python2.7
import rospy
import roslib
import json
import time
import numpy as np
import paho.mqtt.client as mqtt

from alexabot.srv import Command
from alexabot.srv import CommandRequest
from alexabot.srv import CommandResponse


COMMANDS_TOPIC   = "alexabot_commands"

MQTT_BROKER_URL  = "100.81.37.230"
MQTT_BROKER_PORT = 1883


def handle_message(client, msg):
    if msg.topic == COMMANDS_TOPIC:
        
        command_json = json.JSONDecoder().decode(str(msg.payload))
        location     = command_json["location"];
        
        rospy.wait_for_service("voice_commands")
        try:
            command_srv = rospy.ServiceProxy("voice_commands", Command)
            command_srv(location)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)
                            
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    if(rc == 0):
        print("connected OK, return code=0")
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe(COMMANDS_TOPIC)
    else:
        print("Bad connection, return code=", rc)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))
    handle_message(client, msg)


rospy.init_node('voice_command', anonymous=True)
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER_URL, MQTT_BROKER_PORT)

try:
    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    client.loop_forever()
except KeyboardInterrupt:
    # Be a good citizen
    client.disconnect()
