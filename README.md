# Alexabot

Alexabot is a SLAM mapping and autonomous navigation system built on the Turtlebot ros navigation stack. This package uses voice command based control, and delivers command messages via MQTT protocol.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

* Ubuntu - 14.04
* ROS - [Indigo](http://wiki.ros.org/indigo)
* MQTT - [1.6](https://mosquitto.org/download/)
* Turtlebot - [Turtlebot2](https://www.turtlebot.com/turtlebot2/)
* Amazon Echo Dot - [Buy on Amazon](https://www.amazon.com/Echo-Dot/dp/B07FZ8S74R)
* Amazon Web Services - [Create an AWS account](https://aws.amazon.com/)


## Code organization

    .
    ├── alexa_voice_model         # Alexa Skills scripts
    │   ├── lambda_function_call.py
    │   ├── lambda_function_info.py
    │   ├── lambda_function_template.py
    │   └── lambda_rate.py
    ├── alexabot                  # Turtlebot scripts
    │   ├── src                   # Source scripts folder
    │   │   ├── voice_command.py  # Subscribe topics from MQTT broker and publish ROS service
    │   │   └── controller.py     # Subscribe ROS service and publish navigation goal
    │   ├── srv                   # ROS service folder
    │   └── Command.srv           # command service
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── README.md

## Before running the tests

### Mapping
Build a map in an unknown environment using keyboard teleoperation.

For detailed implementation, see this [tutorial](http://edu.gaitech.hk/turtlebot/create-map-kenict.html).

### MQTT Protocol
Connect ROS with AWS using MQTT protocol.

1. Go through this [tutorial](https://aws.amazon.com/tw/blogs/iot/how-to-bridge-mosquitto-mqtt-broker-to-aws-iot/?fbclid=IwAR0JU9mTSQ0NK70NBHrmnJj0uvuTxwqqM1_K0rjpRAIczNOu5rWWAC2U2pg).

2. Some steps in the above tutorial do not work in my situation, listed below:

* Replace `rootCA.pem` file with this [rootCA.pem](https://drive.google.com/open?id=1FhxFStaisveLkaAEWc8zXtjk6mqXiK_d) file.

* Replace `private.key` file with this [private.pem.key](https://drive.google.com/open?id=1FdAdPanrGvpq6I5fc5N7RrJMfU8WP9pA) file.

* Replace `cert.crt` file with this [certificate.pem.crt](https://drive.google.com/open?id=1FhtGtgndtec-t8WMwhV4CpgElCcHuRJF) file.

* Modify the following lines in the configuratoin file `/etc/mosquitto/conf.d/bridge.conf`
```
address XXXXXXXXXX-ats.iot.eu-central-1.amazonaws.com:8883
-----
# Path to the PEM encoded client certificate
bridge_certfile /etc/mosquitto/certs/certificate.pem.crt

# Path to the PEM encoded client private key
bridge_keyfile /etc/mosquitto/certs/private.pem.key
```

* Instead of running mosquitto in the background, run this command to see the log message
`sudo mosquitto -c /etc/mosquitto/conf.d/bridge.conf`

* if the system shows the port 1883 is already in used, try the following commands
```
sudo lsof -t -i:1883
sudo kill -9 XXX
sudo services mosquitto stop
```

### Amazon skills


## Launch the test

1. Run the mosquitto 
`sudo mosquitto -c /etc/mosquitto/conf.d/bridge.conf`

2. Bring up the Turtlebot
`roslaunch turtlebot_bringup minimal.launch`

3. Launch the amcl
`roslaunch turtlebot_navigation amcl_demo.launch map_file:=/tmp/XXX.yaml`

4. Launch the Rviz
`roslaunch turtlebot_rviz_launchers view_navigation.launch`

5. Modify line 23 to your ip address and run `voice_command.py`
`python voice_command.py`

6. Run `controller.py`
``python controller.py


## Implementations

* See this [report](https://github.com/arthur960304/face-recognition-using-pca/blob/master/implementation.pdf) for detailed implementations.

## Results

* You can find the result on [Youtube](https://www.youtube.com).


## Authors

* **Arthur Hsieh** - *Initial work* - [arthur960304](https://github.com/arthur960304)
* **Henry Liu** - *Initial work* - [coldhenry](https://github.com/coldhenry)
