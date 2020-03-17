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
* Amazon Web Services - [Create a AWS account](https://aws.amazon.com/)


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

## Running the tests

### Mapping
Build a map in an unknown environment using keyboard teleoperation.

For detailed implementation, see this [link](http://edu.gaitech.hk/turtlebot/create-map-kenict.html).

### MQTT Protocol
Connect ROS with AWS using MQTT protocol.

1. Go through this [tutorial](https://aws.amazon.com/tw/blogs/iot/how-to-bridge-mosquitto-mqtt-broker-to-aws-iot/?fbclid=IwAR0JU9mTSQ0NK70NBHrmnJj0uvuTxwqqM1_K0rjpRAIczNOu5rWWAC2U2pg).

2. Some steps in above tutorial do not work in my situation, listed below:

* Replace `rootCA.pem` file with this [file](https://drive.google.com/open?id=1FhxFStaisveLkaAEWc8zXtjk6mqXiK_d).

* Replace `private.key` file with this [file](https://drive.google.com/open?id=1FdAdPanrGvpq6I5fc5N7RrJMfU8WP9pA).

* Replace `cert.crt` file with this [file](https://drive.google.com/open?id=1FhtGtgndtec-t8WMwhV4CpgElCcHuRJF).

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

* if the system shows the port is already open, try the following three commands
```
sudo services mosquitto stop
sudo lsof -t -i:1883
sudo kill -9 XXX
```

### Amazon skills



## Implementations

* See this [report](https://github.com/arthur960304/face-recognition-using-pca/blob/master/implementation.pdf) for detailed implementations.

## Results

* You can find the result on [Youtube](https://www.youtube.com).


## Authors

* **Arthur Hsieh** - *Initial work* - [arthur960304](https://github.com/arthur960304)
* **Henry Liu** - *Initial work* - [coldhenry](https://github.com/coldhenry)
