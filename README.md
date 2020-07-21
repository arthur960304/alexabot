# Alexabot

The target of Alexabot is to receive the command from humans, extract the useful information from human's response such as the destination, and move to the assigned position. It is an autonomous navigation system built on the Turtlebot ros navigation stack. This package uses voice-based control, processing verbal sequences with Alexa skills, and delivering commands via MQTT protocol.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Hardware & Software Prerequisites

* Ubuntu - 14.04
* ROS - [Indigo](http://wiki.ros.org/indigo)
* MQTT - [1.6](https://mosquitto.org/download/)
* Turtlebot - [Turtlebot2](https://www.turtlebot.com/turtlebot2/)
* Amazon Echo Dot - [Buy on Amazon](https://www.amazon.com/Echo-Dot/dp/B07FZ8S74R)
* Amazon Web Services - [Create an AWS account](https://aws.amazon.com/)

### Experimental Process

<img src="https://github.com/arthur960304/alexabot/blob/master/pic/steps.jpeg" alt="layout" width="800" />

### Mapping
Build a map in an unknown environment using keyboard teleoperation.

For detailed implementation, see this [tutorial](http://edu.gaitech.hk/turtlebot/create-map-kenict.html).

### MQTT Connection
Connect ROS with AWS using MQTT protocol.

<img src="https://github.com/arthur960304/alexabot/blob/master/pic/MQTT.jpeg" alt="layout" width="800" />

1. Go through this [tutorial](https://aws.amazon.com/tw/blogs/iot/how-to-bridge-mosquitto-mqtt-broker-to-aws-iot/?fbclid=IwAR0JU9mTSQ0NK70NBHrmnJj0uvuTxwqqM1_K0rjpRAIczNOu5rWWAC2U2pg).

2. Some steps in the above tutorial do not work in my situation, listed below:

* Replace `rootCA.pem` file with this [rootCA.pem](https://drive.google.com/open?id=1FhxFStaisveLkaAEWc8zXtjk6mqXiK_d) file.

* Replace `private.key` file with this [private.pem.key](https://drive.google.com/open?id=1FdAdPanrGvpq6I5fc5N7RrJMfU8WP9pA) file.

* Replace `cert.crt` file with this [certificate.pem.crt](https://drive.google.com/open?id=1FhtGtgndtec-t8WMwhV4CpgElCcHuRJF) file.

* Modify the following lines in the configuratoin file `/etc/mosquitto/conf.d/bridge.conf`.
```
address XXXXXXXXXX-ats.iot.eu-central-1.amazonaws.com:8883
-----
# Path to the PEM encoded client certificate
bridge_certfile /etc/mosquitto/certs/certificate.pem.crt

# Path to the PEM encoded client private key
bridge_keyfile /etc/mosquitto/certs/private.pem.key
```

* Instead of running mosquitto in the background, run this command to see the log messages.
`sudo mosquitto -c /etc/mosquitto/conf.d/bridge.conf`

* if the system shows the port 1883 is already in used, try the following commands.
```
sudo lsof -t -i:1883
sudo kill -9 XXX
sudo services mosquitto stop
```

### Amazon skills

Use the Alexa skills console to build a skill for Alexa.

1. Read this [tutorial](https://developer.amazon.com/en-US/alexa/alexa-skills-kit/get-deeper/tutorials-code-samples/build-an-engaging-alexa-skill) and go through all steps in it.

2. Since that we need a place to store variables, it requires the Amazon S3, the service in AWS. Therefore, we need an AWS account to access these functions. For more details, visit [here](https://aws.amazon.com/tw/console/).

3. To access the variables we stored, we use the AWS Lambda function to connect with the Alexa skill. Look [here](https://developer.amazon.com/en-US/docs/alexa/custom-skills/host-a-custom-skill-as-an-aws-lambda-function.html) for more details.

## Code organization

    .
    ├── alexa_voice_model         # Alexa Skills scripts
    │   ├── lambda_function_call.py
    │   ├── lambda_function_info.py
    │   ├── lambda_function_template.py
    │   └── lambda_rate.py
    ├── alexabot                  # Turtlebot scripts
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── src                   # Source scripts folder
    │   │   ├── voice_command.py  # Subscribe topics from MQTT broker and publish ROS service
    │   │   └── controller.py     # Subscribe ROS service and publish navigation goal
    │   ├── srv                   # ROS service folder
    │   └── Command.srv           # command service
    └── README.md

## Running the test

1. Run the mosquitto 
```
sudo mosquitto -c /etc/mosquitto/conf.d/bridge.conf
```

2. Bring up the Turtlebot
```
roslaunch turtlebot_bringup minimal.launch
```

3. Launch the amcl and specify your map file
```
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/tmp/XXX.yaml
```

4. Launch the Rviz and set the initial robot pose
```
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

5. Modify line 23 to your ip address and run `voice_command.py`
```
python voice_command.py
```

6. Run `controller.py`, you may change the xy-coordinate of your interested points in the file
```
python controller.py
```

## Results

* You can find the result on [Youtube](https://youtu.be/9mKWSi7dT5E?fbclid=IwAR1GBm62cn_r-mycmawu5Epz997WsuICPtV1ql0WKWZnioI1zPTcK6F5o5w).


## Authors

* **Arthur Hsieh** - *Initial work* - [arthur960304](https://github.com/arthur960304)
* **Henry Liu** - *Initial work* - [coldhenry](https://github.com/coldhenry)
