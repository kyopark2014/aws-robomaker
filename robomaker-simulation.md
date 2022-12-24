# Robotics Simulations

코드 다운로드 및 Docker Build를 아래와 같이 수행합니다. 

```java
git clone -b noetic-husky-mqtt-dev https://github.com/aws-samples/multi-robot-fleet-sample-application.git
cd ~/environment/multi-robot-fleet-sample-application/simulation_ws/
docker build -t robot_fleet:latest ./
```

- Workshop: [Using AWS to Run Robotics Simulations](https://catalog.us-east-1.prod.workshops.aws/workshops/5b369b7a-2da1-498b-97a9-9af95e3c6294/en-US)

- Github: [Robot fleet simulation using concurrent gazebo instances](https://github.com/aws-samples/multi-robot-fleet-sample-application)

이때의 Dockerfile은 아래와 같습니다. 

```java
FROM ros:noetic

# install tools
RUN apt update && apt install -y git python3-pip

# clone the main application
RUN git clone -b noetic-husky-mqtt-dev https://github.com/aws-samples/multi-robot-fleet-sample-application.git

# install app dependencies
WORKDIR /multi-robot-fleet-sample-application/simulation_ws
RUN rosws update
RUN rosdep install --from-paths src -i -r -y

# build app
WORKDIR /multi-robot-fleet-sample-application/simulation_ws
RUN . /opt/ros/noetic/setup.sh && catkin_make

# install AWS IOT SDK
RUN python3 -m pip install AWSIoTPythonSDK

# install xterm for interactive use
RUN apt install -y xterm

# copy certs and iot config file
WORKDIR /multi-robot-fleet-sample-application/simulation_ws
COPY src/velocity_mqtt_manager/certs src/velocity_mqtt_manager/certs 
COPY src/velocity_mqtt_manager/config/* src/velocity_mqtt_manager/config

# setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
```
