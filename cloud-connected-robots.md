# Cloud Connected Robots

[Workshop - Building Cloud Connected Robots](https://catalog.us-east-1.prod.workshops.aws/workshops/fa208b8e-83d6-4cc1-8356-bfa5b6184fae/en-US)에 따라 Robot을 Robot을 RoboMaker에 연결하는것을 확인합니다. 

코드 다운로드를 합니다.

```java
wget https://ee-assets-prod-us-east-1.s3.us-east-1.amazonaws.com/modules/4301048177d549d3a52333b51ac7e8f7/v2/workshop-assets.tar.gz
tar -xf workshop-assets.tar.gz
```

## Console

아래와 같이 빌드를 합니다. 

```java
sudo apt-get update
cd ~/environment/building-cloud-connected-robots-reinvent2021/robot_ws
rosdep install --from src -i -y
colcon build

cd ~/environment/building-cloud-connected-robots-reinvent2021/simulation_ws
rosdep install --from src -i -y
colcon build
```

```java
export DISPLAY=:0
cd ~/environment/building-cloud-connected-robots-reinvent2021/simulation_ws
source install/setup.sh
```

Gazebo를 실행합니다. 

```java
ros2 launch jetbot_dance_simulation spawn_jetbot.launch.py
```

```java
cd ~/environment/building-cloud-connected-robots-reinvent2021/robot_ws
source install/setup.sh

export DANCE_ROUTINE_PATH=/home/ubuntu/environment/building-cloud-connected-robots-reinvent2021/routines/autumn.json
export ROBOT_ID=jetbot
ros2 launch jetbot_move dance.launch.py
```

메시지 보내기를 합니다.

```java
ros2 topic list

ros2 topic pub /jetbot/dance/demo std_msgs/String "data: autumn"
```

Update를 할 수 있습니다.

```java
cd ~/environment/building-cloud-connected-robots-reinvent2021/simulation_ws
source install/setup.sh
ros2 launch jetbot_dance_simulation spawn_jetbot.launch.py

export DANCE_ROUTINE_PATH=/home/ubuntu/environment/building-cloud-connected-robots-reinvent2021/routines/polka.json
export ROBOT_ID=jetbot
cd ~/environment/building-cloud-connected-robots-reinvent2021/robot_ws
source install/setup.sh
ros2 launch jetbot_move dance.launch.py
```

## Docker

Greengrass 환경을 설정합니다. 

```java
curl -s https://d2s8p88vqu9w66.cloudfront.net/releases/greengrass-nucleus-latest.zip > greengrass-nucleus-latest.zip && unzip greengrass-nucleus-latest.zip -d GreengrassCore

sudo -E java -Droot="/greengrass/v2" -Dlog.store=FILE -jar ./GreengrassCore/lib/Greengrass.jar --thing-name jetbot-dancer --thing-group-name dancers --component-default-user ggc_user:ggc_group --provision true --setup-system-service true --deploy-dev-tools true --aws-region us-east-1

sudo usermod -aG docker ggc_user
sudo usermod -aG i2c ggc_user
sudo usermod -aG video ggc_user

sudo apt-get update
sudo apt-get install libhdf5-dev libssl-dev python3 python3-pip -y
export DOCKER_COMPOSE_VERSION=1.27.4 
sudo pip3 install docker-compose=="${DOCKER_COMPOSE_VERSION}"
pip install docker-compose
```

CloudFront로 인프라를 설치합니다. 

```java
cd ~/environment/building-cloud-connected-robots-reinvent2021

aws cloudformation describe-stacks --stack-name mod-4301048177d549d3
```



### Dockerfile

```java
# Distribution of ROS2
ARG ROS_DISTRO=foxy
ARG BASE_IMAGE=dustynv/ros:${ROS_DISTRO}-ros-base-l4t-r32.6.1

FROM ${BASE_IMAGE} as base
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root

RUN apt-get update && apt-get install python3 python3-pip -y
RUN pip3 install wheel 
RUN pip3 install Adafruit-MotorHAT Adafruit_SSD1306 Cython
RUN LANG=C.UTF-8 pip3 install awsiotsdk

RUN sudo apt-key adv --fetch-key http://repo.download.nvidia.com/jetson/jetson-ota-public.asc
RUN apt-get update

RUN apt-get update && apt-get install -y libboost-python-dev libboost-dev libssl-dev \
          && apt-get install -y --no-install-recommends \
          python3-pip \
          python3-dev \
          python3-apt \
          build-essential \
          python3-tornado \
          python3-twisted \
          python3-pil \
          python3-bson \
          python3-opencv \
          zlib1g-dev \
          zip \
          libjpeg8-dev && rm -rf /var/lib/apt/lists/*

FROM base AS build_step

ADD robot_ws/ /root/ros2_ws/
WORKDIR /root/ros2_ws/

RUN mkdir /opt/jetbot_app
RUN . /opt/ros/${ROS_DISTRO}/install/setup.bash && colcon build --install-base /opt/jetbot_app

FROM base AS runtime_image

COPY --from=build_step /opt/jetbot_app /opt/jetbot_app
WORKDIR /
ADD app_entrypoint.sh /app_entrypoint.sh
RUN chmod +x /app_entrypoint.sh
ENTRYPOINT ["./app_entrypoint.sh"]
```

### app_entrypoint.sh

app_entrypoint.sh은 아래와 같습니다. 

```java
set -e

# setup ros2 environment
source "/opt/jetbot_app/setup.bash"
exec "$@"
```

## Reference

[Building Cloud Connected Robots](https://catalog.us-east-1.prod.workshops.aws/workshops/fa208b8e-83d6-4cc1-8356-bfa5b6184fae/en-US)
