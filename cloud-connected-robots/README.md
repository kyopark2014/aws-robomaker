# Cloud Connected Robots

[Workshop - Building Cloud Connected Robots](https://catalog.us-east-1.prod.workshops.aws/workshops/fa208b8e-83d6-4cc1-8356-bfa5b6184fae/en-US)에 따라 Robot을 Robot을 RoboMaker에 연결하는것을 확인합니다. 

코드 다운로드를 합니다.

```java
wget https://ee-assets-prod-us-east-1.s3.us-east-1.amazonaws.com/modules/4301048177d549d3a52333b51ac7e8f7/v2/workshop-assets.tar.gz
tar -xf workshop-assets.tar.gz
```

## Local에서 실행하기 

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

## Greengrass에서 Docker 컨테이너로 실행하기 

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

Docker 이미지 다운로드후에 ECR에 업로드 합니다. 

```java
wget https://ee-assets-prod-us-east-1.s3.us-east-1.amazonaws.com/modules/4301048177d549d3a52333b51ac7e8f7/v2/jetbot-dance-latest.tar.gz
docker load < jetbot-dance-latest.tar.gz
```

[docker-compose.yaml](https://github.com/kyopark2014/aws-robomaker/blob/main/cloud-connected-robots/building-cloud-connected-robots-reinvent2021/docker-compose.yaml)와 다른 파일들을 업로드합니다.
 
```java 
aws s3 cp docker-compose.yaml s3://[YOUR_S3_BUCKET_NAME]/artifacts/docker-compose.yaml

aws s3 sync ./routines s3://[YOUR_S3_BUCKET_NAME]/artifacts/routines
```

[recipe.yaml](https://github.com/kyopark2014/aws-robomaker/blob/main/cloud-connected-robots/building-cloud-connected-robots-reinvent2021/greengrass/recipe.yaml)으로 deploy합니다.

```java
aws greengrassv2 create-component-version \
    --inline-recipe fileb://greengrass/recipe.yaml
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

## recipe.yaml
[recipe.yaml](https://github.com/kyopark2014/aws-robomaker/blob/main/cloud-connected-robots/building-cloud-connected-robots-reinvent2021/greengrass/recipe.yaml)은 아래와 같습니다.

```java
---
RecipeFormatVersion: '2020-01-25'
ComponentName: com.example.ros2.jetbot.dance
ComponentVersion: '1.0.0'
ComponentDescription: 'The ROS2 JetBot Dance Application'
ComponentPublisher: Amazon
ComponentDependencies:
  aws.greengrass.DockerApplicationManager:
    VersionRequirement: ">=2.0.0 <2.1.0"
  aws.greengrass.TokenExchangeService:
    VersionRequirement: ">=2.0.0 <2.1.0"
ComponentConfiguration:
  DefaultConfiguration:
    auto_start: True
    loaded_routine: 'autumn.json'
    accessControl:
      aws.greengrass.ipc.mqttproxy:
        com.example.PubSubPublisher:pubsub:1:
          policyDescription: "Allows access to publish and subscribe to MQTT topics."
          operations:
          - "aws.greengrass#PublishToIoTCore"
          - "aws.greengrass#SubscribeToIoTCore"
          resources:
          - "chatter"
          - "/jetbot/dance/demo"
          - "/jetbot/dance/start"
Manifests:
  - Platform:
      os: all
    Lifecycle:
        Bootstrap:
          RequiresPrivilege: True
          Script: |
            echo "Bootstrapping the dance application! as root This runs only once during the deployment."
            mkdir -p /home/ggc_user/routines/
            cp {artifacts:path}/*.json /home/ggc_user/routines/
            chown -R ggc_user:ggc_group /home/ggc_user/routines/
            cat << EOF > {artifacts:path}/.env
            LOADED_ROUTINE={configuration:/loaded_routine}
            AUTO_START={configuration:/auto_start}
            SVCUID=$SVCUID
            AWS_GG_NUCLEUS_DOMAIN_SOCKET_FILEPATH_FOR_COMPONENT=$AWS_GG_NUCLEUS_DOMAIN_SOCKET_FILEPATH_FOR_COMPONENT
            EOF
            chown ggc_user:ggc_group {artifacts:path}/.env
        Install:
          Script: |
            echo "Installing the dance application! This will run everytime the Greengrass core software is started."
        Run:
          SetEnv:
            - HOME_ROUTINES: "/home/ubuntu/routines"
          Script: |
            echo "Running the dance application! This is the main application execution script."
            docker-compose -f {artifacts:path}/docker-compose.yaml up
        Shutdown: |
            echo "Shutting down the dance application! This will run each time the component is stopped."
    Artifacts:
      - URI: "docker:[YOUR_REPOSITORY_URI]:latest"
      - URI: "s3://[YOUR_S3_BUCKET_NAME]/artifacts/docker-compose.yaml"
      - URI: "s3://[YOUR_S3_BUCKET_NAME]/artifacts/routines/autumn.json"
      - URI: "s3://[YOUR_S3_BUCKET_NAME]/artifacts/routines/[YOUR_DANCE_ROUTINE_FILE]"
```      

### autumn.json

[autumn.json](https://github.com/kyopark2014/aws-robomaker/blob/main/cloud-connected-robots/building-cloud-connected-robots-reinvent2021/routines/autumn.json)은 아래와 같습니다. 

```java
{
    "name": "AutumnDanceRoutine",
    "songName": "Violin Concerto in F major, RV 293 'Autumn'",
    "artist": "Antonio Vivaldi",
    "audioURL": "https://musopen-files.s3-us-west-2.amazonaws.com/recordings/7c8b33fc-3fa1-4186-baf7-b6fc85a6cdf6.mp3",
    "dancers": {
        "lead": {
            "startPosition": "0 0 0 0 0 0",
            "routine": {
                "1": "left",
                "2": "right",
                "3": "left",
                "4": "right",
                "5": "left",
                "6": "right",
                "7": "left",
                "8": "right",
                "9": "left",
                "10": "right",
                "11": "left",
                "12": "forward",
                "13": "backward",
                "14": "forward",
                "15": "backward",
                "16": "forward",
                "17": "backward",
                "18": "forward",
                "19": "backward",
                "20": "forward",
                "21": "backward",
                "22": "forward",
                "23": "backward",
                "24": "forward",
                "36": "right",
                "46": "left",
                "56": "forward",
                "66": "right",
                "76": "left",
                "86": "end"
            }
        }
    }
}
```


### docker-compose.yaml

[docker-compose.yaml](https://github.com/kyopark2014/aws-robomaker/blob/main/cloud-connected-robots/building-cloud-connected-robots-reinvent2021/docker-compose.yaml)은 아래와 같습니다. 

```java
version: "3"
services: 

  dance_demo:
    build: 
        context: ./
    image: [YOUR_REPOSITORY_URI]:latest
 
  jetbot_base:
    image: [YOUR_REPOSITORY_URI]:latest
    command: ros2 launch jetbot_base jetbot.launch.py
    devices:
      - "/dev/i2c-1:/dev/i2c-1"
      - "/dev/video0:/dev/video0"
    volumes:
      - /tmp/argus_socket:/tmp/argus_socket
    environment:
      - DEFAULT_SPEED=0.3
      - QT_X11_NO_MITSHM=1
      - NVIDIA_DRIVER_CAPABILITIES=all
      - CAMERA=false
      - ROS_DOMAIN_ID=0
      - WLAN0_IP="192.168.0.179"
      - ETH0_IP="192.168.0.179"
  
  dance:
    image: [YOUR_REPOSITORY_URI]:latest
    command: ros2 launch jetbot_move dance.launch.py
    environment:
      - DANCE_ROUTINE_PATH:/home/ubuntu/routines/$LOADED_ROUTINE
      - AUTO_START
    volumes: 
      - "/home/ggc_user/routines:/home/ubuntu/routines"
    
  greengrass_bridge:
    image: [YOUR_REPOSITORY_URI]:latest
    command: "ros2 launch greengrass_bridge greengrass_bridge.launch.py ros_topics:=\"['chatter']\" iot_topics:=\"['/jetbot/dance/start', '/jetbot/dance/demo']\""
    environment:
      - AWS_REGION
      - AWS_IOT_THING_NAME
      - SVCUID
      - AWS_GG_NUCLEUS_DOMAIN_SOCKET_FILEPATH_FOR_COMPONENT
      - AWS_CONTAINER_AUTHORIZATION_TOKEN
      - AWS_CONTAINER_CREDENTIALS_FULL_URI
    volumes: 
      - $AWS_GG_NUCLEUS_DOMAIN_SOCKET_FILEPATH_FOR_COMPONENT:$AWS_GG_NUCLEUS_DOMAIN_SOCKET_FILEPATH_FOR_COMPONENT
```      

## Reference

[Building Cloud Connected Robots](https://catalog.us-east-1.prod.workshops.aws/workshops/fa208b8e-83d6-4cc1-8356-bfa5b6184fae/en-US)
