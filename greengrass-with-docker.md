# Greengrass에 Docker로 ROS를 올리는 예제

[Blog - Deploy and Manage ROS Robots with AWS IoT Greengrass 2.0 and Docker](https://aws.amazon.com/ko/blogs/robotics/deploy-and-manage-ros-robots-with-aws-iot-greengrass-2-0-and-docker/)을 참조하여 예제를 분석합니다.

아래와 같이 코드 다운로드후에 [greengrass_bootstrap.template.yaml](https://github.com/aws-samples/greengrass-v2-docker-ros-demo/blob/main/greengrass/greengrass_bootstrap.template.yaml)에 이용해 ECR 및 IAM Role을 cloudFormation로 생성합니다. 

```java
git clone https://github.com/aws-samples/greengrass-v2-docker-ros-demo.git ~/greengrass-v2-docker-ros-demo
cd ~/greengrass-v2-docker-ros-demo
aws cloudformation create-stack --stack-name GG-Provisioning --template-body file://greengrass/greengrass_bootstrap.template.yaml --capabilities CAPABILITY_NAMED_IAM
```

Greengrass에 thing으로 등록하고 docker를 쓸수 있도록 group을 등록합니다. 

```java
export AWS_ACCESS_KEY_ID=<INSERT_YOUR_AWS_ACCESS_KEY_ID_HERE>
export AWS_SECRET_ACCESS_KEY=<INSERT_YOUR_AWS_SECRET_KEY>

curl -s https://d2s8p88vqu9w66.cloudfront.net/releases/greengrass-nucleus-latest.zip > greengrass-nucleus-latest.zip && unzip greengrass-nucleus-latest.zip -d GreengrassCore

sudo -E java -Droot="/greengrass/v2" -Dlog.store=FILE -jar ./GreengrassCore/lib/Greengrass.jar \
           --thing-name ROS2_Sample_Robot \
           --thing-group-name ROS2_Sample_Robots \
           --component-default-user ggc_user:ggc_group \
           --provision true \
           --setup-system-service true \
           --deploy-dev-tools true
           
 sudo usermod -aG docker ggc_user
 ```

[recipe.yaml](https://github.com/aws-samples/greengrass-v2-docker-ros-demo/blob/main/greengrass/com.example.ros2.demo/1.0.0/recipes/recipe.yaml), [docker-compose.yaml](https://github.com/aws-samples/greengrass-v2-docker-ros-demo/blob/main/docker-compose.yaml)을 이용해 component를 생성합니다. 





### Dockerfile

아래는 [Dockerfile](https://github.com/aws-samples/greengrass-v2-docker-ros-demo/blob/main/Dockerfile)입니다. 
```java
# Set main arguments.
ARG ROS_DISTRO=foxy
ARG LOCAL_WS_DIR=workspace

# ==== ROS Build Stages ====

# ==== Base ROS Build Image ====
FROM ros:${ROS_DISTRO}-ros-base AS build-base
LABEL component="com.example.ros2.demo"
LABEL build_step="ROSDemoNodes_Build"

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
RUN apt-get update && apt-get install python3-pip -y
RUN apt-get update && apt-get install ros-$ROS_DISTRO-example-interfaces
RUN python3 -m pip install awsiotsdk

# ==== Package 1: ROS Demos Talker/Listener ==== 
FROM build-base AS ros-demos-package
LABEL component="com.example.ros2.demo"
LABEL build_step="DemoNodesROSPackage_Build"

# Clone the demos_ros_cpp package from within the ROS Demos monorepo.
RUN mkdir -p /ws/src
WORKDIR /ws
RUN git clone https://github.com/ros2/demos.git \
    -b $ROS_DISTRO \
    --no-checkout \
    --depth 1 \
    --filter=blob:none \
    src/demos
    
RUN cd src/demos && \
    git sparse-checkout set demo_nodes_cpp
    
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --build-base workspace/build --install-base /opt/ros_demos

# ==== Package 2: Greengrass Bridge Node ==== 
FROM build-base AS greengrass-bridge-package
LABEL component="com.example.ros2.demo"
LABEL build_step="GreengrassBridgeROSPackage_Build"
ARG LOCAL_WS_DIR

COPY ${LOCAL_WS_DIR}/src /ws/src
WORKDIR /ws

# Cache the colcon build directory.
RUN --mount=type=cache,target=${LOCAL_WS_DIR}/build:/ws/build \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
     --install-base /opt/greengrass_bridge

# ==== ROS Runtime Image (with the two packages) ====
FROM build-base AS runtime-image
LABEL component="com.example.ros2.demo"

COPY --from=ros-demos-package /opt/ros_demos /opt/ros_demos
COPY --from=greengrass-bridge-package /opt/greengrass_bridge /opt/greengrass_bridge

# Add the application source file to the entrypoint.
WORKDIR /
COPY app_entrypoint.sh /app_entrypoint.sh
RUN chmod +x /app_entrypoint.sh
ENTRYPOINT ["/app_entrypoint.sh"]
```

### app_entrypoint.sh

[app_entrypoint.sh](https://github.com/aws-samples/greengrass-v2-docker-ros-demo/edit/main/app_entrypoint.sh)은 아래와 같습니다. 

```java
#!/bin/bash
set -e

# setup ros2 environment
source "/opt/greengrass_bridge/setup.bash"
source "/opt/ros_demos/setup.bash"
exec "$@"
```

### docker-compose.yaml

[docker-compose.yaml](https://github.com/aws-samples/greengrass-v2-docker-ros-demo/blob/main/docker-compose.yaml)은 아래와 같습니다. 

```java
version: "3"
services: 

  greengrass_demo_image:
    build: 
        context: ./
    image: ros-foxy-greengrass-demo:latest
    
  talker:
    image: ros-foxy-greengrass-demo:latest
    command: ros2 run demo_nodes_cpp talker
      
  listener:
    image: ros-foxy-greengrass-demo:latest
    command: ros2 run demo_nodes_cpp listener

  greengrass_bridge:
    image: ros-foxy-greengrass-demo:latest
    command: ros2 launch greengrass_bridge greengrass_bridge.launch.py ros_topics:="['chatter']" iot_topics:="['cloud_chatter']"
    environment:
      - AWS_REGION
      - SVCUID
      - AWS_GG_NUCLEUS_DOMAIN_SOCKET_FILEPATH_FOR_COMPONENT
      - AWS_CONTAINER_AUTHORIZATION_TOKEN
      - AWS_CONTAINER_CREDENTIALS_FULL_URI
    volumes: 
      - "/greengrass/v2/ipc.socket:/greengrass/v2/ipc.socket"
```      



### greengrass_bootstrap.template.yaml

[greengrass_bootstrap.template.yaml](https://github.com/aws-samples/greengrass-v2-docker-ros-demo/blob/main/greengrass/greengrass_bootstrap.template.yaml)은 아래와 같이 ECR, IAM Role을 설정합니다. 

```java
AWSTemplateFormatVersion: '2010-09-09'
Metadata:
  License: Apache-2.0
Description: 'Sample cloudformation template that creates default Greengrass provisioning resources.'
Resources:
  GreengrassProvisioningUser:
    Type: AWS::IAM::User
  S3Bucket:
    Type: AWS::S3::Bucket
    Properties:
      BucketEncryption:
        ServerSideEncryptionConfiguration:
          - ServerSideEncryptionByDefault:
              SSEAlgorithm: AES256
      VersioningConfiguration:
        Status: Enabled
  GreengrassProvisioningUserGroup:
    Type: AWS::IAM::Group
  Users:
    Type: AWS::IAM::UserToGroupAddition
    Properties:
      GroupName: !Ref 'GreengrassProvisioningUserGroup'
      Users: [!Ref 'GreengrassProvisioningUser']
  GreengrassTokenExchangeRole:
    Type: AWS::IAM::Role
    Properties:
      RoleName: "GreengrassV2TokenExchangeRole"
      AssumeRolePolicyDocument:
        Version: "2012-10-17"
        Statement:
          - Effect: Allow
            Principal:
              Service:
                - credentials.iot.amazonaws.com
            Action:
              - 'sts:AssumeRole'
  RobotPolicy:
    Type: AWS::IAM::Policy
    Properties:
      PolicyName: RobotPolicy
      PolicyDocument:
        Statement:
          - Effect: Allow
            Action: [ "s3:ListBucket" ]
            Resource: !Join ["",[ "arn:aws:s3:::", !Ref S3Bucket ] ]
          - Effect: Allow
            Action: [ 
                "s3:PutObject",
                "s3:GetObject",
                "s3:DeleteObject"
                    ]
            Resource: !Join ["",["arn:aws:s3:::", !Ref S3Bucket,"/*"] ]
          - Effect: Allow
            Action: [ 
                  "ecr:GetAuthorizationToken",
                  "ecr:BatchGetImage",
                  "ecr:GetDownloadUrlForLayer"
                    ]
            Resource: '*'
          - Effect: Allow
            Action: [ "greengrass:" ]
            Resource: '*'
          - Effect: Allow
            Action: [ "iot:" ]
            Resource: '*'
      Roles:  [ !Ref 'GreengrassTokenExchangeRole' ]
  GreengrassTokenPolicy:
    Type: AWS::IAM::Policy
    Properties:
      PolicyName: GreengrassTokenExchangePolicy
      PolicyDocument:
        Statement:
          - Effect: Allow
            Action: [ 
                    "iot:DescribeCertificate",
                    "logs:CreateLogGroup",
                    "logs:CreateLogStream",
                    "logs:PutLogEvents",
                    "logs:DescribeLogStreams",
                    "iot:Connect",
                    "iot:Publish",
                    "iot:Subscribe",
                    "iot:Receive",
                    "s3:GetBucketLocation"
                    ]
            Resource: '*'
      Roles:  [ !Ref 'GreengrassTokenExchangeRole' ]
  GreengrassProvisioningUserPolicies:
    Type: AWS::IAM::Policy
    Properties:
      PolicyName: GreengrassProvisioningUsers
      PolicyDocument:
        Statement:
          - Effect: Allow
            Action: [ 
                      "iot:AddThingToThingGroup",
                      "iot:AttachPolicy",
                      "iot:AttachThingPrincipal",
                      "iot:CreateKeysAndCertificate",
                      "iot:CreatePolicy",
                      "iot:CreateRoleAlias",
                      "iot:CreateThing",
                      "iot:CreateThingGroup",
                      "iot:DescribeEndpoint",
                      "iot:DescribeRoleAlias",
                      "iot:DescribeThingGroup",
                      "iot:GetPolicy",
                      "iam:GetRole",
                      "iam:CreateRole",
                      "iam:PassRole",
                      "iam:CreatePolicy",
                      "iam:AttachRolePolicy",
                      "iam:GetPolicy",
                      "sts:GetCallerIdentity"
                    ]
            Resource: '*'
          - Effect: Allow
            Action: [ 
                      "greengrass:CreateDeployment",
                      "iot:CancelJob",
                      "iot:CreateJob",
                      "iot:DeleteThingShadow",
                      "iot:DescribeJob",
                      "iot:DescribeThing",
                      "iot:DescribeThingGroup",
                      "iot:GetThingShadow",
                      "iot:UpdateJob",
                      "iot:UpdateThingShadow"
                    ]
            Resource: '*'
      Groups: [!Ref 'GreengrassProvisioningUserGroup']
    
Outputs:
  UserName:
    Value: !Ref GreengrassProvisioningUser
    Description: User name for the provisioning user.
  S3BucketName:
    Value: !Ref S3Bucket
    Description: S3 bucket to upload assets to. 
```    


### recipe.yaml

[recipe.yaml](https://github.com/aws-samples/greengrass-v2-docker-ros-demo/blob/main/greengrass/com.example.ros2.demo/1.0.0/recipes/recipe.yaml)은 아래와 같습니다. 

```java
---
RecipeFormatVersion: '2020-01-25'
ComponentName: com.example.ros2.demo
ComponentVersion: '1.0.0'
ComponentDescription: 'A basic component that runs a simple pub/sub ROS2 application'
ComponentPublisher: Amazon
ComponentDependencies:
  aws.greengrass.DockerApplicationManager:
    VersionRequirement: ~2.0.0
  aws.greengrass.TokenExchangeService:
    VersionRequirement: ~2.0.0
ComponentConfiguration:
  DefaultConfiguration:
    accessControl:
      aws.greengrass.ipc.mqttproxy:
        com.example.PubSubPublisher:pubsub:1:
          policyDescription: "Allows access to publish and subscribe to MQTT topics."
          operations:
          - "aws.greengrass#PublishToIoTCore"
          - "aws.greengrass#SubscribeToIoTCore"
          resources:
          - "chatter"
          - "cloud_chatter"
Manifests:
  - Platform:
      os: all
    Lifecycle:
        Install: |
           docker tag <YOUR_PRIVATE_ECR_IMAGE_ID_ROS_GREENGRASS_DEMO> ros-foxy-greengrass-demo:latest
        Startup: |
           docker-compose -f {artifacts:path}/docker-compose.yaml up -d
        Shutdown: |
           docker-compose -f {artifacts:path}/docker-compose.yaml down
    Artifacts:
      - URI: "docker:<YOUR_PRIVATE_ECR_IMAGE_ID_ROS_GREENGRASS_DEMO>"
      - URI: "s3://<YOUR_BUCKET_NAME>/com.example.ros2.demo/1.0.0/artifacts/docker-compose.yaml"
```

## Reference

[Blog - Deploy and Manage ROS Robots with AWS IoT Greengrass 2.0 and Docker](https://aws.amazon.com/ko/blogs/robotics/deploy-and-manage-ros-robots-with-aws-iot-greengrass-2-0-and-docker/)

[Github - ROS2 Docker Sample Application with AWS IoT Greengrass 2.0](https://github.com/aws-samples/greengrass-v2-docker-ros-demo)
