# Greengrass에 Docker로 ROS를 올리는 예제

[Blog - Deploy and Manage ROS Robots with AWS IoT Greengrass 2.0 and Docker](https://aws.amazon.com/ko/blogs/robotics/deploy-and-manage-ros-robots-with-aws-iot-greengrass-2-0-and-docker/)을 참조하여 예제를 분석합니다.

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

## Reference

[Blog - Deploy and Manage ROS Robots with AWS IoT Greengrass 2.0 and Docker](https://aws.amazon.com/ko/blogs/robotics/deploy-and-manage-ros-robots-with-aws-iot-greengrass-2-0-and-docker/)

[Github - ROS2 Docker Sample Application with AWS IoT Greengrass 2.0](https://github.com/aws-samples/greengrass-v2-docker-ros-demo)
