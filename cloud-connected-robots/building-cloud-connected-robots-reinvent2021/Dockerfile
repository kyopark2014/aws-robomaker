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