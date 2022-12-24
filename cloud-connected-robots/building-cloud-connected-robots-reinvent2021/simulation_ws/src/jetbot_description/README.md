# robomaker-jetbot

![Figure 1](https://github.com/cbuscaron/robomaker-jetbot/blob/main/robomaker-jetbot-robot-model-2.jpg)

![Figure 1](https://github.com/cbuscaron/robomaker-jetbot/blob/main/robomaker-jetbot-lidar-on.jpg)

## Clone into your existing local workspace

1. Clone and rebuild

```shell
git clone https://github.com/cbuscaron/robomaker-jetbot.git
catkin_make
```

2. Source builds

```shell
source /opt/ros/<ros-distro>/setup.bash
source /usr/share/gazebo/setup.sh
```

3. Launch

```shell
roslaunch robomaker-jetbot world.launch
```

## Creating a new isolated local workspace

1. Create a workspace. In this case, let’s call it ~/simulation_ws

```shell
mkdir ~/simulation_ws/src -p
```

2. Clone repository

```shell
cd ~/simulation_ws/src
git clone https://github.com/cbuscaron/robomaker-jetbot.git
```

3.  Now let’s compile our newly created workspace

```shell
cd ~/simulation_ws/
catkin_make
```

4. Source builds

```shell
source /opt/ros/<ros-distro>/setup.bash
source /usr/share/gazebo/setup.sh
```

5. Launch  

```shell
roslaunch robomaker-jetbot world.launch
```
