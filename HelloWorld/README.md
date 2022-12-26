# Hello World

[Running a sample application with ROS 2 Foxy and Gazebo 11](https://docs.aws.amazon.com/robomaker/latest/dg/run-hello-world-ros-2.html)을 참조하여 "Hello World"를 구현하고자 합니다. 


## RUN

Robo Application

```java
docker run -it -e DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix/ --name robot_app \
-u robomaker -e ROBOMAKER_GAZEBO_MASTER_URI=http://localhost:5555 \
-e ROBOMAKER_ROS_MASTER_URI=http://localhost:11311 \
helloworldsampleappros2foxygazebo11robotapp:latest  
```

Robo Simulation

```java
docker run -it -e DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix/ --name sim_app \
-u robomaker -e ROBOMAKER_GAZEBO_MASTER_URI=http://localhost:5555 \
-e ROBOMAKER_ROS_MASTER_URI=http://localhost:11311 \
helloworldsampleappros2foxygazebo11simapp:latest
```

## Reference 

[aws-robomaker-sample-application-helloworld](https://github.com/aws-robotics/aws-robomaker-sample-application-helloworld/archive/3527834.zip)



