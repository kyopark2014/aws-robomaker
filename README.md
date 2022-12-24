# AWS RoboMaker


AWS의 로보메이커(RoboMaker)는 클라우드 기반의 로봇 시뮬레이션 서비스입니다. 로봇 개발자들이 로봇을 개발하고 시험하고 Cloud를 이용해 배포할수 있도록 해줍니다. 또한 다수의 로봇을 실행하고 자동화 할 수 있습니다. 

아래는 AWS의 RoboMaker의 Architecture를 보여줍니다. 

<img src="https://user-images.githubusercontent.com/52392004/209247633-304c8356-df34-4558-9374-3f962f38f851.png" width="700">

Supporting Region (2022.12): Europe (Ireland), Asia Pacific (Singapore), Europe (Frankfurt), Asia Pacific (Tokyo), US East (N. Virginia), US East (Ohio), US West (Oregon)

### ROS

A set of software libraries and tools, from drivers to algorithms, that help developers build robot applications

ROS is an open-source framework that provides inter-process communication mechanisms, standard message types, robotic control and observation tools, hardware abstractions, and common algorithms for robot software development.

ROS makes it easy to reuse and extend existing software so that developers can focus on new algorithms, hardware, and use cases.


### Gazebo
Robust physics engine, high-quality graphics, and programmatic and graphical interfaces to help developers simulate robots

[Gazebo](https://gazebosim.org/home) offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments.


### Colcon

Colcon is a command line tool built by the OSRF(Open Source Robotics Foundation). It is a command line toool to improve the workflow of building, testing and using multiple software packages. It automates the building and bundling of ROS and ROS2 applications.

### Rviz

Rvis is a 3D Visulizer for the ROS framework. It provides information on the robot state and world around it (virtual or real).


### Rqt

Rqt is an extensible dashboard framework that implements the various GUI tools and dashboards in the form of plugins for your robot applications.


## Working

### ROS Robot in Greengrass

[Grrengrass with Dockor](https://github.com/kyopark2014/aws-robomaker/blob/main/greengrass-with-docker.md)에서는 Greengrass2.0에 Docker로 된 ROS Robot을 올리는것에 대해 설명합니다. 



### Robotics Simulations

[Robotics Simulations](https://github.com/kyopark2014/aws-robomaker/blob/main/robomaker-simulation.md)에 대한 코드 분석을 합니다. 



## Workshop 

[Building Cloud Connected Robots](https://catalog.us-east-1.prod.workshops.aws/workshops/fa208b8e-83d6-4cc1-8356-bfa5b6184fae/en-US)


[NVIDIA Isaac Replicator and Isaac Sim on AWS RoboMaker Batch](https://catalog.us-east-1.prod.workshops.aws/workshops/bf038477-a314-403e-9272-508642bc0fcb/en-US)

[Make a robot with AWS IoT Workshop](https://catalog.us-east-1.prod.workshops.aws/workshops/446304b7-b946-4c40-b78f-08bf0025d8f6/en-US)

[Running NVIDIA Isaac Sim on AWS RoboMaker](https://catalog.us-east-1.prod.workshops.aws/workshops/c8280014-6276-4a6c-830c-a0ce18581221/en-US)



## Reference

[What Is AWS RoboMaker? - Developer Guide](https://docs.aws.amazon.com/robomaker/latest/dg/chapter-welcome.html)

[AWS Robotics Blog](https://aws.amazon.com/ko/blogs/robotics/)

[AWS RoboMaker](https://aws.amazon.com/ko/robomaker/)

[AWS Robotics - Github](https://github.com/aws-robotics)

[AWS RoboMaker 기능](https://aws.amazon.com/ko/robomaker/features/)

[Build and Simulate Robotics Applications in AWS Cloud9](https://aws.amazon.com/ko/blogs/robotics/robotics-development-in-aws-cloud9/): EC2에 ROS를 설치하고 Cloud9으로 활용하는 방법을 보여줍니다. 

[Building autonomous robots for everyday tasks with AWS RoboMaker](https://www.youtube.com/watch?v=2UWNIyBaDxg)

[AWS re:Invent 2020: Build a better robot faster with AWS and ROS](https://www.youtube.com/watch?v=6R_CImH8DYs)

[Simulation and Testing Robots using AWS RoboMaker](https://summit.robomakerworkshops.com/ws/multi_robot_fleet_simulations)
