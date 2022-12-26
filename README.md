# AWS RoboMaker


AWS의 로보메이커(RoboMaker)는 클라우드 기반의 로봇 시뮬레이션 서비스입니다. 로봇 개발자들이 로봇을 개발하고 시험하고 Cloud를 이용해 배포할수 있도록 해줍니다. 또한 다수의 로봇을 실행하고 자동화 할 수 있습니다. 

아래는 AWS의 RoboMaker의 Architecture를 보여줍니다. 

<img src="https://user-images.githubusercontent.com/52392004/209247633-304c8356-df34-4558-9374-3f962f38f851.png" width="700">

Supporting Region (2022.12): Europe (Ireland), Asia Pacific (Singapore), Europe (Frankfurt), Asia Pacific (Tokyo), US East (N. Virginia), US East (Ohio), US West (Oregon)

### ROS

[Robot Operating System (ROS)](https://github.com/kyopark2014/aws-robomaker/blob/main/ros.md)는 로봇 응용 프로그램 개발용 위한 미들웨어 프레임워크입니다. 

### Development Tools

[Development Tools](https://github.com/kyopark2014/aws-robomaker/blob/main/development-tools.md)에서는 Gazebo, Rviz, Rqt등 개발툴에 대해 정리합니다.


## 중요 References

Cloud9을 이용할 경우에, [Cloud9에서 EBS 크기 변경](https://github.com/kyopark2014/technical-summary/blob/main/resize.md)에 따라 cloud9의 볼륨을 조정합니다. 

### Hello World

[RoboMaker: Hello World](https://github.com/kyopark2014/aws-robomaker/blob/main/robomaker-helloworld.md)에서는 컨테이너를 이용해 ROS workspace를 구동하는것에 대한 코드를 분석합니다. 

### ROS Robot in Greengrass

[Grrengrass with Dockor](https://github.com/kyopark2014/aws-robomaker/blob/main/greengrass-with-docker.md)에서는 Greengrass2.0에 Docker로 된 ROS Robot을 올리는것에 대한 코드를 분석합니다. 


### Robotics Simulations

[Robotics Simulations](https://github.com/kyopark2014/aws-robomaker/blob/main/robomaker-simulation.md)에 대한 코드 분석을 합니다. 


### Cloud Connected Robots

[Cloud Connected Robots](https://github.com/kyopark2014/aws-robomaker/tree/main/cloud-connected-robots)에 대한 코드를 분석합니다. 


## Workshop 



[NVIDIA Isaac Replicator and Isaac Sim on AWS RoboMaker Batch](https://catalog.us-east-1.prod.workshops.aws/workshops/bf038477-a314-403e-9272-508642bc0fcb/en-US)

[Make a robot with AWS IoT Workshop](https://catalog.us-east-1.prod.workshops.aws/workshops/446304b7-b946-4c40-b78f-08bf0025d8f6/en-US)

[Running NVIDIA Isaac Sim on AWS RoboMaker](https://catalog.us-east-1.prod.workshops.aws/workshops/c8280014-6276-4a6c-830c-a0ce18581221/en-US)



## Reference

[What Is AWS RoboMaker? - Developer Guide](https://docs.aws.amazon.com/robomaker/latest/dg/chapter-welcome.html)

[AWS Robotics Blog](https://aws.amazon.com/ko/blogs/robotics/)

[AWS RoboMaker](https://aws.amazon.com/ko/robomaker/)

[AWS Robotics - Github](https://github.com/aws-robotics)

[AWS RoboMaker 기능](https://aws.amazon.com/ko/robomaker/features/)

[Simulation and Testing Robots using AWS RoboMaker](https://summit.robomakerworkshops.com/ws/multi_robot_fleet_simulations): RoboMaker 사용법에 대한 간단한 설명입니다.

[Run a Simulation in AWS RoboMaker](https://www.youtube.com/watch?v=CocGUfhp-I8)

[Build and Simulate Robotics Applications in AWS Cloud9](https://aws.amazon.com/ko/blogs/robotics/robotics-development-in-aws-cloud9/): EC2에 ROS를 설치하고 Cloud9으로 활용하는 방법을 보여줍니다. 

[Building autonomous robots for everyday tasks with AWS RoboMaker](https://www.youtube.com/watch?v=2UWNIyBaDxg)

[AWS re:Invent 2020: Build a better robot faster with AWS and ROS](https://www.youtube.com/watch?v=6R_CImH8DYs): ROS에 대한 상세한 설명과 Console에서 수행하는 Robomaker Hands on의 내용이 좋습니다.


[Build a Robot Application with AWS RoboMaker - AWS Online Tech Talks](https://www.youtube.com/watch?v=b7wzhSo4F_M): 전반적인 좋은 설명을 들을 수 있고, 이전 UI이지만, Hands on도 참고할만합니다. 
