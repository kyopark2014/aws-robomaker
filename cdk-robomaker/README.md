## 실행결과 

Launch config를 아래와 같이 설정하였습니다.

![image](https://user-images.githubusercontent.com/52392004/209526562-1422f641-9865-432a-af0e-5927f8285cf7.png)

그런데 실행한 결과를 보면 아래와 같이 바이너리 이슈로 실행에 실패하고 있습니다.

![image](https://user-images.githubusercontent.com/52392004/209526320-aaaec5d7-7550-490f-b9d2-03c38732020f.png)

실패한 원인은 "A RoboMaker Compatible Container (RCC) MUST be built for X86_64/amd64 architecture. Image architecture is arm64"와 같이 RoboMaker은 X86만 지원하는데, ARM64를 사용할 수 없다고 합니다. 

[Dockerfile](https://github.com/kyopark2014/aws-robomaker/blob/main/cloud-connected-robots/building-cloud-connected-robots-reinvent2021/Dockerfile)을 확인하면 Base Image는 아래와 같습니다.

```java
ARG BASE_IMAGE=dustynv/ros:${ROS_DISTRO}-ros-base-l4t-r32.6.1
```

이것과 유사한 이미지를 [dockerhub](https://hub.docker.com/layers/dustynv/ros/foxy-ros-base-l4t-r34.1.1/images/sha256-e6babc951718898f374335db070acc6e2b4ecd5862ec633bf04a18fbe968dd4c?context=explore)에서 찾아보면 아래와 같이 ARM64용 이미지임을 확인할 수 있었습니다. 

![image](https://user-images.githubusercontent.com/52392004/209528039-1964d578-6509-421d-b939-8d99a9b44605.png)
