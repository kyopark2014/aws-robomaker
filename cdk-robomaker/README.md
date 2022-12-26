## 실행결과 

Launch config를 아래와 같이 설정하였습니다.

![image](https://user-images.githubusercontent.com/52392004/209526562-1422f641-9865-432a-af0e-5927f8285cf7.png)

그런데 실행한 결과를 보면 아래와 같이 바이너리 이슈로 실행에 실패하고 있습니다.

![image](https://user-images.githubusercontent.com/52392004/209526320-aaaec5d7-7550-490f-b9d2-03c38732020f.png)

실패한 원인은 "A RoboMaker Compatible Container (RCC) MUST be built for X86_64/amd64 architecture. Image architecture is arm64"와 같이 RoboMaker은 X86만 지원하는데, ARM64를 사용할 수 없다고 합니다. 
