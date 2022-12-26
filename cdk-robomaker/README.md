## CDK로 배포하기

CDK로 빌드하여 Docker container를 생성합니다. 

```java
    const asset = new DockerImageAsset(this, 'BuildImage', {
      directory: path.join(__dirname, '../../HelloWorld/HelloWorldSampleAppROS2FoxyGazebo11SimApp/'),
    })

    const imageUri = asset.imageUri
    new cdk.CfnOutput(this, 'ImageUri', {
      value: imageUri,
      description: 'Image Uri',
    });
```    

아래와 같이 배포합니다.

```java
cdk deploy
```
