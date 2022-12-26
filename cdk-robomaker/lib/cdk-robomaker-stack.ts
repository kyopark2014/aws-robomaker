import * as cdk from 'aws-cdk-lib';
import { Construct } from 'constructs';

import * as path from "path";
import { DockerImageAsset } from 'aws-cdk-lib/aws-ecr-assets';
import * as greengrassv2 from 'aws-cdk-lib/aws-greengrassv2';
import * as s3 from 'aws-cdk-lib/aws-s3';
import * as s3Deploy from "aws-cdk-lib/aws-s3-deployment"

export class CdkRobomakerStack extends cdk.Stack {
  constructor(scope: Construct, id: string, props?: cdk.StackProps) {
    super(scope, id, props);

    const asset = new DockerImageAsset(this, 'BuildImage', {
      directory: path.join(__dirname, '../../HelloWorld/HelloWorldSampleAppROS2FoxyGazebo11SimApp/'),
    })

    const imageUri = asset.imageUri
    new cdk.CfnOutput(this, 'ImageUri', {
      value: imageUri,
      description: 'Image Uri',
    }); 
  }
}
