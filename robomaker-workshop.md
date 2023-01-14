## Role

ros1-automation-cfn-EC2InstanceRole-1WDBF1W77VXN5

```java
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Effect": "Allow",
            "Principal": {
                "Service": [
                    "ssm.amazonaws.com",
                    "ec2.amazonaws.com"
                ]
            },
            "Action": "sts:AssumeRole"
        }
    ]
}
```

AmazonEC2FullAccess, AdministratorAccess, AmazonSSMManagedInstanceCore
