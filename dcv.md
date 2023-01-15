# NICE DCV

NICE DCV is a high-performance remote display protocol.

The NICE DCV server software is used to create a secure session. You install and run your applications on the server. 

## Install

### Prerequisites for Linux NICE DCV servers
#### [Install a desktop environment and desktop manager](https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-prereq.html)

- Ubuntu 18.x

```java
sudo apt update
sudo apt install ubuntu-desktop -y
sudo apt install lightdm -y 
sudo apt upgrade
sudo reboot
```

#### [Install the NICE DCV Server on Linux](https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-server.html)

- Ubuntu 18.04 (64-bit x86)

```java
wget https://d1uj6qtbmh3dt5.cloudfront.net/NICE-GPG-KEY
gpg --import NICE-GPG-KEY
wget https://d1uj6qtbmh3dt5.cloudfront.net/2022.2/Servers/nice-dcv-2022.2-14175-ubuntu1804-x86_64.tgz
tar -xvzf nice-dcv-2022.2-14175-ubuntu1804-x86_64.tgz && cd nice-dcv-2022.2-14175-ubuntu1804-x86_64 
sudo apt install ./nice-dcv-server_2022.2.14175-1_amd64.ubuntu1804.deb -y
sudo usermod -aG video dcv
sudo apt install ./nice-xdcv_2022.2.487-1_amd64.ubuntu1804.deb
sudo apt install ./nice-dcv-gl_2022.2.983-1_amd64.ubuntu1804.deb -y
sudo apt install ./nice-dcv-simple-external-authenticator_2022.2.198-1_amd64.ubuntu1804.deb
```

아래 명령어로 dcv 동작 가능 여부를 확인합니다. 정상이라면 "SI:localuser:dcv"라고 결과가 나옵니다.

```java
sudo DISPLAY=:0 XAUTHORITY=$(ps aux | grep "X.*\-auth" | grep -v grep | sed -n 's/.*-auth \([^ ]\+\).*/\1/p') xhost | grep "SI:localuser:dcv$"
```

동작확인

```java
sudo dcvgldiag

NICE DCV - Diagnostic Script
============================================

Date:             Sun, 15 Jan 2023 07:31:32 +0000

Host:             ip-172-31-21-203
Architecture:     x86_64
Operating System: Ubuntu 18.04.6 LTS
Kernel Version:   5.4.0-1093-aws
Nvidia GPU:       unknown
Nvidia Driver:    unknown
Runlevel:         5

X configuration file: /etc/X11/xorg.conf

DCV GL (GLVND) is enabled for 64 bit applications.

Running tests: .......... DONE

ERROR (1/1)

  The '/etc/X11/xorg.conf' file is missing or not readable.

  Please, check the X Server configuration.


WARNING (1/1)

  No NVIDIA card found

  Please, check a NVIDIA card is installed.



There is 1 error and 1 warning.

A detailed report about the tests is available in '/home/ubuntu/environment/dcvgldiag-yASQDn'
```

## Reference 

[What Is NICE DCV?](https://docs.aws.amazon.com/dcv/latest/adminguide/what-is-dcv.html)
