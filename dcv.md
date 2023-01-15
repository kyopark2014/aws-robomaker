# NICE DCV

NICE DCV is a high-performance remote display protocol.

The NICE DCV server software is used to create a secure session. You install and run your applications on the server. 

## Install

### Prerequisites for Linux NICE DCV servers
#### [Install a desktop environment and desktop manager](https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-prereq.html)

- Ubuntu 18.x

```java
sudo apt update
sudo apt install ubuntu-desktop
sudo apt install lightdm
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
sudo apt install ./nice-dcv-server_2022.2.14175-1_amd64.ubuntu1804.deb
sudo usermod -aG video dcv
sudo apt install ./nice-xdcv_2022.2.487-1_amd64.ubuntu1804.deb
sudo apt install ./nice-dcv-gl_2022.2.983-1_amd64.ubuntu1804.deb
sudo apt install ./nice-dcv-simple-external-authenticator_2022.2.198-1_amd64.ubuntu1804.deb
sudo DISPLAY=:0 XAUTHORITY=$(ps aux | grep "X.*\-auth" | grep -v grep | sed -n 's/.*-auth \([^ ]\+\).*/\1/p') xhost | grep "SI:localuser:dcv$"
```





## Reference 

[What Is NICE DCV?](https://docs.aws.amazon.com/dcv/latest/adminguide/what-is-dcv.html)
