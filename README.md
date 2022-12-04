# FD for dronelab

## This USB has persistence (not live)
30G/; 30G/home;
After initial install: (without install local ros2)
13.8G/; 19.9G/home
After Ros2 setup: ( minus 10.9GB from ros2 built from source)
13.2G/; 18.4G/home

### Storage suggestions (~txa): 
I deactivated updates, removed unnecessary software
https://linuxconfig.org/disable-automatic-updates-on-ubuntu-20-04-focal-fossa-linux
https://linuxnightly.com/how-to-disable-automatic-updates-in-ubuntu/
*Keep OBS Studio for demoes but regularly remove excess videos (automatic?TBD)
*rosdep-update/apt-upgrade executed once | NOT purged but I do not recommend it
(low on storage: might remove opencv 500MB; might remove pcl/mocap 800MB).
 
--------------------------------------------------------------------
## Architecture choices:

### Installed docker-engine, requires Ubuntu 18.04 and up ie. 64-bit arch
https://docs.docker.com/engine/install/ubuntu/
https://docs.docker.com/engine/install/linux-postinstall/

### Installed ros2 docker with preinstalled ros msgs (starting point)
Used docker for: ROS2 docker ("foxy") with port 1000 -> local Unity install.
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md

### Installed local ros for testing the Crazyswarm2 framework
https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html
$ sudo apt install libpcl-dev
https://imrclab.github.io/crazyswarm2/installation.html

### Installed unityhub, adapted to linuxes 18.04 and 20.04 only
https://itsfoss.com/unity-editor-linux/

-----------------------------------------------
## --fixes:

### Grub fix for USB bootloader
https://askubuntu.com/questions/883992/stuck-at-grub-command-line

### ROS Install fails build-essential
 libsqlite3-dev : Depends: libc6-dev but it is not going to be installed
 libc6-dev : Depends: libc6 (= 2.31-0ubuntu9.7) but 2.31-0ubuntu9.9 is to be installed
SOLVED BY:
https://serverfault.com/questions/993576/debian-apt-install-build-essential-fails-because-of-unmet-dependencies
sudo aptitude install g++
sudo aptitude -f install build-essential

### ISSUE WITH acl.c fatal error
https://github.com/iustin/pylibacl/issues/11
sudo apt install libacl1-dev
ONCE build-essential works:
rosdep install --from-paths src --ignore-src --rosdistro dashing -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"
  apt: command [sudo -H apt-get install -y libopencv-dev] failed
apt-get install python3-dev libtinyxml2-dev

------------------------------------------
## Original ROS1 Swarm Stack 
a library of **well-known swarm behaviors** built with ROS 1 upon the Crazyswarm open-source project. 

### 1. Encapsulating multi-step tasks.
    *from single-robot commands to dynamic management.*

### 2. Encapsulating swarm instructions.
    *multi-robot instructions encapsulated within the multi-step tasks.*

### 3. Encapsulating individual task execution.
    *Robot-specific instructions encapsulated within the swarm instructions.*

### examples
Multi-step applications bring together the tasks defined in lower levels.

| TO-AND-FRO LOOP | SWARM DEMO | DRONES & MIXED REALITY DEMO |
|-- | -- | -- |
|![Application: to and fro](/full_applications/to_and_fro.png "To and Fro") | ![Application: octogon](/full_applications/swarm_demo.png "Octogon") | ![Application: octogon](/full_applications/trajectory_diagram.png "Octogon") |
| smlib.oneDrone_ToAndFro (id: _int32 drone reference_) | smlib.execOctogonAndTrajOnCollision (ids: _int32[] drone references_) | |
| # move to position 1<br># move to position 2<br># and loop<br># land if abort | # position drone 1<br># position drone 2<br># concurrent figures of eight<br># 3 drone octogon<br># generalized landing<br># if Unity3D flag appears:<br># if so, concurrent helis | |

> More Examples: https://github.com/ThomasCarstens/swarmStack_flightarena/blob/ros1/readme.md


# Run instructions
Github passkey:
txa@zone2$ ~/home/txa/codes

```
$ unityhub
# Tested with ROS_UNITY_FollowingTutorial (Unity 2020.3)

$ docker run -it --rm -p 10000:10000 foxy /bin/bash
$ ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```
------------------------------------------------
