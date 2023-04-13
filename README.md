# master-thesis
This repositoriy contains the files and software used in the master's thesis project of Tristan Schwörer and Jonathan Eichild Schmidt in Robotics (MSc) at Aalborg University. 
The project were conducted during the spring semester of 2023.

# Docker

#### Docker install
https://docs.docker.com/engine/install/ubuntu/
#### Post install instructions 
https://docs.docker.com/engine/install/linux-postinstall/

## Docker setting on jetson (ssh into it):
#### Move docker root to mounted SD card using:
```
  sudo systemctl stop docker
  sudo systemctl stop docker.socket
  sudo systemctl stop containerd
  sudo mkdir -p /mnt/SDcard/docker/root
  sudo mv /var/lib/docker /mnt/SDcard/docker/root
```
#### docker /etc/docker/daemon.json
```
  {
      "runtimes": {
          "nvidia": {
              "path": "nvidia-container-runtime",
              "runtimeArgs": []
          }
      },
      "default-runtime": "nvidia",
      "data-root": "/mnt/SDcard/docker/root"
  }
```
#### docker config is set to the one included in this repository using:
```
  docker --config /mnt/SDcard/master-thesis/ ps
```
#### To make it persistent
```
  echo export DOCKER_CONFIG=/mnt/SDcard/master-thesis/.docker > ~/.profile
```
#### Restart docker for good measure using:
```
  sudo systemctl restart docker
```
#### Check if root directory is correct:
```
  docker info -f '{{ .DockerRootDir}}'
```
should return /mnt/SDcard/docker/root

## Pull the repository with all submodules using:
```
git clone --recursive -j8 git@github.com:Tristan9497/master-thesis.git
```

*The docker containers are build on an SDcard that is permanently mounted to /mnt/Sdcard/ to make it viable to deploy on a Jetson. For different paths the docker-compose.yml needs to be adjusted accordingly*

## Build 
Build the two images for the individual Docker containers using (takes hours):
```
cd /mnt/SDcard/master-thesis/jetson-containers

./scripts/docker_build_ros.sh --distro humble --package ros_base --with-pytorch

cd ..

docker compose -f /realsense-ros2-docker/docker/docker-compose-nvidia.yml build
```
After build update the image id of the rostorch container in the docker-compose.yml file 

## Use the docker containers
*To use the docker containers for this project the **docker-compose.yml** file contains the needed instructions and the containers can be run using:*
```
cd /mnt/SDcard/master-thesis
docker compose up
```

#### Create a terminal in the rostorch container using:
```
docker exec -it master-thesis-rostorch-1 bash
```

#### Build the ros2 workspace when in rostorch container using:
```
cd docker-volume/ros_ws
colcon build
source install setup.bash
```

#### Run pose_esitmator when in rostorch container using:
```
ros2 run multi_person_tracker multi_person_tracker
```


# PoseNet
As the roject uses PoseNet from the jetson-inference package, it is important that the network is downloaded before use. This can be done using the tool provided by Nvidia *jetson-inference/tools/download-models.sh* and downloading **Pose-ResNet18-Body**


# Issues
in case of docker error like:
```
  ERROR [internal] load metadata for nvcr.io/nvidia/l4t-jetpack:r35.2.1 
```
The docker image needs to be pulled manually using e.g.:
```
  docker pull nvcr.io/nvidia/l4t-jetpack:r35.2.1 
```
*this was happening when rebuilding the containers*
in case of docker pull access denied on docker compose up

 sudo systemctl daemon-reload

 sudo systemctl restart docker
