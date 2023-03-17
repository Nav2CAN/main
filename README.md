# master-thesis
This repositoriy contains the files and software used in the Master thesis project of Tristan Schwörer and Jonathan Eichild Schmidt in Robotics (MSc) at Aalborg University. 
The project were conducted during the spring semester of 2023.

## Docker
To use the docker containers for this project the **docker-compose.yml** file contains the needeed instructions and the containers can be initialised with *docker-compose up*

## PoseNet
As the roject uses PoseNet from the jetson-inference package, it is important that the network is downloaded before use. This can be done using the tool provided by Nvidia *jetson-inference/tools/download-models.sh* and downloading **Pose-ResNet18-Body**