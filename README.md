# 1. build docker image


### 1_1. build docker image with Dockerfile
You can create a docker image using one of the two methods below.

```bash
cd your/catkin_ws/
docker build -t lidartag .
```
<br/>

### 1_2. build docker image using docker pull

```bash
docker pull yechanpark5714/lidartag:latest
```
<br/>


    
# 2. create docker container
You can create a container with the docker run command below.


**!! IMPORTANT !!**: The **--volume** should be set to the path to the folder where your lidartag is located, and **the container name** and **image name** should be modified accordingly.

```bash
nvidia-docker run --privileged -it \
--gpus all \
-e NVIDIA_DRIVER_CAPABILITIES=all \
-e NVIDIA_VISIBLE_DEVICES=all \
--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
--net=host \
--ipc=host \
--shm-size=2gb \
--volume=/your/catkin_ws/path:/home/lidartag \
--name=your docker container name \
--env="DISPLAY=$DISPLAY" \
your docker image name
```


<br/>

# Acknowledgments
[LiDARTag](https://github.com/UMich-BipedLab/LiDARTag.git): A Real-Time Fiducial Tag System for Point Clouds
