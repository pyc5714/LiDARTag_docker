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


**!! IMPORTANT !!**: The **--volume** parameter should be set to the directory containing your lidartag path, and you need to update the **container name** and **image name** as appropriate.

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

# 3. start docker container
```bash
    xhost +local:docker
    sudo docker start <your lidartag container>
    sudo docker exec -it <your lidartag container> /bin/bash
 ```
# 4. build

```bash
    cd /home/lidartag
    git clone https://github.com/pyc5714/lidartag.git

    source /opt/ros/melodic/setup.bash
    catkin_make
    source /home/lidartag/devel/setup.bash

    roslaunch lidartag LiDARTag_outdoor.launch
```



# Acknowledgments
[LiDARTag](https://github.com/UMich-BipedLab/LiDARTag.git): A Real-Time Fiducial Tag System for Point Clouds
