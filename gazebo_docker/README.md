## Install https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html to use GPU

## To build Docker run:

```shell
xhost +
```

```shell
sudo docker build -t name_of_thing .
```

```shell
sudo docker run -it 	
--env="DISPLAY=$DISPLAY" 	
--env="QT_X11_NO_MITSHM=1" 	
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" 	
--env="XAUTHORITY=$XAUTH" 	
--volume="$XAUTH:$XAUTH" 	
--runtime=nvidia 	
--privileged 	
--network host 	
--name $name$ 	
$name$

```

### After running this, gazebo should pop up:
```shell
roslaunch unitree_gazebo normal.launch rname:=go1 wname:=stairs_single
```

---

## In a new terminal run (everything below this is in this new terminal):

### Get the CONTAINER ID of the build:
```shell
sudo docker ps
```

### Run the Docker thing in this terminal (you might be able to replace $CONTAINER_ID$ with $name$, skipping the previous step:
```shell
sudo docker exec -it $CONTAINER_ID$ bash
```

```shell
service ssh start
```

### The password is `password`:
```shell
ssh root@localhost -p2233
```

```shell
cd /root/Go1_ctrl_ws/
```

```shell
catkin build
```

```shell
source /root/Go1_ctrl_ws/devel/setup.bash
```

```shell
echo "source /root/Go1_ctrl_ws/devel/setup.bash" >> /.bashrc
```

---

### Run these commands to make it so the robot is in default position, and then pause the gazebo simulation:
### This should reset the robot. Also press `Ctrl+C` so you can type another command after pausing:
```shell
rosrun unitree_controller unitree_servo
```
```shell
rosrun unitree_controller unitree_move_kinetic
```

### Run the MPC controller, and then unpause the gazebo simulation to see the control work:
```shell
roslaunch Go1_cpp go1_ctrl.launch type:=gazebo solver_type:=qp
```
