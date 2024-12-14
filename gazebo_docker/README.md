## To build Docker run:

```shell
xhost +
```

```shell
sudo docker build -t name_of_thing .
```

```shell
sudo docker run -it --rm \
    -p 2233:2233 \
    --env="DISPLAY=$DISPLAY" \
    --env="XDG_SESSION_TYPE=$XDG_SESSION_TYPE" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    name_of_thing /bin/bash
```

### After running this, gazebo should pop up:
```shell
roslaunch unitree_gazebo normal.launch rname:=a1 wname:=stairs_single
```

---

## In a new terminal run (everything below this is in this new terminal):

### Get the CONTAINER ID of the build:
```shell
sudo docker ps
```

### Run the Docker thing in this terminal:
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
cd /root/A1_ctrl_ws/
```

```shell
catkin build
```

```shell
source /root/A1_ctrl_ws/devel/setup.bash
```

```shell
echo "source /root/A1_ctrl_ws/devel/setup.bash" >> /.bashrc
```

---

### Run this command, and then pause the gazebo simulation:
### This should reset the robot. Also press `Ctrl+C` so you can type another command after pausing:
```shell
rosrun unitree_controller unitree_move_kinetic
```

### Run the MPC controller, and then unpause the gazebo simulation to see the control work:
```shell
roslaunch a1_cpp a1_ctrl.launch type:=gazebo solver_type:=mpc
```
