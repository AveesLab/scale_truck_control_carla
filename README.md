# After Host PC setting

# 1. Set container
> Open teminal
```
docker pull aveeslab/platooning:carla
xhost +
docker run -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --env QT_X11_NO_MITSHM=1 --name world aveeslab/platooning:carla /bin/bash

# In the container
cd /home && mkdir -p ros2_ws/src
```
> Build ros packages
```
cd /home/ros2_ws/src
git clone -b edit https://github.com/AveesLab/scale_truck_control_carla.git
```
```
cd /home/ros2_ws
source /opt/ros/galactic/setup.bash
colcon build --packages-select ros2_msg

source ./install/setup.bash
colcon build --symlink-install
```
# 2. Run ROS2 pakcages
- Open three terminal
> First terminal
```
# Docker exec
docker start world
xhost +
docker exec -it world /bin/bash
```
```
cd /home/ros2_ws
source /opt/ros/galactic/setup.bash
source ./install/setup.bash
ros2 launch scale_truck_control_ros2 LV.launch.py
```
> Second terminal
```
# Docker exec
docker start world
xhost +
docker exec -it world /bin/bash
```
```
cd /home/ros2_ws
source /opt/ros/galactic/setup.bash
source ./install/setup.bash
ros2 launch scale_truck_control_ros2 FV1.launch.py
```
> Third terminal
```
# Docker exec
docker start world
xhost +
docker exec -it world /bin/bash
```
```
cd /home/ros2_ws
source /opt/ros/galactic/setup.bash
source ./install/setup.bash
ros2 launch scale_truck_control_ros2 FV2.launch.py
```


