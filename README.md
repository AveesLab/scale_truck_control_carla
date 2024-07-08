# After Host PC setting

# 1. Install ROS 2 (Galactic)
> Open teminal
- Set locale
    ```
    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings
    ```
- Setup sources
    ```
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    ```
    ```
    sudo apt update && sudo apt install curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
    ```
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
- Install ROS 2 packages
    ```
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-galactic-desktop
    ```
- Colcon install
    ```
    sudo apt install python3-colcon-common-extensions
    ```
    
# 2. Run ROS2 pakcages
- Open three terminal
> First terminal
```
# Docker exec
docker start world
xhost +
docker exec world /bin/bash
```
```
cd /home/ros2_ws
source /opt/ros/galactic/setup.bash
ros2 launch scale_truck_control_ros2 LV.launch.py
```
> Second terminal
```
# Docker exec
docker start world
xhost +
docker exec world /bin/bash
```
```
cd /home/ros2_ws
source /opt/ros/galactic/setup.bash
ros2 launch scale_truck_control_ros2 FV1.launch.py
```
> Third terminal
```
# Docker exec
docker start world
xhost +
docker exec world /bin/bash
```
```
cd /home/ros2_ws
source /opt/ros/galactic/setup.bash
ros2 launch scale_truck_control_ros2 FV2.launch.py
```


