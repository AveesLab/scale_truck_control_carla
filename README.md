# After Host PC setting

# 1. Install ROS 2 (Galactic)

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
    
# 2. Install ROS 2 pakcages
- Create ROS2 workspace
    ```
    source /opt/ros/galactic/setup.bash
    mkdir -p ~/ros2_ws/src
    ```
- Install packages
    ```
    cd ~/ros2_ws/src
    git clone https://github.com/AveesLab/scale_truck_control_carla.git
    ```
- Build packages
    ```
    cd ~/ros2_ws
    source /opt/ros/galactic/setup.bash
    colcon build --packages-select ros2_msg
    ```
    ```
    source ./install/setup.bash
    colcon build --symlink-install
    ```

# 3. Run
- LV run
    ```
    source ./install/setup.bash
    ros2 launch scale_truck_control_carla LV.launch.py 
    ```
- FV1 run
    ```
    source ./install/setup.bash
    ros2 launch scale_truck_control_carla FV1.launch.py 
    ```
- FV2 run
    ```
    source ./install/setup.bash
    ros2 launch scale_truck_control_carla FV2.launch.py 
    ```


