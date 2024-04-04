# Carla for Truck Platooning


# System Requirements
- **Ubuntu 20.04.** CARLA provides support for previous Ubuntu versions up to 16.04. **However** proper compilers are needed for Unreal Engine to work properly.
Dependencies for Ubuntu 18.04 and previous versions are listed separately below. Make sure to install the ones corresponding to your system.
- **130 GB disk space.** Carla will take around 31 GB and Unreal Engine will take around 91 GB so have about 130 GB free to account for both of these plus additional minor software installations.
- **An adequate GPU.** CARLA aims for realistic simulations, so the server needs at least a 6 GB GPU although 8 GB is recommended. A dedicated GPU is highly recommended for machine learning.
- **Two TCP ports and good internet connection.** 2000 and 2001 by default. Make sure that these ports are not blocked by firewalls or any other applications.
- Python 3.8 ++

# 1. Install Dependency
```
sudo apt-add-repository "deb http://apt.llvm.org/focal/ llvm-toolchain-focal main"
sudo apt-get update
sudo apt-get install build-essential clang-10 lld-10 g++-7 cmake ninja-build libvulkan1 python python-dev python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-10/bin/clang++ 180 &&
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-10/bin/clang 180
```
```
pip install --user setuptools &&
pip3 install --user -Iv setuptools==47.3.1 &&
pip install --user distro &&
pip3 install --user distro &&
pip install --user wheel &&
pip3 install --user wheel auditwheel
```

# 2. Install Unreal Engine 4
> Clone the content for CARLA's fork of Unreal Engine 4.26 to your local computer:
```
git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git ~/UnrealEngine_4.26
```
> Navigate into the directory where you cloned the repository:
```
cd ~/UnrealEngine_4.26
```
> Make the build. This may take an hour or two depending on your system.
```
./Setup.sh && ./GenerateProjectFiles.sh && make
```
> Open the Editor to check that Unreal Engine has been installed properly.
```
cd ~/UnrealEngine_4.26/Engine/Binaries/Linux && ./UE4Editor
```

# 3. Install Carla 0.9.15
```
sudo apt install --reinstall build-essential clang-8 lld-8 g++-7 cmake vim ninja-build libvulkan1 python-dev python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf wget libtool rsync libxml2-dev
```
```
cd
git clone https://github.com/carla-simulator/carla
cd ~/carla
git checkout tags/0.9.15
```
```
vi ~/carla/Util/BuildTools/Setup.sh
```
> Remove 94~99 lines & insert this code
```
wget "https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/${BOOST_PACKAGE_BASENAME}.tar.gz" || true
```
> Get assets
```
./Update.sh
```
> Set Unreal Engine Environment Variables
```
sudo vi ~/.bashrc
```
> Insert this code in last line
```
export UE4_ROOT=~/UnrealEngine_4.26
```
> Reboot Terminal
> Build carla
```
cd ~/carla
make PythonAPI
make launch
```

# 4. Import Truck & Trailer models
- Install models 
```
git clone https://github.com/ysrhee6199/CarlaSemiTruckTrailer.git
```
> 1. In ```Content/Carla/Blueprint/Vehicle```, open the ```VehicleFactory``` file.
> 2. In the **Generate Definitions** tab, double click **Vehicles**.
> 3. In the **Details panel**, expand the **Default Value** section and add a new element to the vehicles array.
> 4. Fill in the **Make** and **Model** of your vehicle. For the truck name the **Make**: "DAFxf". And for the trailer name the **Make**: "trailer".
> 5. Fill in the **Class** value with your ```BP_<vehicle_name>``` file.
> 6. Optionally, provide a set of recommended colors for the vehicle. Name the "model" an "make" as below in the picture

![image](https://github.com/AveesLab/scale_truck_control_carla/assets/117966644/6f08583c-eff5-4734-87f6-a0dc88671be0)


> 7. Compile and save.

![image](https://github.com/AveesLab/scale_truck_control_carla/assets/117966644/ed9bc67b-2432-4cfa-ade1-21297d185d00)


- Test the vehicle
> Launch CARLA, open a terminal in ```PythonAPI/examples``` and run the following command:
```
python manual_controlSemiTrailer.py
```




