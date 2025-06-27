# Setup CARLA
## How setup local development environments
### Environment System and Setup Configuration 
* Microsoft Windows 11 Enterprise Version 10.0.22631 Build 22631
* WSL2
    ``` 
    - WSL version: 2.2.4.0
    - Kernel version: 5.15.153.1-2
    - WSLg version: 1.0.61
    - MSRDC version: 1.2.5326
    - Direct3D version: 1.611.1-81528511
    - DXCore version: 10.0.26091.1-240325-1447.ge-release
    - Windows version: 10.0.22631.4169 
    ```
* Ubuntu 22.04 (Jammy) on WSL
* Python Version 3.10
* Nvidia 
    ```
    - Nvidia Studio (or Game Ready) Driver (Window Installer Version)
    - Nvidia CUDA tool-kit 12.6 (Specific Version WSL2 .dep package not Windows Installer)
    ```
* ROS 2 Humble (LTS) Distribution (On Ubuntu)
* Gazebos Fortress (LTS) (On Ubuntu)
* Carla 0.9.15 (Windows Package Version)
## Environment Setup Steps
### 1. [Install WSL2 and then Ubuntu 22.04 based on the Microsoft official docs](https://learn.microsoft.com/en-us/windows/wsl/install) 
1. For ROS2 Nodes under WSL2 Ubuntu to be able to connect Carla Server running on Windows via "localhost:port", you should create a configuration file named ```.wslconfig``` under ```C:\Users\<username>``` and add the following to it

    ```
    [wsl2]
    networkingMode=mirrored
    ```
2. You should also define proper inbound and output rules for window firewall or disable it by running the following command:
    ```
    netsh advfirewall set currentprofile state off
    ```
### 2. Install Nvidia Driver and CUDA tool-kits
According to the official Microsoft and Nvidia, you don't need (specifically should not) install the Nvidia graphic card driver on the Linux WSL2 distribution because the WSL uses the host graphic card driver on Windows.

1. [Install proper Nvidia driver for your graphic card on Windows](https://www.nvidia.com/en-us/geforce/drivers) 
 * There are two class of drivers, Game Ready and Nvidia Studio, I prefer Studio becaus due to Nvidia docs, Studio versions are more reliable
 * The version you pick is very important, some versions may not compatible with Gazebo or Ubuntu mesa-util package. I found proper version by trial and error because there isn't any official documentations for compatibility check. on my local machine I test with Nvidia GeForce GTX 1050 (notebook) graphic card and Driver Version 560.81 Nvidia Studio 
2. [Install DirectX 3d12 on Windows](https://learn.microsoft.com/en-us/windows/ai/directml/gpu-tensorflow-plugin)
3. [Install WSL2 CUDA version on Ubuntu](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=WSL-Ubuntu&target_version=2.0&target_type=deb_local)
    * You should install specific .dep package distribution which specifically provided by Nvidia for WSL2 (not windows and not stan alone Linux versions) [for more details see this repository](https://github.com/ashishpatel26/Cuda-installation-on-WSL2-Ubuntu-20.04-and-Windows11) [and this official documentation](https://docs.nvidia.com/cuda/wsl-user-guide/index.html) and [this link](https://developer.nvidia.com/cuda/wsl)
    * You could download the file using windows and move it under Ubuntu WSL directory manually instead of directly download the file from Ubuntu shell. you could access windows C drive directories via Ubuntu shell from `cd /mnt/c`

        ```
        $ wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-wsl-ubuntu.pin
        
        $ sudo mv cuda-wsl-ubuntu.pin /etc/apt/preferences.d/cuda-repository-pin-600

        $ wget https://developer.download.nvidia.com/compute/cuda/12.6.1/local_installers/cuda-repo-wsl-ubuntu-12-6-local_12.6.1-1_amd64.deb
        
        $ sudo dpkg -i cuda-repo-wsl-ubuntu-12-6-local_12.6.1-1_amd64.deb
        
        $ sudo cp /var/cuda-repo-wsl-ubuntu-12-6-local/cuda-*-keyring.gpg /usr/share/keyrings/
        
        $ sudo apt-get updatesudo apt-get -y install cuda-toolkit-12-6
        ```
4. Install mesa-utils

    ```
    $ sudo apt install mesa-utils
    ```
5. Change Mesa profile to use Nvidia graphic card instead of integrated intel GPU
    ```
    $ export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    ```
    Then check if it correctly identifies your graphic card model

    ```
    $ glxinfo -B
    ```

### 3. [Install ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
1. Check for utf-8:

    Check for UTF-8 encoding using the command locale; if it’s not there, execute the command locale. You can run all the commands below irrespective of having UTF-8.
    ```
    # if locale shows below output, no need to execute 
    # below commands
    
    # Expected output from command `locale`
    # LANG=en_US.UTF-8
    # LANGUAGE=en_IN:en
    # LC_CTYPE="en_US.UTF-8"
    # LC_NUMERIC="en_US.UTF-8"
    # LC_TIME="en_US.UTF-8"
    # LC_COLLATE="en_US.UTF-8"
    # LC_MONETARY="en_US.UTF-8"
    # LC_MESSAGES="en_US.UTF-8"
    # LC_PAPER="en_US.UTF-8"
    # LC_NAME="en_US.UTF-8"
    # LC_ADDRESS="en_US.UTF-8"
    # LC_TELEPHONE="en_US.UTF-8"
    # LC_MEASUREMENT="en_US.UTF-8"
    # LC_IDENTIFICATION="en_US.UTF-8"
    # LC_ALL=en_US.UTF-8
    
    $ locale  # check for UTF-8

    $ sudo apt update && sudo apt install locales
    $ sudo locale-gen en_US en_US.UTF-8
    $ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    $ export LANG=en_US.UTF-8
    $ locale  # verify settings
    ```
2. Add the ROS 2 apt Repository
You will need to add the ROS 2 apt repository to your system. First, ensure that the Ubuntu [Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled.
    ```
    $ sudo apt install software-properties-common
    $ sudo add-apt-repository universe
    ```
    Now add the ROS 2 GPG key with apt.

    ```
    $ sudo apt update && sudo apt install curl -y
    $ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
    Then add the repository to your sources list.
    ```
    $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
3. apt Package Update:
    
    Do apt package update and upgrade
    ```
    $ sudo apt update
    $ sudo apt upgrade
    ```
    On Ubuntu 22.04, for humble to work properly you need systemd and udev to be updated. To see if these packages are in their latest version or not, use below command,
    ```
    $ sudo apt-get install --only-upgrade udev systemd
    ```
4. Install ROS 2 Humble:

    The official documentation recommends installing the ros-humble-desktop-full package, as it includes ROS 2, RViz, demos, Gazebo, tutorials, and more, all bundled together. However, if you prefer to install Gazebo separately, you can use the ros-humble-desktop package instead.
    ```
    $ sudo apt install ros-humble-desktop-full
 
    # OR
 
    $ sudo apt install ros-humble-desktop
    ```
5. Sourcing .bashrc file:

    Add below command in .bashrc to automatically load humble when opening a terminal,

    ```
    # open ~/.bashrc and add this line
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    
    # close gedit and source `.bashrc`
    $ source ~/.bashrc
    ```
6. ROS 2 Build tool:

    In ROS1, the preferred build tool was catkin, but in ROS 2 it is colcon. Install that using the command,

    ```
    $ sudo apt install python3-colcon-common-extensions
    ```
    Above command, install colcon_argcomplete as well. add below in the .bashrc,

    ```
    $ echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
 
    # close gedit and source `.bashrc`

    $ source ~/.bashrc
    ```
    colcon supports multiple build types. The recommended build types are ament_cmake and ament_python. Also supported are pure cmake packages.

7. Setup colcon_cd:

    Run below command to enable colcon_cd,
    ```
    $ echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
 
    $ echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
    
    # close gedit and source `.bashrc`
    $ source ~/.bashrc
    ```
8. Understanding rosdep Understanding rosdep

    Understanding rosdep is crucial. This tool identifies and installs dependencies, acting as a meta-package manager. It uses its system knowledge and the package.xml file to find and install the appropriate packages, referred to as rosdep keys. These keys are cross-referenced against a central index to locate the exact package name for installation. rosdep retrieves this central index to your local machine to avoid network access each time it runs. Here are the instructions to install rosdep,
    
    ```	
    $ sudo apt-get install python3-rosdep
    ```
    * Note:
    On Debian and Ubuntu, there is another, similarly named package called python3-rosdep2. If that package is installed, make sure to remove it before installing python3-rosdep.

    First time using rosdep, it must be initialized via,
    
    ```
    $ sudo rosdep init # creates a source list file
 
    $ rosdep update
    ```

    Finally, we can run rosdep install to install dependencies. Run the following command from the root of the workspace,
    ```
    $ rosdep install --from-paths src -y --ignore-src
    ```
9. Test ROS 2

    Check if ROS 2 is working properly. To check that we will be using an in-built publisher and subscriber. Run the below commands to check,
    ```
    # in a new terminal
    $ source /opt/ros/humble/setup.bash
 
    $ ros2 run demo_nodes_cpp talker
    ```

    ```
    # in a new terminal 
    $ source /opt/ros/humble/setup.bash
    
    $ ros2 run demo_nodes_py listener
    ```
### 4. [Carla Installation on Windows](https://carla.readthedocs.io/en/0.9.15/start_quickstart/)

In the following instructions Replace ```username``` with your ubuntu username.Also make sure you have an NVIDIA driver and cuda-toolkit installed in your system, unless Carla won’t be able to use your GPU. 

1. You can try to install all the dependencies and compile everything from scratch by following the official guide. But, we are just going to download the pre-compiled file and use it. To do so,
Download the ```CARLA_0.9.15.zip``` from [this link](https://github.com/carla-simulator/carla/releases/tag/0.9.15) under any directory that you want (suggest a drive except of Windows ```C``` drive because of possible access permissions). assume it would be folder named carla_simulator inside ```Your Path To\CARLA_0.9.15``` and unzip that zip file inside.

2. Install the Carla API client python package in python 3.10, and finally check by importing Carla. 
    ```
    $ pip install carla
    ```
### 5. Carla ROS 2 Bridge Setup on Ubuntu WSL2
The Carla ROS-Bridge officially supports ROS 2 Foxy on Ubuntu 20.04. To make it work with ROS 2 Humble on Ubuntu 22.04 and Carla 0.9.15, I will use a modified fork of the ROS-Bridge. You can find [this version of Carla ROS-Bridge here](https://github.com/samanipour/carla_ros)
1. Run below commands to install carla ros-bridge.
    ```
    #required for running `ros-bridge`
    $ sudo apt install ros-humble-derived-object-msgs 
    
    $ mkdir -p ~/carla-ros-bridge/catkin_ws/src
    
    $ cd ~/carla-ros-bridge/catkin_ws/src
    
    $ git clone --recurse-submodules https://github.com/samanipour/carla_ros.git
    
    $ cd .. # after cd make sure you are inside ~/carla-ros-bridge/catkin_ws/
    
    $ rosdep install --from-paths src --ignore-src -r
    $ pip install pygame
    $ colcon build --symlink-install
    $ source install/setup.bash
    ```
2. launch a simple example

    Run below commands to start ros-bridge. Please download the [objects.json](https://github.com/carla-simulator/ros-bridge/blob/master/carla_spawn_objects/config/objects.json) and [carla_ros.rviz](https://gist.github.com/soumya997/0b7a43fe23d2549aafe19972349664e1) these two files will be used in future.

    ```
    # in a Windows terminal, run carla first
    $ cd Your Path To\CARLA_0.9.15
    $ .\CarlaUE4.exe  # or .\CarlaUE4.exe -prefernvidia
    ```

    ```
    # in a Ubuntu terminal, get inside the `carla-ros-bridge/colcon_ws` folder 
    $ cd ~/carla-ros-bridge/colcon_ws
    $ source install/setup.bash # source the workspace
    # launch the `carla ros-bridge`
    $ ros2 launch carla_ros_bridge carla_ros_bridge.launch.py   synchronous_mode:=True town:=Town03 # <town number, eg: 03>
    
    # in a new Ubuntu terminal, launch the objects.json
    $ cd ~/carla-ros-bridge/colcon_ws
    $ source install/setup.bash # source the workspace
    $ ros2 launch carla_spawn_objects carla_example_ego_vehicle.launch.py   spawn_sensors_only:=False objects_definition_file:=/absolute/path/to/ objects.json
    
    # in new Ubuntu terminal, launch the rviz2
    $ rviz2 -d /path/to/carla_ros.rviz
    ```
### 6. Ubuntu Setup
1. You May add the followings to .bashrc 

    ```
    echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf > /dev/null
    export _colcon_cd_root=/opt/ros/humble/
    export CARLA_ROOT=/home/ali/CARLA_0.9.14/
    export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/ PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    source /opt/ros/humble/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    source /usr/share/colcon_cd/function/colcon_cd.sh
    source ~/carla-ros-bridge/catkin_ws/install/setup.bash
    ```

## Project Task Board
[Project Task Board](https://github.com/samanipour/CAXSim/projects)
