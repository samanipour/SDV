# Autoware
## How to set up Autoware on Ubuntu 22.04 Windows WSL2

### Installing Ubuntu on WSL2

```
wsl --install Ubuntu-22.04
```

### Setting up Ubuntu

```
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf > /dev/null
sudo apt-get -y update
```
#### Install git
```
sudo apt-get -y install git
```

### How to set up a Autoware development environment
Clone autowarefoundation/autoware and move to the directory.
```
git clone https://github.com/autowarefoundation/autoware.git
cd autoware
```

# Ansible Collection - autoware.dev_env

This collection contains the playbooks to set up the development environment for Autoware.

## Set up a development environment

### Ansible installation
[source](https://github.com/autowarefoundation/autoware/tree/main/ansible#ansible-installation
)
```bash
# Remove apt installed ansible (In Ubuntu 22.04, ansible the version is old)
sudo apt-get purge ansible

# Install pipx
sudo apt-get -y update
sudo apt-get -y install pipx

# Add pipx to the system PATH
python3 -m pipx ensurepath

# Install ansible
pipx install --include-deps --force "ansible==6.*"
```

### Install ansible collections

This step should be repeated when a new playbook is added.

```bash
cd ~/autoware # The root directory of the cloned repository
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
```
# Build Tools
[source](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/build_tools#manual-installation)

This role installs build tools for building Autoware.

## Tools

- ccache

## Inputs

## Manual Installation

```bash
# Update package lists
sudo apt-get update

# Install ccache
sudo apt-get install -y ccache
```
# Set up ROS2
[source](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
)
## Set locale
Make sure you have a locale which supports UTF-8. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
## Setup Sources

You will need to add the ROS 2 apt repository to your system.

First ensure that the Ubuntu Universe repository is enabled.
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Now add the ROS 2 GPG key with apt.
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Then add the repository to your sources list.
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.
```
### Install ROS 2 packages

Update your apt repository caches after setting up the repositories.
```bash
sudo apt update
```
ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.
```bash
sudo apt upgrade
```
Warning

Due to early updates in Ubuntu 22.04 it is important that systemd and udev-related packages are updated before installing ROS 2. The installation of ROS 2’s dependencies on a freshly installed system without upgrading can trigger the removal of critical system packages.

Please refer to ros2/ros2#1272 and Launchpad #1974196 for more information.

Desktop Install (Recommended): ROS, RViz, demos, tutorials.
```bash
sudo apt install ros-humble-desktop-full
```
ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.
```bash
sudo apt install ros-humble-ros-base
```
Development tools: Compilers and other tools to build ROS packages
```bash
sudo apt install ros-dev-tools
```
### Environment setup
#### Sourcing the setup script

Set up your environment by sourcing the following file.
```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```
#### Try some examples
Talker-listener

If you installed ros-humble-desktop above you can try some examples.

In one terminal, source the setup file and then run a C++ talker:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```
In another terminal source the setup file and then run a Python listener:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```
### Uninstall

If you need to uninstall ROS 2 or switch to a source-based install once you have already installed from binaries, run the following command:
```bash
sudo apt remove ~nros-humble-* && sudo apt autoremove
```
You may also want to remove the repository:
```bash
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt autoremove
# Consider upgrading for packages previously shadowed.
sudo apt upgrade
```

# devel
[source](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/dev_tools#manual-installation
)

This role installs optional development tools for Autoware.

## Tools

- Git LFS
- pre-commit
- clang-format
- Go
- PlotJuggler

## Inputs

| Name                            | Required | Description                             |
| ------------------------------- | -------- | --------------------------------------- |
| pre_commit_clang_format_version | true     | The version of clang-format to install. |
| rosdistro                       | true     | The ROS distribution.                   |

## Manual Installation

```bash
# For the environment variables
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

sudo apt-get update

sudo apt install python3-pip
sudo apt-get install -y golang
sudo apt-get install -y ros-${rosdistro}-plotjuggler-ros
sudo apt-get install -y git-lfs

# Setup Git LFS
git lfs install

pip3 install pre-commit
pip3 install clang-format==${pre_commit_clang_format_version}
```

# Role: gdown
[source](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/gdown#manual-installation)

This role installs gdown to download files from CMakeLists.txt.

## Inputs

None.

## Manual Installation

```bash
# Install gdown to download files from CMakeLists.txt
pip3 install gdown
```
# geographiclib
[source](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/geographiclib#manual-installation
)
## Inputs

None.

## Manual Installation

```bash
sudo apt install geographiclib-tools

# Add EGM2008 geoid grid to geographiclib
sudo geographiclib-get-geoids egm2008-1
```

# pacmod
[source](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/pacmod#manual-installation
    )

This role sets up the prerequisites to install pacmod3_msgs following [this page](https://github.com/astuff/pacmod3_msgs#installation) and [this page](https://github.com/astuff/docker-builds/blob/e9f563ef04b0578ba0b545711ea3e41b20a51d27/ros2/foxy/ros-core/Dockerfile).

## Inputs

| Name      | Required | Description     |
| --------- | -------- | --------------- |
| rosdistro | true     | The ROS distro. |

## Manual Installation

For Universe, the `rosdistro` variable can also be found in:
[../../playbooks/universe.yaml](../../playbooks/universe.yaml)

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# Taken from https://github.com/astuff/pacmod3#installation
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-${rosdistro}-pacmod3
```

# rmw_implementation

This role sets up ROS 2 RMW implementation following [this page](https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html).

## Inputs

| Name               | Required | Description         |
| ------------------ | -------- | ------------------- |
| rosdistro          | true     | The ROS distro.     |
| rmw_implementation | true     | RMW implementation. |

## Manual Installation
[source](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/rmw_implementation#manual-installation)

For Universe, the `rosdistro` and `rmw_implementation` variable can also be found in:
[../../playbooks/universe.yaml](../../playbooks/universe.yaml)

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# For details: https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html
sudo apt update
rmw_implementation_dashed=$(eval sed -e "s/_/-/g" <<< "${rmw_implementation}")
sudo apt install ros-${rosdistro}-${rmw_implementation_dashed}

# (Optional) You set the default RMW implementation in the ~/.bashrc file.
echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=${rmw_implementation}" >> ~/.bashrc
```
# Installing CUDA Toolkit on WSL2
[source](https://developer.nvidia.com/cuda-12-3-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=WSL-Ubuntu&target_version=2.0&target_type=deb_local
)

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-wsl-ubuntu.pin
sudo mv cuda-wsl-ubuntu.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda-repo-wsl-ubuntu-12-3-local_12.3.0-1_amd64.deb
sudo dpkg -i cuda-repo-wsl-ubuntu-12-3-local_12.3.0-1_amd64.deb
sudo cp /var/cuda-repo-wsl-ubuntu-12-3-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-3
```

# Installing Build Essentials
```bash
sudo apt update
sudo apt install build-essential
gcc -version
```

# Setting up LD_PATH
```bash
export LD_LIBRARY_PATH=/usr/local/cuda-12.3/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.3/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export PATH=/usr/local/cuda-12.3/bin${PATH:+:${PATH}}
```
# Installing CUDNN9
```bash
sudo apt install zlib1g
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cudnn9-cuda-12
sudo apt install -y libcudnn9-samples
```

```bash
# https://forums.developer.nvidia.com/t/freeimage-is-not-set-up-correctly-please-ensure-freeimae-is-set-up-correctly/66950/2
sudo apt-get install libfreeimage3 libfreeimage-dev

find /usr -name "mnistCUDNN" -type d
# cd the output of the upper command
sudo make
./mnistCUDNN
# you should see Test passed!
```

# Installing TensorRT
[source](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/tensorrt#manual-installation)

```bash
# For the environment variables
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

sudo apt-get install -y \
libcudnn8=${cudnn_version} \
libnvinfer8=${tensorrt_version} \
libnvinfer-plugin8=${tensorrt_version} \
libnvparsers8=${tensorrt_version} \
libnvonnxparsers8=${tensorrt_version} \
libcudnn8-dev=${cudnn_version} \
libnvinfer-dev=${tensorrt_version} \
libnvinfer-plugin-dev=${tensorrt_version} \
libnvinfer-headers-dev=${tensorrt_version} \
libnvinfer-headers-plugin-dev=${tensorrt_version} \
libnvparsers-dev=${tensorrt_version} \
libnvonnxparsers-dev=${tensorrt_version}

sudo apt-mark hold \
libcudnn8 \
libnvinfer8 \
libnvinfer-plugin8 \
libnvparsers8 \
libnvonnxparsers8 \
libcudnn8-dev \
libnvinfer-dev \
libnvinfer-plugin-dev \
libnvparsers-dev \
libnvonnxparsers-dev \
libnvinfer-headers-dev \
libnvinfer-headers-plugin-dev
```
verify installation:
```bash
nvcc --version
```

# qt5ct_setup Ansible role
[source](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/qt5ct_setup#readme)

## Overview

The `qt5ct_setup` Ansible role automates the configuration of the `qt5ct` environment for Autoware.
It won't affect the system-wide configuration.

## Installation

Follow the instructions below to **install** or **update** the custom theme for `RViz2` in Autoware.

> **Important:** Both commands must be run when you want to update the theme.

```bash
cd ~/autoware # The root directory of the cloned repository
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
ansible-playbook autoware.dev_env.install_rviz_theme  --ask-become-pass
```

## How to use the custom theme in RViz2

To apply a custom theme to RViz2, you can use the `qt5ct` platform theme. Follow these steps to ensure that the `QT_QPA_PLATFORMTHEME` environment variable is set correctly for your RViz2 instance when used with Autoware.

### Manual setup for running RViz2

Before running RViz2 manually, set the `QT_QPA_PLATFORMTHEME` environment variable to `qt5ct`.
This ensures that the custom theme settings are applied.

```bash
export QT_QPA_PLATFORMTHEME=qt5ct
```

Then, start RViz2 as usual.

```bash
rviz2
```

### Automatic setup in Autoware

In Autoware, the `QT_QPA_PLATFORMTHEME` environment variable is automatically set within the main [autoware.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/autoware.launch.xml) file.
Therefore, you do not need to manually set this environment variable when launching Autoware.

In the `autoware.launch.xml` file, RViz2 is configured with the following `<node>` element:

```xml
<node
  pkg="rviz2"
  exec="rviz2"
  name="rviz2"
  output="screen"
  args="-d $(var rviz_config) -s $(find-pkg-share autoware_launch)/rviz/image/autoware.png"
  if="$(var rviz)"
  respawn="$(var rviz_respawn)">
  <env name="QT_QPA_PLATFORMTHEME" value="qt5ct"/>
</node>
```

This configuration automatically sets the `QT_QPA_PLATFORMTHEME` to `qt5ct` when RViz2 is launched as part of Autoware.
It also includes additional options such as respawn behavior and custom RViz2 configurations.

# Installing Autoware

1. Clone the Autoware Universe repository.

```bash
git clone git@github.com:autowarefoundation/autoware.git
```
2.Move to the workspace directory.
```bash
cd autoware 
mkdir src 
```
3. Import Autoware and Simulator dependencies.
```bash
vcs import src < autoware.repos  
vcs import src < simulator.repos
```
4. Install dependencies for Autoware Universe.

```bash
source /opt/ros/humble/setup.bash
sudo apt update && sudo apt upgrade
rosdep update
rosdep install -iry --from-paths src --rosdistro $ROS_DISTRO
```
Run below `setup-dev-env.sh` just to check if all required tools installed as it should be it is important to answer the both of the following questions with `No`, because we install them manaullay in the previouse steps. also running this script set environment path variables. 
if all things goes well, you should see `complete` meesage at the end.
```bash
./setup-dev-env.sh
```
```bash
Install NVIDIA libraries? [y/N]: n
[Warning] Should the ONNX model files and other artifacts be downloaded alongside setting up the development environment.
Download artifacts? [y/N]: n
```
6. verify installation of grid_map packages

before building the workspace, make sure `gird_map` packages installed properly,
they should be installed during `rosdep update` and `rosdep install ...` commands.
```bash
#veryfing grid_map dependencies installations 
ros2 pkg list | grep grid_map

# output should be as follows
# autoware_grid_map_utils
# autoware_occupancy_grid_map_outlier_filter
# autoware_probabilistic_occupancy_grid_map
# grid_map_core
# grid_map_costmap_2d
# grid_map_cv
# grid_map_msgs
# grid_map_pcl
# grid_map_ros
```
If above dependencies were not installed, you should repeat from step 1 
and specifically step 3.
`DO NOT` Install these pacakges using `sudo apt-get install ros-$ROS_DISTRO-grid-map`;
```
note:
You shall find the `grid_map*` packages in `/opt/ros/humble/include/grid_map*`
```
```
note: 
If the `grid_map*` deps were not installed automatically, 
you may want to [build grid_map manually by your self](https://github.com/anybotics/grid_map)(Its not recommended to do it manaullay), in this case, you should run the sutup.sh script and add builds to the PATH. 
because dependent Autoware packages, look the path for these dependencies.
current compatiable version for grid_map_core is `2.0.1`.
```
7. Build the workspace.
It is important to have stable internet connection before running the following command, because some modules needs to clone their dependent repositories during the build process. 

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# there would be at least 385 packages to build, including simpulators.repos
```

# Running test scenarios modules
if you see the error ` ...your system is not enough powerfull to run simulation at specified frame rate ... `
, lowering frame rate (from 30.0 to about 10.0) and incrising initialize_duration (from 30 to about 100)
will help to run simulations.

```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py \
architecture_type:=awf/universe/20230906 \
record:=false \
scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample.yaml' \
sensor_model:=sample_sensor_kit \
vehicle_model:=sample_vehicle \
global_frame_rate:=10.0 \
initialize_duration:=100
```

```bash
# TODO: add instructions for developing custom test scenarios
# https://github.com/tier4/scenario_simulator_v2/blob/master/docs/user_guide/QuickStart.md
# https://autowarefoundation.github.io/autoware-documentation/main/tutorials/scenario-simulation/planning-simulation/scenario-test-simulation/
```
# Running random test scenarios
```bash
# TODO: add instructions for executing and developing random test scenarios
# https://github.com/tier4/scenario_simulator_v2/blob/master/docs/user_guide/random_test_runner/QuickStart.md```
# https://autowarefoundation.github.io/autoware-documentation/main/tutorials/scenario-simulation/planning-simulation/random-test-simulation/
```