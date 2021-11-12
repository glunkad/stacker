```
sudo  apt update \
  && sudo apt upgrade -y \
  && sudo apt install -y \
    curl \
    git \
    gnupg2 \
    lsb-release \
  && sudo locale-gen en_US en_US.UTF-8 \
  && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && echo "export LANG=en_US.UTF-8" >> ~/.bashrc \
  && sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
  && sudo apt update \
  && sudo apt install -y \
    ros-noetic-desktop-full \
  && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
  && source /opt/ros/noetic/setup.bash \
  && sudo apt install -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-rosdep \
  && sudo rosdep init \
  && rosdep update \
  && sudo apt install -y \
	  ninja-build \
	  exiftool \
	  python3-empy \
	  python3-toml \
	  python3-numpy \
	  python3-yaml \
	  python3-dev \
	  python3-pip \
	  ninja-build \
	  protobuf-compiler \
	  libeigen3-dev \
	  genromfs \
    libignition-rendering3 \
  && pip3 install --user \
	  pandas \
	  jinja2 \
	  pyserial \
	  cerberus \
	  pyulog \
	  numpy \
	  toml \
	  pyquaternion \
    kconfiglib \
    packaging \
    jsonschema \
  && sudo apt install -y \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    libeigen3-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libprotobuf-dev \
    libprotoc-dev \
    libxml2-utils \
    protobuf-compiler \
    python-jinja2 \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-rosinstall-generator \
    python3-rospkg \
  && mkdir -p ~/catkin_ws/src \
  && cd ~/catkin_ws \
  && catkin init \
  && wstool init src \
  && rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall \
  && rosinstall_generator --upstream mavros --rosdistro kinetic | tee -a /tmp/mavros.rosinstall \
  && wstool merge -t src /tmp/mavros.rosinstall \
  && wstool update -t src -j4 \
  && rosdep install --from-paths src --ignore-src -y \
  && sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh \
  && catkin build \
  && source devel/setup.bash \
  && cd ~ \
  && sudo usermod -a -G dialout $USER \
  && sudo apt-get remove -y \
    modemmanager \
  && wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage \
  && chmod +x ./QGroundControl.AppImage \
  && cd ~/catkin_ws/src \
  && git clone https://github.com/PX4/PX4-Autopilot.git --recursive \
  && cd ~/catkin_ws \
  && catkin build \
  && pip install --user \
    px4tools \
    pymavlink \
    
    # Add these lines to your ~/.bashrc 
   source ~/catkin_ws/src/PX4-Autopilot/Tools/setup_gazebo.bash ~/catkin_ws/src/PX4-Autopilot/ ~/catkin_ws/src/PX4-Autopilot/build/px4_sitl_default >> /dev/null 
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/PX4-Autopilot 
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/PX4-Autopilot/Tools/sitl_gazebo
```
