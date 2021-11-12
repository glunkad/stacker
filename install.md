```sudo apt update
&& sudo apt upgrade - y
&& apt install -y \
curl \
git \
gnupg2 \
Isb-release \
&& sudo locale-gen en_us en_US.UTF-8
&& sudo update - Locale LC ALL=en US.UTF-8 LANG=en US.UTF-8
&& echo "export LANG=en US.UTF-8" >> -/.bashrc
&& sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(isb_release -sc) main"> /etc/apt/sources.list.d/ros-latest.list'
&& curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc sudo apt-key add 1
&& sudo apt update
&& sudo apt install -y
ros-noetic-desktop-full \
&& echo "source /opt/ros/noetic/setup.bash" >> -/.bashre
&& source /opt/ros/noetic/setup.bash
&& sudo apt install -y \
build-essential \
python3-rosdep \
python3-rosinstall \
python3- rosinstall-generator \
python3-wstool
python3-rosdep
&& sudo rosdep init
&& rosdep update
&& sudo apt install -y
ninja-build \
exiftool \
python3-empy \
python3-tont \
python3.numpy \
python3-yant \
python3-dev \
python3-pip \
ninja-build \
Protobuf-compiler \
libeigen3-dev \
genromfs \
libignition-rendering3 \
&& pip3 install --user
pandas \
jinja2 \
pyserial \
cerberus \
pyulog \
numpy \
tont \
pyquaternion \
kconfiglib \
packaging \
jsonschema \
&& sudo apt install -y \
gstreamer1.0-plugins-bad \
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
gstreamer 1.0-plugins-bad \
gstreamer1.0-plugins-base \
gstreamer1.0-plugins-good \
gstreamer 1.0-plugins-ugly \
libeigen3.dev \
libgstreamer.plugins-base1.o-dev \
libgstreamer1.0-dev \
Libprotobuf-dev \
libprotoc-dev \
libxml2-utils \
Protobuf-compiler \
python-jinja2 \
python3-catkin-tools \
python3-osrf-pycommon \
python3-rosinstall-generator \
python3-rospkg \
&& mkdir -p ~/catkin_ws/src \
&& cd -/catkin_ws \
&& catkin init \
&& wstool init src \
&& rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall \
&& rosinstall_generator upstream mavros - rosdistro kinetic | tee -a /tmp/mavros.rosinstall \
&& wstool merge -t src/tmp/mavros.ros install \
&& wstool update -t srcj4 \
&& rosdep install .-from-paths src -- ignore-src .y \
&& sudo /src/mavros/mavros/scripts/install_geographiclib_datasets.sh \
&& catkin build \
&& source devel/setup.bash \
&& cd  
&& sudo usermoda dialout $USER
&& sudo apt-get remove y
modermanager
&& wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGround Control. AppImage
&& chrod +x ./QGround Control. AppImage
&& cd-/catkin_ws/src
&& git clone https://github.com/PX4/PX4 - Autopilot.git -recursive
&& cd PX4 - Autopilot/
&& DONT_RUN=1 make px4_sitl_default gazebo
&& pip install user
px4 tools
pymavlink
```
