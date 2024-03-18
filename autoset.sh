sudo vi /etc/apt/sources.list +%s/kr.archive.ubuntu.com/mirror.kakao.com +wq!
sudo vi /etc/apt/sources.list +%s/security.ubuntu.com/mirror.kakao.com +wq!
sudo vi /etc/apt/sources.list +%s/ports.ubuntu.com/mirror.yuki.net.uk +wq!
sudo add-apt-repository ppa:graphics-drivers/ppa -y
sudo apt update
sudo apt install fonts-noto-cjk-extra gnome-user-docs-ko hunspell-ko ibus-hangul language-pack-gnome-ko language-pack-ko hunspell-en-gb hunspell-en-au hunspell-en-ca hunspell-en-za -y
ibus restart
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop-full -y
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-turtlebot4-desktop -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python3-rosdep -y
sudo rosdep init
rosdep update
sudo apt install ~nros-humble-rqt* -y
sudo apt install python3-colcon-common-extensions ros-humble-image-transport-plugins python3-pip pv -y
sudo apt-get install ros-humble-gazebo-msgs ros-humble-gazebo-plugins ros-humble-gazebo-ros ros-humble-gazebo-ros2-control ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-octomap-ros libgoogle-glog-dev libgflags-dev -y
sudo apt install ros-humble-turtlebot4-desktop ros-humble-turtlebot4-navigation ros-humble-teleop-twist-keyboard ros-humble-irobot-create-msgs ros-humble-turtlebot4-msgs -y
sudo apt install 
printenv | grep -i ROS_DISTRO

echo "# colcon_cd Setting " >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
echo "" >> ~/.bashrc

sudo apt install python3-bloom python3-rosdep fakeroot debhelper dh-python -y
sudo rosdep init
rosdep update



echo "alias eb='gedit ~/.bashrc'" >> ~/.bashrc
echo "alias sb='source ~/.bashrc'" >> ~/.bashrc
echo "alias up='sudo apt update && sudo apt upgrade -y'" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "" >> ~/.bashrc
source ~/.bashrc

echo "# ipfrag Setting " >> ~/.bashrc
echo "# sudo sysctl net.ipv4.ipfrag_time=3" >> ~/.bashrc
echo "# sudo sysctl net.ipv4.ipfrag_high_thresh=134217728" >> ~/.bashrc
echo "" >> ~/.bashrc
eval "$(cat ~/.bashrc | tail -n +10)"
echo "Type 'source ~/.bashrc' to apply settings"

sudo apt install terminator -y
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb

sudo reboot
