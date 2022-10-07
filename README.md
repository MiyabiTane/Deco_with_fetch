## ロボットと部屋の飾り付けをする

### 環境構築
**fetchを動かすための環境構築**<br>
詳細は[jsk-ros-pkg/jsk_robot/jsk_fetch_robot](https://github.com/jsk-ros-pkg/jsk_robot/tree/develop/fetch/jsk_fetch_robot#setup-environment-for-remote-pc)参照

```
mkdir -p catkin_ws/src
cd  catkin_ws/src
wstool init .
wstool set --git jsk-ros-pkg/jsk_robot https://github.com/jsk-ros-pkg/jsk_robot.git -y
wstool merge -t . https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_fetch_robot/jsk_fetch_user.rosinstall.$ROS_DISTRO
wstool update -t .
# To use eus10, furuschev script is required.
wget https://raw.githubusercontent.com/jsk-ros-pkg/jsk_roseus/master/setup_upstream.sh -O /tmp/setup_upstream.sh
bash /tmp/setup_upstream.sh -w ../ -p jsk-ros-pkg/geneus -p euslisp/jskeus
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y -r --from-paths . --ignore-src
cd ../
catkin build fetcheus jsk_fetch_startup
source devel/setup.bash
```

**本デモを動かすための環境構築**
```
source catkin_ws/devel/setup.bash
roscd jsk_fetch_startup
cd ../
git clone https://github.com/MiyabiTane/Deco_with_fetch.git
cd catkin_ws
catkin build deco_with_fetch
```

### 開発源
飾り付け生成、眉毛デバイス等のソースや詳細は[MiyabiTane/Deco_with_robot](https://github.com/MiyabiTane/Deco_with_robot.git)にあります
