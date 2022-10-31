## ロボットと部屋の飾り付けをする

### 環境構築
**fetchを動かすための環境構築**<br>
詳細は[jsk-ros-pkg/jsk_robot/jsk_fetch_robot](https://github.com/jsk-ros-pkg/jsk_robot/tree/develop/fetch/jsk_fetch_robot#setup-environment-for-remote-pc)参照

```
mkdir -p catkin_ws/src
cd  catkin_ws/src
wstool init .
wstool set --git jsk-ros-pkg/jsk_robot https://github.com/jsk-ros-pkg/jsk_robot.git -v develop/fetch -y
wstool merge -t . https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_fetch_robot/jsk_fetch_user.rosinstall.$ROS_DISTRO

# (optional): the two lines below are necessary when you want to use roseus_resume
wstool merge -t . https://gist.githubusercontent.com/Affonso-Gui/25518fef9dc7af0051147bdd2a94b116/raw/e3fcbf4027c876329801a25e32f4a4746200ddae/guiga_system.rosinstall
wstool update -t .

# (optional): the two lines below are necessary when you want to use eus10
wget https://raw.githubusercontent.com/jsk-ros-pkg/jsk_roseus/master/setup_upstream.sh -O /tmp/setup_upstream.sh
bash /tmp/setup_upstream.sh -w ../ -p jsk-ros-pkg/geneus -p euslisp/jskeus

source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y -r --from-paths . --ignore-src
cd ../
# (optional): if you want to use roseus_resume, build roseus_resume, too.
catkin build fetcheus jsk_fetch_startup

source devel/setup.bash
```

**飾り付けデモを動かすための環境構築**<br>
```
source catkin_ws/devel/setup.bash
roscd jsk_fetch_startup
cd ../
git clone https://github.com/MiyabiTane/Deco_with_fetch.git
cd catkin_ws
catkin build deco_with_fetch
```

**眉毛デバイスを動かすための環境構築**<br>
1. サーバーの環境構築

    ※docker, docker-composeのインストールがされている必要がある
    ```
    source catkin_ws/devel/setup.bash
    roscd deco_with_fetch
    cd eyebrows_server

    ## webサーバーの環境構築
    $ docker-compose run --rm app /bin/bash
    # npx express-generator
    # npm install
    # exit
    $ docker-compose up

    ## 眉毛アニメーションの環境構築
    $ sudo cp app/route/* src/routes/  # 本ディレクトリtypoしてるので注意
    $ sudo cp app/public/javascripts/* src/public/javascripts
    $ sudo cp app/views/* src/views
    $ sudo cp app/app.js src/app.js
    ```
    ```
    $ docker-compose up
    ```
    して http://localhost:3000/rbrow, http://localhost:3000/lbrow にアクセスできればOK（Google Chrome推奨）

2. 表情選択の環境構築

    ```
    cd catkin_ws/src
    git clone https://github.com/jsk-ros-pkg/jsk_3rdparty.git
    git checkout -b deco_with_fetch f4f06486db34ce274aa779be416865907055e9c3
    cd ../
    catkin build dialogflow_task_executive
    ```

    [facialexpressionoriginal-cphs-377bf1229657.json](https://drive.google.com/file/d/1_nvnwLta4yW7vOffhXcGwdUL6wudqood/view)をダウンロードし、`deco_with_fetch/json`下に置く

    左右の動きにずれが生じる際は`views/lbrow.js`または`views/rbrow.js`中の`var delay_ms`の値をいじる。これを大きくすると動き出しが遅くなる。

**meboを用いた雑談を行うための環境構築**<br>

[apikey.json](https://drive.google.com/file/d/1tAT_WQqCMqvtbM0-CSTomWjwMP4jcOi9/view)をダウンロードし、`deco_with_fetch/json`下に置く

**構文解析を使うための環境構築**<br>
[ros_google_cloud_languageフォルダ](https://github.com/k-okada/jsk_3rdparty/tree/google_nlp/ros_google_cloud_language)を頑張ってダウンロードし（いつかmasterにマージされるかもしれない）、`catkin_ws/src`下に置く。

```
cd catkin_ws/src
catkin build ros_google_cloud_language
source catkin_ws/devel/setup.bash
```

[eternal-byte-236613-4bc6962824d1.json](https://drive.google.com/file/d/1VxniytpH9J12ii9jphtBylydY1_k5nXf/view)をダウンロードし、`deco_with_fetch/json`下に置く


### 実行方法
```
source catkin_ws/devel/setup.bash
roscd deco_with_fetch
cd eyebrows_server
docker-compose up
```
```
rossetmaster fetch1075
rossetip # 133系になっているか確認すること
source catkin_ws/devel/setup.bash
roslaunch deco_with_fetch deco_demo.launch
```
```
rossetmaster fetch1075
rossetip # 133系になっているか確認すること
source catkin_ws/devel/setup.bash
roscd deco_with_fetch
cd euslisp
roseus demo_main.l
(main)
```

### 開発源
飾り付け生成、眉毛デバイス等のソースや詳細は[MiyabiTane/Deco_with_robot](https://github.com/MiyabiTane/Deco_with_robot.git)にあります
