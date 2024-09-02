# MoonSim
## 初めに
このプログラムは，４脚 ８脚ロボットを，gazebo ignition を使って動かすためのプログラムです．
動作環境は以下です．
- [Ubuntu 22.04]()
- [ROS2(Robot Operation System 2) Humble](https://docs.ros.org/en/humble/index.html)
- [GZ Fortress (LTS)](https://gazebosim.org/docs/latest/ros_installation)

## 事前準備
### ROS2 Humble のインストール 
[公式サイト](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Gazebo Fortress のインストール
Gazeboをインストールします。以下のリンクに沿って、対応しているバージョンを入れてください。
https://gazebosim.org/docs/latest/ros_installation

公式がサポートしているバージョンをインストールする場合は、以下のコマンドを打つだけです。
```bash
sudo apt-get install ros-humble-ros-gz
```

その他，必要なものをインストールします．
ros control, python3, とか，要check
```bash
sudo apt install ros-humble-ign-ros2-control
sudo apt install python3
```

## プログラムコンパイル
このgit をクローンします．
```bash
git clone https://github.com/amby-1/MoonSim.git
```

フォルダ内部で，コンパイルして下さい
```bash
cd MoonSim
colcon build
source install/setup.bash
```
(git に上げたファイルが過不足ないのかはまだ未検証)
いろいろエラーが出た場合は，指示に従って，いろいろ入れてください（すみません）

## プログラム実行
### 4Leg robot 
２つターミナルを立ち上げてください．１つ目でgazeboをたちあげます
```bash
cd MoonSim
source install/setup.bash
ros2 launch robot_gazebo_ros2_control trajectory_controller_2.launch
```
これで，Gazeboが立ち上がり，４脚ロボットが出てきます．
エラーが出なければ正常です．ワーニングは出ます．
rvizも立ち上がるようにしてますが，設定はちゃんとしてないので，矢印だけ出てきます．

２つ目のターミナルで，ジョイントに位置指令を送り続けるノードを立ち上げます．
```bash
cd MoonSim
source install/setup.bash
ros2 run test_controller talker
```
うまくいって入れば，４つの脚が同期して上下に動くと思います．

gazebo の情報についても、新しいターミナルで以下のコマンドを打つことで、ROS と同じようにトピック情報を見ることができます。
```bash
ign topic -l
ign topic -e -t /world/empty/clock
```
### 8Leg robot 
２つターミナルを立ち上げてください．１つ目でgazeboをたちあげます
```bash
cd MoonSim
source install/setup.bash
ros2 launch robot_gazebo_ros2_control trajectory_controller_8leg.launch
```
これで，Gazeboが立ち上がり，8脚ロボットが出てきます．
エラーが出なければ正常です．ワーニングは出ます．
rvizも立ち上がるようにしてますが，設定はちゃんとしてないので，矢印だけ出てきます．

２つ目のターミナルで，ジョイントに位置指令を送り続けるノードを立ち上げます．
```bash
cd MoonSim
source install/setup.bash
ros2 run test_controller talker_8leg
```
うまくいって入れば，8つの脚が少し位相遅れしながら上下に動くと思います．



## プログラムの説明
TODO 






