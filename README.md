# 卒業研究

## 依存パッケージのインストール
##### ROSのインストール
Ubuntu20.04のときにはros <a href="http://wiki.ros.org/noetic/Installation/Ubuntu">noetic</a>

Ubuntu18.04のときにはros <a href="http://wiki.ros.org/melodic/Installation/Ubuntu">melodic</a>
##### vispのインストール
```
mkdir -p  ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/tsuchidashinya/graduation_paper/
cd ~/ros_ws/src/graduation_paper/depends_package/visp/
mkdir build
cd build
cmake ..
make
sudo make install
```
##### その他のパッケージのインストールとコンパイル
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

```
Ubuntu20.04のとき、以下はpython3のほうをインストールする
```
sudo apt install python3-catkin-tools
sudo apt install python3-catkin-pkg
sudo apt install python3-osrf-pycommon
```
Ubuntu18.04以下のとき、以下はpython2の方をインストールする
```
sudo apt install python-catkin-tools
sudo apt install python-catkin-pkg
sudo apt install python-osrf-pycommon
```
ここからはどちらも共通
```
cd ~/ros_ws/
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

## プログラムの実行
### シミュレーターの環境セットアップ
```
roslaunch visual_servo_tsuchida setup_simulator.launch
```
```
roslaunch visual_servo_tsuchida moveit.launch sim:=true
```
```
roslaunch visual_servo_tsuchida arm_jog.launch
```
### 実機のロボット環境のセットアップ
```
```
