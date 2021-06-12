## 土田真哉の卒業研究

### 依存パッケージのインストール
##### ROSのインストール
Ubuntu20.04のときにはros<a href="http://wiki.ros.org/noetic/Installation/Ubuntu">noetic</a>

Ubuntu18.04のときにはros<a href="http://wiki.ros.org/melodic/Installation/Ubuntu">melodic</a>
##### vispのインストール
```
cd depends_package/visp/
mkdir build
cd build
cmake ..
make
sudo make install
```
