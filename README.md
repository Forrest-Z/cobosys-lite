 cobosys-lite: 轻量先进的移动机器人操作系统</br>
 ===
     
     
   
功能：
---

    1.支持差分(diff)平台，按照docs开发串口驱动即可立即接入此系统。
    2.支持多种传感器、硬件驱动，目前支持的有：激光雷达Rplidar S1、北通阿修罗游戏手柄。
    3.移植自最先进的ROS的cartographer SLAM算法，系统完全独立。
    4.导航，支持集群。
    
系统要求：
---

    gcc>=5的Linux

安装：
---
    new installed ubuntu16.04.5 compile passed
    
1.安装环境及依赖：
>sudo apt-get update
>sudo apt-get install cmake git
>git clone https://github.com/lyeemax/cobosys-lite.git

2.安装主程序：</br>
>sudo apt-get install libjpeg-dev libfltk1.3-dev libyaml-cpp-dev libgnomecanvas2-dev</br>
>cd cobosys-lite</br>
>mkdir build && cd build </br>
>cmake ..</br>
>make -j12</br>
>sudo make install</br>
>sudo cp  -r /usr/local/lib/libplayer* /usr/lib</br>

3.安装ceres优化库</br>
>sudo apt-get install -y google-mock libboost-all-dev libcairo2-dev libcurl4-openssl-dev libeigen3-dev libgflags-dev libgoogle-glog-dev liblua5.2-dev libsuitesparse-dev ninja-build python-sphinx</br>
>cd extensions/cobotslam</br>
>cd ceres-solver</br>
>mkdir build</br>
>cd build</br>
>cmake .. -G Ninja -DCXX11=ON</br>
>ninja</br>
>sudo ninja install</br>

4.安装protobuf</br>
>VERSION="v3.4.1"</br>
>git clone https://github.com/google/protobuf.git</br>
>cd protobuf</br>
>git checkout tags/${VERSION}</br>
>mkdir build</br>
>cd build</br>
>cmake -G Ninja \-DCMAKE_POSITION_INDEPENDENT_CODE=ON \-DCMAKE_BUILD_TYPE=Release \ -Dprotobuf_BUILD_TESTS=OFF \../cmake </br>
>ninja</br>
>sudo ninja install</br>

5.编译cartoslam</br>
>cd ..</br>
>mkdir build &&cd build </br>
>cmake ..</br>
>make -j12 </br>

6.安装stage仿真（非必须）</br>
>git clone https://github.com/lyeemax/Stage.git</br>
>cd stage</br>
>mkdir build</br>
>cd build</br>
>cmake ..</br>
>make -j12</br>
>sudo make install</br>
>sudo cp  -r /usr/local/lib/libstage*  /usr/lib</br>
>sudo cp  -r /usr/local/lib/stage*  /usr/lib</br>

版权：
---

    此系统基于Player修改而来，基于GNU General Public License
    此系统由'Lyeemax'独立开发
    版权所有人为武汉库柏特科技有限责任公司
关于维护：
---

    本版本迁移至库柏特私有仓库后，不再由'Lyeemax'维护
    如由问题，请联系：13657230818/Wechat:Ruochenpro
致谢：
---

    本系统的完成离不开库柏特的支持信任，感谢张少华博士一直以来对我的支持与关爱，此外特别感谢邹调清、黄威的帮助。


