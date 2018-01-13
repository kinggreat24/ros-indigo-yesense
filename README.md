# ros-indigo-yesense
An Imu package for ros-indigo!

(1)代码组织：
src
  -- yesense.h
  -- yesense.cpp
  -- yesense_node.cpp
  -- baseYesense.h
CMakeLists.txt
package.xml
include

 
其中，yesense.h,yesense.cpp 为封装的串口读取的类;
yesense_node.cpp将读取的数据转化为sensor_msgs/Imu类型的数据并发布;
baseYesense.h定义了串口数据处理以及异常的接口。



(2)安装
该包依赖Qt5,因此如果是indigo版本，默认的Qt的版本为Qt4，需要自行下载Qt5。注意应下载Qt5.6及以下的版本，因为indigo不支持c++11，而Qt5.7及其以上的版本需要c++11。


这里给出一个下载地址：http://download.qt.io/official_releases/qt/5.6/5.6.0/

安装：
a.进入下载完成的目录（我放在~/Downloads/）
cd ~/Download　　
b.给该文件增加可执行的权限
chmod +x qt-opensource-linux-x64-5.6.0.run
c.然后双击该文件,进入图形化安装界面，选择安装地址。

d.在CMakeLists.txt中，将Qt_prefix的值设置为你的安装路径
set(Qt_prefix "/home/XXX/Qt5.6.0/5.6/gcc_64/lib/cmake")


e.cd ~/catkin_ws
f.catkin_make

