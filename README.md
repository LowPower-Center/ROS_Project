# ROS_Project
目前有两个工作
+ ros小车slam
+ ros机械臂仿真示教

## 开发环境
+ VScode
+ 主从机的设置
+ 主机和从机hosts文件添加彼此ip记录
+ 主从机终端ros_master_name=自己的主机名
+ 主从机终端ros_master_uri=主机ip:11311



## 小车部分

+ 树莓派4B 8GB版本
+ LDS-006雷达
+ 3s 2200mah电池
+ arduino mega驱动电机
+ 电机和驱动是TB轮趣科技家(贵，不推荐)的
+ 底层驱动在另外的ROS_arduino_bridge仓库


小车的启动文件为slam.launch
+ 整体配置参考autolabor公司文档，地址http://www.autolabor.com.cn/book/ROSTutorials/
+ 配置arduino端口为/dev/arduino,雷达端口为/dev/lidar,端口配置自行查阅搜索引擎
+ 控制器使用xboxones无线控制器蓝牙连接,配置蓝牙参考https://www.labno3.com/2021/03/16/setting-up-xbox-controllers-on-the-raspberry-pi/
+ 雷达驱动使用neato雷达的二代驱动，在源码基础上加入了雷达开启和关闭的函数,雷达选用型号为科沃斯lds-006，通讯协议在github上可以查到


## 机械臂部分

+ arduino nano+nano拓展板
+ 淘宝六轴机械臂 借的
+ usb摄像头


机械臂的urdf为sw导出，关节的参考坐标系设定在连接件的地方


moveit配置整体六个关节，基本实现rviz和真实舵机同步，这一点取决于装配误差
