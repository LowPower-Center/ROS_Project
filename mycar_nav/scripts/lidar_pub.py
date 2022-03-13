import ros
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import lidar_reader as ld
import math


lidar_reader = ld.LidarReader(serial=ld.ser)
lidar_reader.start()
rospy.init_node("lidar_pub")

laser_pub=rospy.Publisher("scan", data_class=LaserScan, queue_size=100)
#初始化雷达数据
laser_data=LaserScan()
laser_data.angle_increment=2.0*math.pi/90
laser_data.angle_min=0
laser_data.angle_max=2*math.pi-laser_data.angle_increment

laser_data.range_min=0.0
laser_data.range_max=ld._MAX_DISTANCE_MM/1000
rospy.sleep(5)
#读取雷达数据并发布
while not rospy.is_shutdown():
    lidar_reader.wait_read()
    laser_data.ranges=lidar_reader.cache
    laser_data.intensities=lidar_reader.intensities
    laser_data.header.stamp=rospy.Time.now()
    laser_data.header.frame_id="laser"
    rpm=lidar_reader.speed
    laser_data.time_increment=float(1.0/(6*rpm))*4
    laser_data.scan_time=laser_data.time_increment*90
    laser_pub.publish(laser_data)
lidar_reader.stop()
print("host down!")

