#encoding=utf-8
#dump a bag into csv.
import sys
import rospy
import rosbag

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

slam_topic_name = "/vins_estimator/odometry"
gps_topic_name = "/mavros/global_position/global"
imu_topic_name = "/imu_stereo"
mag_topic_name = "/mavros/imu/mag"




def _main_():
    print 'usage: python dumpbag.py [bag_path]'
    bag = rosbag.Bag(sys.argv[1])
    with open('output.csv','w+') as f:
        for topic, msg, t in bag.read_messages(topics=[slam_topic_name, gps_topic_name,imu_topic_name,mag_topic_name]):
            if(topic == slam_topic_name):
                pos = msg.pose.pose.position
                orient = msg.pose.pose.orientation
                writelist = ['S',msg.header.stamp,pos.x,pos.y,pos.z,orient.x,orient.y,orient.z,orient.w]
                f.write(','.join(map(str,writelist))+'\n')
            if(topic == gps_topic_name):
                writelist = ['G',msg.header.stamp,msg.longitude,msg.latitude,msg.altitude]
                f.write(','.join(map(str,writelist))+'\n')
            if(topic == imu_topic_name):
                gyro = msg.angular_velocity
                acc = msg.linear_acceleration
                writelist = ['I',msg.header.stamp,gyro.x,gyro.y,gyro.z,acc.x,acc.y,acc.z]
                f.write(','.join(map(str,writelist))+'\n')
            if(topic == mag_topic_name):
                mag = msg.magnetic_field
                writelist = ['M',msg.header.stamp,mag.x,mag.y,mag.z]
                f.write(','.join(map(str,writelist))+'\n')
    bag.close()

if __name__ == '__main__':
    _main_()
