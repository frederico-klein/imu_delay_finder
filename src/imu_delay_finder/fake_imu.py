#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

import numpy as np

rospy.init_node("chatter", anonymous=True)

#rospy.loginfo_once(rospy.get_param("/"))

publication_time_delay = rospy.get_param("~pub_delay", 0.0 )
lag_time_delay = rospy.get_param("~lag_delay", 0.0 )
freq = rospy.get_param("freq", 1.0 )
topic_name = rospy.get_param("~topic", "imu1" )

pub = rospy.Publisher(topic_name, Imu)

r = rospy.Rate(100)

while not rospy.is_shutdown():
    imu_msg = Imu()
    t = rospy.Time.now().to_sec()-lag_time_delay
    imu_msg.header.stamp = rospy.Time.now()-rospy.Duration(nsecs=publication_time_delay*1000000)
    imu_msg.angular_velocity.x = np.sin(np.pi *t*freq)
    #imu_msg.angular_velocity.x = np.sin(np.pi *t*freq)+ 1/3.* np.sin(np.pi * t*freq*3)+1/5.* np.sin(np.pi * t*freq*5)
    #imu_msg.angular_velocity.x = np.sin(np.pi *t*freq)+ 1/3.* np.sin(np.pi * t*freq*3)+1/5.* np.sin(np.pi * t*freq*5)+1/7.* np.sin(np.pi * t*freq*7) #+t%2.0
    #imu_msg.angular_velocity.x = -t%1.0+(t-0.5)%1.0 


    imu_msg.angular_velocity.y = np.sin(np.pi * t*freq + np.pi*0.333)
    imu_msg.angular_velocity.z = np.sin(np.pi * t*freq + np.pi*0.666)
    pub.publish(imu_msg)
    r.sleep()
