#!/usr/bin/env python3

from scipy import signal
from scipy.interpolate import interp1d

import numpy as np
import rospy
from sensor_msgs.msg import Imu 
from std_msgs.msg import Float64
from collections import deque

sampling_rate = 100.0
seconds_to_record = 2.5

LIST_SIZE= int(sampling_rate*seconds_to_record) ## if it is 100Hz, then it will be 1 second

imu_list1 = deque([], maxlen=LIST_SIZE)
imu_list2 = deque([], maxlen=LIST_SIZE)

def callback(some_deque : deque, imu_msg : Imu, name: str) -> None:
    xy = (imu_msg.header.stamp.to_sec(), imu_msg)
    #rospy.logdebug(name+" "+str(xy))
    some_deque.append(xy)


rospy.init_node("find_delay_between_imus")

rospy.Subscriber(rospy.get_param("~imu1" ,"imu1"), Imu, callback=lambda msg: callback(imu_list1,msg,"1") )
rospy.Subscriber(rospy.get_param("~imu2" ,"imu2"), Imu, callback=lambda msg: callback(imu_list2,msg,"2") )

r = rospy.Rate(5)

pub = rospy.Publisher("d", Float64)

#cumulative_distribution = []
#num_frames = 1

def look_at_signal(t1,t2,y1,y2, new_time):
    
    # Linear interpolation for resampling
    interp1 = interp1d(t1, y1, kind='linear')
    interp2 = interp1d(t2, y2, kind='linear')

    # Resample both signals
    y1_ = interp1(new_time)
    y2_ = interp2(new_time)

    mode = "full"
    correlation = signal.correlate(y1_, y2_, mode=mode, method='direct')
    lags = signal.correlation_lags(y1_.size, y2_.size, mode=mode)
    found_delay = lags[np.argmax(correlation)]

    #print(correlation.size)

    #print(f"idk:{-found_delay/sampling_rate} : {np.sum(correlation)}")
    #val.data = found_delay/sampling_rate
    #val.data = -(found_delay)/sampling_rate ## draw the graphs and you will see why
    return correlation, lags

def explode_imu_msg_list(Iy):
    ang_velocity_x= []
    ang_velocity_y= []
    ang_velocity_z= []
    linear_acceleration_x= []
    linear_acceleration_y= []
    linear_acceleration_z= []

    for iIy in Iy:
        ang_velocity_x.append(iIy.angular_velocity.x)
        ang_velocity_y.append(iIy.angular_velocity.y)
        ang_velocity_z.append(iIy.angular_velocity.z)
        linear_acceleration_x.append(iIy.linear_acceleration.x)
        linear_acceleration_y.append(iIy.linear_acceleration.y)
        linear_acceleration_z.append(iIy.linear_acceleration.z)
        
    return  np.array(ang_velocity_x), np.array(ang_velocity_y), np.array(ang_velocity_z), np.array(linear_acceleration_x), np.array(linear_acceleration_y), np.array(linear_acceleration_z)

coor = deque(maxlen=10)

while not rospy.is_shutdown() :
    
    if len(imu_list1) < imu_list1.maxlen or len(imu_list2) < imu_list2.maxlen:
        r.sleep()
        continue
    rospy.logdebug(f"time 1 {imu_list1[-1][0]}, time 2: {imu_list2[-1][0]}")

    #rospy.loginfo(imu_list1)
    #rospy.loginfo(imu_list2)


    t1, Iy1 = zip(*imu_list1)

    t2, Iy2 = zip(*imu_list2)

    t1 = np.array(t1)
    t2 = np.array(t2)
    
    a1,b1,c1,d1,e1,f1 = explode_imu_msg_list(Iy1)
    a2,b2,c2,d2,e2,f2 = explode_imu_msg_list(Iy2)

    publication_lag = np.mean(t1-t2)
    ## I don't care about the publication delay
    #rospy.loginfo(f" (we don't care about this:) publication_lag: {publication_lag} [s]")


    # Find overlapping time range
    ws = np.argmax([t1[0], t2[0]])
    we = np.argmin([t1[-1], t2[-1]])

    start_time = [t1[0], t2[0]][ws]
    end_time = [t1[-1], t2[-1]][we]

    rospy.logdebug(f"starts_latest: {ws} ends_soonest: {we}")

    # New common time vector for resampling
    #new_time = np.linspace(start_time, end_time, int((end_time - start_time) * sampling_rate))
    new_time = np.linspace(start_time, end_time, 500)
    

    coora, lags = look_at_signal(t1,t2,a1,a2, new_time)
    coorb, lags = look_at_signal(t1,t2,b1,b2, new_time)
    coorc, lags = look_at_signal(t1,t2,c1,c2, new_time)
    coord, lags = look_at_signal(t1,t2,d1,d2, new_time)
    coore, lags = look_at_signal(t1,t2,e1,e2, new_time)
    coorf, lags = look_at_signal(t1,t2,f1,f2, new_time)

    coor.append(coora +coorb +coorc +coord +coore +coorf)

    if len(coor) < coor.maxlen:
        r.sleep()
        continue
    else:
        try:
            cor_all = np.sum(coor,axis=0)
        except:
            for fff in coor:
                print(fff.size)
            #print(coor)
    val = Float64()
    
    #print(correlation.size)
    found_delay = lags[np.argmax(cor_all)]

    #total_delay+=found_delay/sampling_rate
    #num_samples+=1
    print(f"idk:{-found_delay/sampling_rate}")
    #print(f"idk:{-total_delay/num_samples}")
    #val.data = found_delay/sampling_rate
    val.data = -(found_delay)/sampling_rate ## draw the graphs and you will see why



    pub.publish(val)
    r.sleep()
    #exit()

