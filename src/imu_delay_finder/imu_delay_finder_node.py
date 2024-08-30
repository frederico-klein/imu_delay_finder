#!/usr/bin/env python3

from scipy import signal
from scipy.interpolate import interp1d

import numpy as np
import rospy
from sensor_msgs.msg import Imu 
from std_msgs.msg import Float64
from collections import deque

sampling_rate = 100.0
seconds_to_record = 1.5

LIST_SIZE= int(sampling_rate*seconds_to_record) ## if it is 100Hz, then it will be 1 second

imu_list1 = deque([], maxlen=LIST_SIZE)
imu_list2 = deque([], maxlen=LIST_SIZE)

def callback(some_deque : deque, imu_msg : Imu, name: str) -> None:
    xy = (imu_msg.header.stamp.to_sec(), imu_msg.angular_velocity.x)
    rospy.logdebug(name+" "+str(xy))
    some_deque.append(xy)


rospy.init_node("find_delay_between_imus")

rospy.Subscriber("imu1", Imu, callback=lambda msg: callback(imu_list1,msg,"1") )
rospy.Subscriber("imu2", Imu, callback=lambda msg: callback(imu_list2,msg,"2") )

r = rospy.Rate(10)

pub = rospy.Publisher("d", Float64)
pub1= rospy.Publisher("d_opt", Float64)

#cumulative_distribution = []
#num_frames = 1

while not rospy.is_shutdown() :
    
    if len(imu_list1) < imu_list1.maxlen or len(imu_list2) < imu_list2.maxlen:
        r.sleep()
        continue
    rospy.logdebug(f"time 1 {imu_list1[-1][0]}, time 2: {imu_list2[-1][0]}")

    #rospy.loginfo(imu_list1)
    #rospy.loginfo(imu_list2)


    t1, y1 = zip(*imu_list1)

    t2, y2 = zip(*imu_list2)

    t1, y1 = np.array(t1), np.array(y1)
    ## but i only want the first listsize, right?
    t2, y2 = np.array(t2), np.array(y2)

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
    new_time = np.linspace(start_time, end_time, int((end_time - start_time) * sampling_rate))

    # Linear interpolation for resampling
    interp1 = interp1d(t1, y1, kind='linear')
    interp2 = interp1d(t2, y2, kind='linear')

    # Resample both signals
    y1_ = interp1(new_time)
    y2_ = interp2(new_time)


    #y1_ = np.array(imu_list1)
    #y2_ = np.array(imu_list2)
    #mode = "same"
    #mode = "valid"
    mode = "full"
    correlation = signal.correlate(y1_, y2_, mode=mode, method='direct')
    ## lets penalize higher value delays

    #for i in range(len(correlation)):
    #    correlation[i] *= 1./((i/LIST_SIZE-1.)**2./200+1.)

    lags = signal.correlation_lags(y1_.size, y2_.size, mode=mode)
    found_delay = lags[np.argmax(correlation)]
    #found_delay = find_delay.find_delay(list(imu_list1), list(imu_list2), freq_array_1=100, freq_array_2=100, compute_envelope=False, verbosity=0)
    #rospy.logdebug(found_delay)
    #rospy.loginfo(f"i think this is the index, {found_delay}, so this should be the lag delay {found_delay/sampling_rate}[s],\n\t\t\t but combined with the publication lag it should be something like {found_delay/sampling_rate + publication_lag}")
    val = Float64()

    #print(correlation.size)

    print(f"idk:{-found_delay/sampling_rate}")
    #val.data = found_delay/sampling_rate
    val.data = -(found_delay)/sampling_rate ## draw the graphs and you will see why
    #val.data = (np.argmax(correlation)-LIST_SIZE)/sampling_rate

    #if len(cumulative_distribution) == 0:
    #    cumulative_distribution = correlation
    #else:

    #    cumulative_distribution += correlation/num_frames
    #    num_frames+=1

    #val_opt = Float64()
    
    #val_opt.data = (np.argmax(cumulative_distribution)-LIST_SIZE)/sampling_rate
    #pub1.publish(val_opt)
    pub.publish(val)
    r.sleep()
    #exit()

