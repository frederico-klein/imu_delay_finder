#

    ./fake_imu.py -> the reference "IMU"
    rosrun imu_dekay_finder _topic:=imu2 _lag_delay=.14 _pub_delay:=0.05
    ./imu_delay_finder_node.py

this prints the right value.
