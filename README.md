#

    ./fake_imu.py -> the reference "IMU"
    rosrun imu_delay_finder fake_imu.py _topic:=imu2 _lag_delay=.14 _pub_delay:=0.05
    ./imu_delay_finder_node.py

this prints the right value.


testing with ximus also works


    roslaunch ximu3_ros ximu_feet.launch wait_to_start:=false
    rosrun imu_delay_finder imu_delay_finder_node.py _imu1:=/ximu_talus_r/imu _imu2:=ximu_talus_l/imu


the difference in delay i found was quite high of around 20ms.

![delay zoomed](/delay_image_zoomed.png)
