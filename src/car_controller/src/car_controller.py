#!/usr/bin/env python
import sensingModelAndMap
import rospy as rp
def laser_cb(msg):
    
    
def main():

    plt.ion()

    rp.init_node('racecar', anonymous=True)

    rp.Subscriber('/robot/wall_door_sensor', String, laser_cb)
    global cmd_vel_pub
    cmd_vel_pub = rp.Publisher('/robot/cmd_vel', Twist, queue_size=10)

    rp.spin()

if __name__ == '__main__':
    try:
        main()
    except rp.ROSInterruptException:
        pass