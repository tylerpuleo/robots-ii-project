#!/usr/bin/env python
#from ..world.nodes import car
import roslib; roslib.load_manifest('car_controller')
import rospy as rp
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
def laser_cb(msg):
    print "HI"
    

    if msg.ranges[90] < 5:
        cmd_vel_pub.publish(angular=Vector3(z=1))
    else:
        cmd_vel_pub.publish(linear=Vector3(x=1))
    print msg.ranges[90]

def main():
    rp.init_node('robot_0', anonymous=True)
    rp.Subscriber('/robot_0/base_scan', LaserScan, laser_cb)

    global cmd_vel_pub
    cmd_vel_pub = rp.Publisher('/car_1/velocity', Twist, queue_size=1)

        
    print "hey"
    rp.spin()
if __name__ == '__main__':
    try:
        main()
    except rp.ROSInterruptException:
        pass