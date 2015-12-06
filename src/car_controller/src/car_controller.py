#!/usr/bin/env python
#from ..world.nodes import car
import roslib; roslib.load_manifest('car_controller')
import rospy as rp
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import turtlesim.msg
"""
def handle_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),tf.transformations.quaternion_from_euler(0, 0, msg.theta),rp.Time.now(),"car_1","world")
"""
def laser_cb(msg):

    if msg.ranges[90] < 2:
        cmd_vel_pub.publish(angular=Vector3(z=1))
    else:
        if msg.ranges[45] < 2 and msg.ranges[135] > 2:
            cmd_vel_pub.publish(linear=Vector3(x=3))
            cmd_vel_pub.publish(angular=Vector3(z=1))
        elif msg.ranges[135] < 2 and msg.ranges[45] > 2:
            cmd_vel_pub.publish(linear=Vector3(x=3))
            cmd_vel_pub.publish(angular=Vector3(z=-1))
        else:
            cmd_vel_pub.publish(linear=Vector3(x=3))

def main():
    rp.init_node('robot_0', anonymous=True)
    rp.Subscriber('/robot_0/base_scan', LaserScan, laser_cb)
    #rp.Subscriber('/robot_0/base_pose_ground_truth',turtlesim.msg.Pose,handle_pose)

    global cmd_vel_pub
    cmd_vel_pub = rp.Publisher('/car_1/velocity', Twist, queue_size=1)

    rp.spin()
if __name__ == '__main__':
    try:
        main()
    except rp.ROSInterruptException:
        pass