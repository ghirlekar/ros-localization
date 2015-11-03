#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from sensor_msgs.msg import Imu


def cb_orientation(msg_orientation, msg_imu):
    msg_imu.header.stamp = msg_orientation.header.stamp
    msg_imu.header.seq = msg_orientation.header.seq
    msg_imu.header.frame_id = msg_orientation.header.frame_id
    msg_imu.orientation.x = msg_orientation.quaternion.x
    msg_imu.orientation.y = msg_orientation.quaternion.y
    msg_imu.orientation.z = msg_orientation.quaternion.z
    msg_imu.orientation.w = msg_orientation.quaternion.w

def cb_angular_velocity(msg_angular_velocity, msg_imu):
    msg_imu.header.stamp = msg_angular_velocity.header.stamp
    msg_imu.header.seq = msg_angular_velocity.header.seq
    msg_imu.header.frame_id = msg_angular_velocity.header.frame_id
    msg_imu.angular_velocity.x = msg_angular_velocity.vector.x
    msg_imu.angular_velocity.y = msg_angular_velocity.vector.y
    msg_imu.angular_velocity.z = msg_angular_velocity.vector.z

def cb_linear_acceleration(msg_linear_acceleration, msg_imu):
    msg_imu.header.stamp = msg_linear_acceleration.header.stamp
    msg_imu.header.seq = msg_linear_acceleration.header.seq
    msg_imu.header.frame_id = msg_linear_acceleration.header.frame_id
    msg_imu.linear_acceleration.x = msg_linear_acceleration.vector.x
    msg_imu.linear_acceleration.y = msg_linear_acceleration.vector.y
    msg_imu.linear_acceleration.z = msg_linear_acceleration.vector.z


if __name__ == '__main__':
    try:
    	rospy.init_node('imu_broadcaster')
        msg_imu = Imu()
    	sub_orientation = rospy.Subscriber('orientation', QuaternionStamped, cb_orientation, msg_imu)
        sub_angular_velocity = rospy.Subscriber('angular_velocity', Vector3Stamped, cb_angular_velocity, msg_imu)
        sub_linear_acceleration = rospy.Subscriber('linear_acceleration', Vector3Stamped, cb_linear_acceleration, msg_imu)
        pub_imu = rospy.Publisher('imu', Imu, queue_size=10)
        r = rospy.Rate(40)

        while not rospy.is_shutdown():
            pub_imu.publish(msg_imu)
            r.sleep()

    except rospy.ROSInterruptException:
        pass
        