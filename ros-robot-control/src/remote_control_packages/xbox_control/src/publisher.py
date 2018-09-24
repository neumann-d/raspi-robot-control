#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import xbox
import time
from motor_control_msg.msg import Motor

def init():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('xbox_control_publisher', anonymous=True)

    # Run publisher
    pub = rospy.Publisher("motor", Motor, queue_size=1)

    msg = Motor()
    msg.command = 0
    msg.velocity_left = 0
    msg.velocity_right = 0

    rospy.loginfo(rospy.get_caller_id() + ": Publisher started.")

    joy = xbox.Joystick()
    rospy.loginfo(rospy.get_caller_id() + ": Xbox controller started. Press a button during 3 seconds to activate it!")
    left_stick_deadzone = 0.1

    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        pressed = False
        velocity_left = 0
        velocity_right = 0
        if joy.leftTrigger():
            rospy.loginfo(rospy.get_caller_id() + ": Xbox joy.leftTrigger() " + str(joy.leftTrigger()))
            msg.command = 1
            velocity_left = max(-100, velocity_left - 100.0 * joy.leftTrigger())
            velocity_right =  max(-100, velocity_right - 100.0 * joy.leftTrigger())
            pressed = True
        if joy.rightTrigger():
            rospy.loginfo(rospy.get_caller_id() + ": Xbox joy.rightTrigger() " + str(joy.rightTrigger()))
            msg.command = 1
            velocity_left = min(100, velocity_left + 100.0 * joy.rightTrigger())
            velocity_right =  min(100, velocity_right + 100.0 * joy.rightTrigger())
            pressed = True
        if joy.leftX():
            rospy.loginfo(rospy.get_caller_id() + ": Xbox joy.leftX() " + str(joy.leftX()))
            # reduce left speed, if stick goes to right and vice versa
            if joy.leftX() > left_stick_deadzone:
                velocity_left = velocity_left - velocity_left * joy.leftX()
            if joy.leftX() < -left_stick_deadzone:
                velocity_right = velocity_right + velocity_right * joy.leftX()
            pressed = True
        if joy.B():
            rospy.loginfo(rospy.get_caller_id() + ": Xbox joy.B()")
            msg.command = 0
            msg.velocity_left = 0
            msg.velocity_right = 0
            pressed = True
        if pressed:
            msg.velocity_left = velocity_left
            msg.velocity_right = velocity_right
            pub.publish(msg)
        rate.sleep()

    # release Xbox controller
    joy.close()

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    init()
