#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from motor_control_msg.msg import Motor
from L298N import L298N

l298n = None
msg = None

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ": Subscriber callback, command=%i, vel_left=%f,vel_right=%f", 
                data.command, data.velocity_left, data.velocity_right)
    global msg
    msg = data

def update_motor(data):
    rospy.loginfo(rospy.get_caller_id() + ": Motor update, command=%i, vel_left=%f,vel_right=%f", 
                  data.command, data.velocity_left, data.velocity_right)

    if data.command is 0:
        l298n.stop_dc_motors([l298n.DC_Motor_1, l298n.DC_Motor_2])
    else:
        speed_left = None if abs(data.velocity_left) >= 100 else abs(data.velocity_left)
        l298n.run_dc_motors([l298n.DC_Motor_1], forwards=(data.velocity_left >= 0), speed=speed_left)

        speed_right = None if abs(data.velocity_right) >= 100 else abs(data.velocity_right)
        l298n.run_dc_motors([l298n.DC_Motor_2], forwards=(data.velocity_right >= 0), speed=speed_right)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # Init L298N Board
    global l298n
    l298n = L298N.L298N()
    l298n.set_L298N_pins(5, 6, 13, 26)    # Set PINs for controlling the 2 motors (GPIO numbering)
    l298n.set_pwm_frequency([20,20])
    l298n.stop_dc_motors([l298n.DC_Motor_1, l298n.DC_Motor_2])
    rospy.loginfo(rospy.get_caller_id() + ": Motor started.")

    # Run subscriber
    rospy.Subscriber("motor", Motor, callback)
    rospy.loginfo(rospy.get_caller_id() + ": Subscriber started.")

    global msg
    rate = rospy.Rate(2) # 2hz
    while not rospy.is_shutdown():
        if msg is not None:
            update_motor(msg)
            msg = None
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    listener()
