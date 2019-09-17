#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import time
import math as m
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64,Int32

bot1 = Pose()
bot2 = Pose()
bot3 = Pose()
bot4 = Pose()

bot1o = Pose()
bot2o = Pose()
bot3o = Pose()
bot4o = Pose()

bpub1 = rospy.Publisher('bot1twistmeas',Twist,queue_size = 10)
bpub2 = rospy.Publisher('bot2twistmeas',Twist,queue_size = 10)
bpub3 = rospy.Publisher('bot3twistmeas',Twist,queue_size = 10)
bpub4 = rospy.Publisher('bot4twistmeas',Twist,queue_size = 10)

time=0.0
to=-0.1

def bop1(msg):
    global bot1
    global bpub1
    global bot1o
    if bot1 is not None:
        bot1 = msg
        bot1o = pubtwist(bot1,bot1o,bpub1)

def bop2(msg):
    global bot2
    global bpub2
    global bot2o
    if bot2 is not None:
        bot2 = msg
        bot2o = pubtwist(bot2,bot2o,bpub2)

def bop3(msg):
    global bot3
    global bpub3
    global bot3o
    if bot3 is not None:
        bot3 = msg
        bot3o = pubtwist(bot3,bot3o,bpub3)

def bop4(msg):
    global bot4
    global bpub4
    global bot4o
    if bot4 is not None:
        bot4 = msg
        bot4o = pubtwist(bot4,bot4o,bpub4)

def tick(msg):
    global time
    time = msg.data

def pubtwist(bot,boto,bpub):
    '''
    Function to publish the measured twist
    pixels per second for linear
    radians per second for angular

    bot: pose of the given robot
    boto: old pose of the given robot
    bpub: publishing object
    '''
    global time
    global to
    tw = Twist()
    x = bot.position.x
    y = bot.position.y
    th = 2*m.atan(bot.orientation.z)
    xo = boto.position.x
    yo = boto.position.y
    tho = 2*m.atan(boto.orientation.z)
    try:
        '''
        to counter the discontinuity at pi and -pi
        '''
        if th < m.pi and th > m.pi/2  and tho > -m.pi and tho < -m.pi/2:
            difth = -(4*m.pi + tho - th)
        elif tho < m.pi and tho > m.pi/2  and th > -2*m.pi and th < -m.pi/2:
            difth = (4*m.pi + tho - th)
        else:
            difth = th - tho

        vx = 1000*(x-xo)/float(time-to)
        vy = 1000*(y-yo)/float(time-to)
        wz = 1000*(difth)/float(time-to)
        tw.linear.x = vx
        tw.linear.y = vy
        tw.angular.z = wz
        bpub.publish(tw)
        xo = x
        yo = y 
        tho = th
        to = time
        return bot
    except:
        return bot



def run():
    global bot1
    global bot2
    global bot3
    global bot4
    global bot1o
    global bot2o
    global bot3o
    global bot4o
    global time
    global to
    rospy.init_node('Botspeed', anonymous=True)
    rospy.Subscriber('/bot1pose',Pose,bop1)
    rospy.Subscriber('/bot2pose',Pose,bop2)
    rospy.Subscriber('/bot3pose',Pose,bop3)
    rospy.Subscriber('/bot4pose',Pose,bop4)
    rospy.Subscriber('Time',Float64,tick)
    rospy.spin()
        
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass