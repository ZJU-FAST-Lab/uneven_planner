#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math
import time

    # Wheel spacing: 0.18 
    # Axle length: 0.13*2

class CmdVel2Gazebo:

    def __init__(self):
        rospy.init_node('cmdvel2gazebo', anonymous=True)
        rospy.Subscriber('/racebot/cmd_vel', Twist, self.callback, queue_size=1)

        self.pub_steerL = rospy.Publisher('/racebot/left_front_steering_position_controller/command', Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher('/racebot/right_front_steering_position_controller/command', Float64, queue_size=1)
        self.pub_rearL = rospy.Publisher('/racebot/left_rear_velocity_controller/command', Float64, queue_size=1)
        self.pub_rearR = rospy.Publisher('/racebot/right_rear_velocity_controller/command', Float64, queue_size=1)
        self.pub_frontR = rospy.Publisher('/racebot/right_front_velocity_controller/command', Float64, queue_size=1)
        self.pub_frontL = rospy.Publisher('/racebot/left_front_velocity_controller/command', Float64, queue_size=1)

        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0

        # car Wheelbase (in m)
        self.L = 0.26

        # car Tread
        self.T_front = 0.18
        self.T_rear = 0.18 
        
        # car max vel(m/s)
        self.maxvel = 1.5

        # how many seconds delay for the dead man's switch
        self.timeout=rospy.Duration.from_sec(0.2)
        self.lastMsg=rospy.Time.now()

        # maximum steer angle of the "inside" tire
        self.maxsteerInside=1.5
        # self.maxsteerInside=0.6

        # turning radius for maximum steer angle just with the inside tire
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        rMax = self.L/math.tan(self.maxsteerInside)

        # radius of inside tire is rMax, so radius of the ideal middle tire (rIdeal) is rMax+treadwidth/2
        rIdeal = rMax+(self.T_front/2.0)

        # maximum steering angle for ideal middle tire
        # tan(angle) = wheelbase/radius
        self.maxsteer=math.atan2(self.L,rIdeal)

        # loop
        rate = rospy.Rate(100) # run at 100Hz
        # rate = rospy.Rate(10) # run at 10Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()
        

    def callback(self,data):
        # w = v / r
        self.x = min(self.maxvel, self.x)
        self.x = data.linear.x / 0.05
        # constrain the ideal steering angle such that the ackermann steering is maxed out
        self.z = max(-self.maxsteer,min(self.maxsteer,data.angular.z))
        self.lastMsg = rospy.Time.now()

    def publish(self):
        # now that these values are published, we
        # reset the velocity, so that if we don't hear new
        # ones for the next timestep that we time out; note
        # that the tire angle will not change
        # NOTE: we only set self.x to be 0 after 200ms of timeout
        delta_last_msg_time = rospy.Time.now() - self.lastMsg
        msgs_too_old = delta_last_msg_time > self.timeout
        if msgs_too_old:
            self.x = 0
            msgRear = Float64()
            msgRear.data = self.x
            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)
            self.pub_frontR.publish(msgRear)
            self.pub_frontL.publish(msgRear)
            msgSteer = Float64()
            msgSteer.data = 0
            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)
            return

        # The self.z is the delta angle in radians of the imaginary front wheel of ackerman model.
        if self.z != 0:
            T_rear = self.T_rear
            T_front = self.T_front
            L=self.L
            # self.v is the linear *velocity*
            r = L/math.fabs(math.tan(self.z))

            rL_rear = r-(math.copysign(1,self.z)*(T_rear/2.0))
            rR_rear = r+(math.copysign(1,self.z)*(T_rear/2.0))
            rL_front = r-(math.copysign(1,self.z)*(T_front/2.0))
            rR_front = r+(math.copysign(1,self.z)*(T_front/2.0))
            msgRearR = Float64()
            # the right tire will go a little faster when we turn left (positive angle)
            # amount is proportional to the radius of the outside/ideal
            msgRearR.data = self.x*rR_rear/r
            msgRearL = Float64()
            # the left tire will go a little slower when we turn left (positive angle)
            # amount is proportional to the radius of the inside/ideal
            msgRearL.data = self.x*rL_rear/r

            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)

            msgSteerL = Float64()
            msgSteerR = Float64()
            # the left tire's angle is solved directly from geometry
            msgSteerL.data = math.atan2(L,rL_front)*math.copysign(1,self.z)
            self.pub_steerL.publish(msgSteerL)
    
            # the right tire's angle is solved directly from geometry
            msgSteerR.data = math.atan2(L,rR_front)*math.copysign(1,self.z)
            self.pub_steerR.publish(msgSteerR)
            msgFrontL = Float64()
            msgFrontR = Float64()
            # ckx todo: right below?
            msgFrontL.data = math.sqrt(rR_front*rR_front + L*L)*self.x/r
            msgFrontR.data = math.sqrt(rL_front*rL_front + L*L)*self.x/r
            self.pub_frontL.publish(msgFrontL)
            self.pub_frontR.publish(msgFrontR)
            
            
        else:
            # if we aren't turning
            msgRear = Float64()
            msgFront = Float64()
            # msgRear.data = 0;
            # msgFront.data = 0;
            msgRear.data = self.x
            msgFront.data = self.x
            self.pub_frontR.publish(msgFront)
            self.pub_frontL.publish(msgFront)
            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)

            msgSteer = Float64()
            msgSteer.data = self.z

            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)


if __name__ == '__main__':
    time.sleep(5) #sleep for 5 seconds to wait for spawning model
    try:
        CmdVel2Gazebo()
    except rospy.ROSInterruptException:
        pass
