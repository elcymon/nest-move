#!/usr/bin/env python
import rospy
import angles
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64,String, Bool
from sensor_msgs.msg import Imu
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
import random
import math
import time
import numpy as np
import wave
import pyaudio
from threading import Thread
import sys
sys.path.insert(0,'../../audio_gen') # add audio gen folder to pythonpath
from audio_gen_oop import Audio_gen

class NestPkg:
    def __init__(self,robotID = 'nest', experimentDuration = 600, velocity = 0,distance = 0,experimentWaitDuration = 0):
        self.robotID = robotID

        self.hz = 40
        self.experimentWaitDuration = experimentWaitDuration
        self.linear_vel = velocity
        self.angular_vel = velocity / 1.77 # half of 0.354 diameter
        self.experimentDuration = experimentDuration
        self.experimentStart = False

        self.yaw = 0
        
        self.bumper = BumperEvent()
        self.avoid_obstacle = False

        self.drd_heading = 0
        self.start = True
        self.hdg_ctrl_effort = 0
        self.hdg_scale = self.angular_vel/100.0 #max_rot_vel/max_ctrl_effort

        self.goal = Point()
        self.goal.x,self.goal.y = distance,0
        self.pose = Point()
        # pose = Odometry()
        self.pkg_path = '/home/turtlebot/catkin_ws/src/audio_gen/'
        # log_filename = pkg_path+'results/'+time.strftime('%Y%m%d%H%M%S') + 'data.txt'

    def callback_imu(self,data):

        quaternion = (data.orientation.x,
                    data.orientation.y,
                    data.orientation.z,
                    data.orientation.w)
        (r, p, y) = euler_from_quaternion(quaternion)
        self.yaw = y
        if self.start:
            self.start = False
            self.drd_heading = self.yaw
    def callback_bumper(self,data):
        self.bumper = BumperEvent()
        self.bumper = data
        if self.bumper.state == 1:
            self.avoid_obstacle = True

    def callback_hdg_pid(self,data):
        self.hdg_ctrl_effort = data.data

    def callback_odom(self,data):
        self.pose = Point()
        self.pose = data.pose.pose.position

    def goal_direction(self,g,p):
        """ function to compute the orientation of robot wrt to gaol.
            g is goal x,y location and p is Twist of robot current pose"""
        # return math.atan2(g.y - p.pose.pose.position.y, g.x - p.pose.pose.position.x)
        return math.atan2(g.y - p.y, g.x - p.x)

    def goal_distance(self,g,p):
        """computes distance of robot from desired goal location"""
        # return math.sqrt(pow(g.x-p.pose.pose.position.x,2) + pow(g.y-p.pose.pose.position.y,2))
        return math.sqrt(pow(g.x-p.x,2) + pow(g.y-p.y,2))
    def callback_experimentStart(self,data):
        self.experimentStart = data.data
    # def callback_log(data):
    #     with open(log_filename,'a') as f:
    #         f.write(data.data+'\n')
    def explore(self,sound=True):
        
        pub = rospy.Publisher('/{}/mobile_base/commands/velocity'.format(self.robotID), Twist,queue_size=1)
        pub_hdg_setpoint = rospy.Publisher('/{}/hdg/setpoint'.format(self.robotID),Float64,queue_size=1)
        pub_hdg_state = rospy.Publisher('/{}/hdg/state'.format(self.robotID),Float64,queue_size=1)

        sub_imu = rospy.Subscriber('/{}/mobile_base/sensors/imu_data'.format(self.robotID),Imu,self.callback_imu,queue_size=1)
        sub_bumper = rospy.Subscriber('/{}/mobile_base/events/bumper'.format(self.robotID),BumperEvent,self.callback_bumper,queue_size=1)
        sub_hdg_pid = rospy.Subscriber('/{}/hdg/control_effort'.format(self.robotID),Float64,self.callback_hdg_pid,queue_size=1)

        sub_odom = rospy.Subscriber('/{}/odom'.format(self.robotID),Odometry,self.callback_odom,queue_size=1)
        sub_experimentStart = rospy.Subscriber('/experimentStart',Bool,self.callback_experimentStart,queue_size=10)
        
        pub_log = rospy.Publisher('/log',String,queue_size=1)
        # sub_log = rospy.Subscriber('/nest_move/log',String,callback_log,queue_size=1)
        rate = rospy.Rate(self.hz)
        straight = Twist()
        straight.linear.x = self.linear_vel

        turn_left = Twist()
        turn_left.angular.z = self.angular_vel
        turn_left.linear.x = 0
        
        turn_right = Twist()
        turn_right.angular.z = -self.angular_vel
        turn_right.linear.x = self.linear_vel#-linear_vel/10.0
        
        stop = Twist()
        stop.angular.z = 0
        stop.linear.x = 0
        x = 0


        #create thread for starting nest's speaker as it starts moving
        audio_gen = Audio_gen(self.pkg_path+'White-noise-sound-20sec-mono-44100Hz.wav')
        thread = Thread(target = audio_gen.send_to_speaker)
        
        
        #start sound
        if sound:
            thread.start()

        t_elapsed = 0
        goal_d = self.goal_distance(self.goal,self.pose)
        logheader = self.robotID + ':t,goal_d,nest_x,nest_y,nest_yaw'
        
        #pause for some  seconds before starting motion
        time.sleep(self.experimentWaitDuration)
        while not self.experimentStart: #busy wait till experiment start is true
            pub_log.publish(logheader)

        t = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown():# and x < 10 * 60 * 4
            t_elapsed = rospy.Time.now().to_sec()-t
            # print t_elapsed,goal_d,self.bumper.state
            
            x +=1
            rvel = straight#default is straight
            
            
            
            if self.linear_vel > 0:
                print self.linear_vel
                #stop based on distance travelled
                goal_d = self.goal_distance(self.goal,self.pose)
                stopCondition = goal_d < 0.1 or self.pose.x > self.goal.x
            else:
                #stop based on experimet duration
                stopCondition = self.experimentDuration < t_elapsed
            if stopCondition:
                
                #stop robot
                rvel = stop

                #close speaker
                audio_gen.close_speaker()
                break
            else:
                rot_vel = self.hdg_ctrl_effort * self.hdg_scale * 10
                if rot_vel > self.angular_vel: rot_vel = self.angular_vel
                if rot_vel < -self.angular_vel: rot_vel = -self.angular_vel

                rvel.angular.z = rot_vel
            
            #stop whenever an obstacle is hit
            if self.bumper.state == 1:
                rvel = stop
            
            # pub_log.publish(log)
            
            pub.publish(rvel)
            
            #heading control
            self.drd_heading = math.atan2(self.goal.y - self.pose.y, self.goal.x - self.pose.x)
            set_p = self.drd_heading if self.drd_heading > 0 else 2 * math.pi + self.drd_heading
            set_p = angles.normalize_angle(set_p)
            
            state_p = self.yaw
            if set_p < 0:
                set_p = set_p + math.pi * 2.0
            if state_p < 0:
                state_p = state_p + math.pi * 2.0
            # print 'straight',state_p,set_p,rot_vel
            pub_hdg_setpoint.publish(set_p)
            pub_hdg_state.publish(state_p)
            # print(yaw)        
            log = '{}:{},{},{},{}'.format(self.robotID,goal_d,self.pose.x,self.pose.y,self.yaw)
            # pub_log.publish(log)
            rospy.loginfo(str(t_elapsed) + ',' + log)
            
            rate.sleep()
            
        print("Quitting")

if __name__=="__main__":
    node = rospy.init_node('my_turtle',anonymous=True)
    try:
        rospy.loginfo(','.join(sys.argv))
        robotID = sys.argv[1]
        experimentDuration = eval(sys.argv[2])
        velocity = eval(sys.argv[3])
        distance = eval(sys.argv[4])
        experimentWaitDuration = eval(sys.argv[5])
        sound = eval(sys.argv[6])

        nest = NestPkg(robotID=robotID,experimentDuration=experimentDuration,velocity=velocity,distance=distance,experimentWaitDuration=experimentWaitDuration)
        nest.explore(sound=sound)
    except rospy.ROSInterruptException:
        pass
