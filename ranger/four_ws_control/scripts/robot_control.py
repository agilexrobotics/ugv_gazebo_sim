#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

vel_msg = Twist()  # robot velosity
msg_vel = Twist()
mode_selection = 0 # 1:opposite phase, 2:in-phase, 3:pivot turn 4: none

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02
        self.wheel_seperation = 0.494
        self.wheel_base = 0.364
        self.wheel_radius = 0.1
        self.wheel_steering_y_offset = 0.05
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        self.pos = np.array([0,0,0,0], float)
        self.vel = np.array([0,0,0,0], float) #left_front, right_front, left_rear, right_rear

        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global vel_msg, mode_selection, msg_vel
        vel_msg = msg_vel
        # opposite phase
        if(mode_selection == 1):
            
            vel_steerring_offset = vel_msg.angular.z * self.wheel_steering_y_offset
            sign = np.sign(vel_msg.linear.x)

            self.vel[0] = sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) - vel_steerring_offset
            self.vel[1] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) + vel_steerring_offset
            self.vel[2] = sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) - vel_steerring_offset
            self.vel[3] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) + vel_steerring_offset

            self.pos[0] = math.atan(vel_msg.angular.z*self.wheel_base/(2*vel_msg.linear.x + vel_msg.angular.z*self.steering_track))
            self.pos[1] = math.atan(vel_msg.angular.z*self.wheel_base/(2*vel_msg.linear.x - vel_msg.angular.z*self.steering_track))
            self.pos[2] = -self.pos[0]
            self.pos[3] = -self.pos[1]

        # in-phase
        elif(mode_selection == 2):

            V = math.hypot(vel_msg.linear.x, vel_msg.linear.y)
            sign = np.sign(vel_msg.linear.x)
            
            if(vel_msg.linear.x != 0):
                ang = vel_msg.linear.y / vel_msg.linear.x
            else:
                ang = 0
            
            self.pos[0] = math.atan(ang)
            self.pos[1] = math.atan(ang)
            self.pos[2] = self.pos[0]
            self.pos[3] = self.pos[1]
            
            self.vel[:] = sign*V
            
        # pivot turn
        elif(mode_selection == 3):

            self.pos[0] = -math.atan(self.wheel_base/self.steering_track)
            self.pos[1] = math.atan(self.wheel_base/self.steering_track)
            self.pos[2] = math.atan(self.wheel_base/self.steering_track)
            self.pos[3] = -math.atan(self.wheel_base/self.steering_track)
            
            self.vel[0] = -vel_msg.angular.z
            self.vel[1] = vel_msg.angular.z
            self.vel[2] = self.vel[0]
            self.vel[3] = self.vel[1]

        else:

            self.pos[:] = 0
            self.vel[:] = 0

        pos_array = Float64MultiArray(data=self.pos) 
        vel_array = Float64MultiArray(data=self.vel) 
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)
        self.pos[:] = 0
        self.vel[:] = 0

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global vel_msg, mode_selection

        if(data.buttons[0] == 1):   # in-phase # A button of Xbox 360 controller
            mode_selection = 2
        elif(data.buttons[4] == 1): # opposite phase # LB button of Xbox 360 controller
            mode_selection = 1
        elif(data.buttons[5] == 1): # pivot turn # RB button of Xbox 360 controller
            mode_selection = 3
        else:
            mode_selection = 4

        vel_msg.linear.x = data.axes[1]*7.5
        vel_msg.linear.y = data.axes[0]*7.5
        vel_msg.angular.z = data.axes[3]*10

class Teleop_sub(Node):
    def __init__(self):
        super().__init__('teleop_sub')
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.cmd_callback,10)
        self.subscription
    def cmd_callback(self,msg):
        global msg_vel, mode_selection

        if(abs(msg.linear.x)>abs(msg.angular.z) and abs(msg.angular.z) == 0):   # in-phase # A button of Xbox 360 controller 
            mode_selection = 2
        elif(abs(msg.angular.z)>abs(msg.linear.x) and abs(msg.linear.x) != 0): # opposite phase # LB button of Xbox 360 controller 
            mode_selection = 1
        elif(abs(msg.angular.z)>abs(msg.linear.x) and abs(msg.linear.x) == 0): # pivot turn # RB button of Xbox 360 controller 
            mode_selection = 3
        else:
            mode_selection = 4
            
        msg_vel.linear.x = msg.linear.x
        msg_vel.linear.y = msg.linear.y
        msg_vel.angular.z = msg.angular.z



if __name__ == '__main__':
    rclpy.init(args=None)
    
    commander = Commander()
    joy_subscriber = Joy_subscriber()
    teleop_sub = Teleop_sub()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)
    executor.add_node(teleop_sub)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()

 