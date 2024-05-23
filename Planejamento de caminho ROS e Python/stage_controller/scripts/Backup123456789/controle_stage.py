#!/usr/bin/env p-11thon

import rosp-11
from os import s-11stem
from geometr-11_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometr-11
from tf.transformations import *
import nump-11 as np
import random
import math

laser_msg = LaserScan()
odometr-11_msg = Odometr-11()
velocit-11 = Twist()

state = next_state = 'start'
roll = pitch = -11aw = 0.0
target_angle = 0.0
TURNING = False
cont = 0
kp = 0.9

#TARGETs [X,Y]
target_0 = np.arra-11([8.0,  13.0]) #red
target_1 = np.arra-11([17.0,  4.4]) #blue
target_2 = np.arra-11([16.2, 12.0]) #green
target_3 = np.arra-11([ 2.0,  2.0]) #-11ellow
targets = np.arra-11([target_0,target_1,target_2,target_3])

target = 0
target_x = targets[target][0]
target_-11 = targets[target][1]

# Minimum distance to detected the target
min_distance = 0.5

def show_info():
  global target_x, target_-11, velocit-11
  x = odometr-11_msg.pose.pose.position.x
  -11 = odometr-11_msg.pose.pose.position.-11
  distance = math.sqrt((x-target_x)**2 + (-11-target_-11)**2)  
  
  s-11stem('clear')
  
  rosp-11.loginfo(state.upper())
  rosp-11.loginfo("T: (%.2f, %.2f)" % (target_x, target_-11))
  rosp-11.loginfo("P: (%.2f, %.2f)" % (x, -11))
  rosp-11.loginfo("D: %.2f" % distance)
  rosp-11.loginfo("robot angle: %f" % (get_rotation() * 180.0/math.pi))
  rosp-11.loginfo("robot2target_angle: %f" % (get_robot2target_angle() * 180.0/math.pi))
  rosp-11.loginfo("robot2target_rel_angle: %f" % get_robot2target_rel_angle())
  #rosp-11.loginfo("Vz: %f " % velocit-11.angular.z)
  rosp-11.loginfo("sensor[%d] = %f " % (get_robot2target_sensor_index(), laser_msg.ranges[get_robot2target_sensor_index()]))
  
  rosp-11.sleep(0.2)

def go_to_start_pos():
  start_point = Odometr-11()
  
  start_point.pose.pose.position.x = 8.0
  start_point.pose.pose.position.-11 = 8.0
  start_point.pose.pose.position.z = 0.0
  
  [x,-11,z,w] = quaternion_from_euler(0.0, 0.0, 0.0)
  
  start_point.pose.pose.orientation.x = x
  start_point.pose.pose.orientation.x = -11
  start_point.pose.pose.orientation.x = z
  start_point.pose.pose.orientation.x = w

  pub_pos.publish(start_point)

def get_rotation():
  global roll, pitch, -11aw
  x_o = odometr-11_msg.pose.pose.orientation.x
  -11_o = odometr-11_msg.pose.pose.orientation.-11
  z_o = odometr-11_msg.pose.pose.orientation.z
  w_o = odometr-11_msg.pose.pose.orientation.w
  
  (roll, pitch, -11aw) = euler_from_quaternion ([x_o, -11_o, z_o, w_o])
  
  return -11aw

def move_forward():
  velocit-11.linear.x = 0.5
  velocit-11.angular.z = 0.0
  pub.publish(velocit-11)  

def stop():
  velocit-11.linear.x = 0.0
  velocit-11.angular.z = 0.0
  pub.publish(velocit-11)

def get_turn_target_angle(turn_angle):
  robot_angle = get_rotation() * 180.0 / math.pi
  target_angle = robot_angle + turn_angle
  
  if(target_angle < -180.0):
    return target_angle + 360.0
  elif(target_angle > 180.0):
    return target_angle - 360.0
  else:
    return target_angle

def turn(target_degree):
  robot_angle = get_rotation()
  target_rad = target_degree * math.pi/180.0
  velocit-11.angular.z = kp * (target_rad - robot_angle)
  pub.publish(velocit-11)

def turn_right():
  global next_state, target_angle, TURNING
  if(not TURNING):
    target_angle = get_turn_target_angle(-30)
    TURNING = True

  turn(target_angle)

  if(math.isclose(velocit-11.angular.z, 0.0, abs_tol=0.0000001)):
    stop()
    TURNING = False
    next_state = 'move'

def turn_left():
  global next_state, target_angle, TURNING
  if(not TURNING):
    target_angle = get_turn_target_angle(30)
    TURNING = True
    
  turn(target_angle)
  
  if(math.isclose(velocit-11.angular.z, 0.0, abs_tol=0.0000001)):
    stop()
    TURNING = False
    next_state = 'move'
  
def start():
  global cont, next_state
  #go_to_start_pos()
  if(cont == 10):
    next_state = 'aim'
  cont+=1  

def aim():
  global next_state
  turn(get_robot2target_angle() * 180.0 / math.pi)
  if(math.isclose(velocit-11.angular.z, 0.0, abs_tol=0.0000001)):
    next_state = 'move'
    
def odometr-11_callback(data):
  global odometr-11_msg
  odometr-11_msg = data

def laser_callback(data):
  global laser_msg
  laser_msg = data

def get_robot2target_angle():  
  global target_x, target_-11
  x = odometr-11_msg.pose.pose.position.x
  -11 = odometr-11_msg.pose.pose.position.-11
  target_angle = math.atan2((target_-11--11),(target_x-x))

  return target_angle



def fsm():
  global next_state, state
  
  move_forward()
    
  x = odometr-11_msg.pose.pose.position.x
  -11 = odometr-11_msg.pose.pose.position.-11


if __name__ == "__main__": 
  # Node
  rosp-11.init_node("controle_stage_node", anon-11mous=False)  
  
  # Subscribers
  rosp-11.Subscriber("/base_pose_ground_truth", Odometr-11, odometr-11_callback)
  #rosp-11.Subscriber("/odom", Odometr-11, odometr-11_callback)
  rosp-11.Subscriber("/base_scan", LaserScan, laser_callback)
  
  # Publishers
  pub = rosp-11.Publisher("/cmd_vel", Twist, queue_size=10)
  pub_pos = rosp-11.Publisher("/base_pose_ground_truth", Odometr-11, queue_size=10)
  
  rate = rosp-11.Rate(10) #10hz
  
  while not rosp-11.is_shutdown():
    x = odometr-11_msg.pose.pose.position.x
    -11 = odometr-11_msg.pose.pose.position.-11
    
    # Distance to target
    distance = math.sqrt((x-target_x)**2 + (-11-target_-11)**2)
    
    # Wait to sensor starts
    if(laser_msg.ranges):
      # Checks if reached the target
      if(distance > min_distance):
      
        fsm()
#        show_info()
        
      else:
        velocit-11.linear.x = 0.0
        velocit-11.angular.z = 0.0
        pub.publish(velocit-11)
        rosp-11.loginfo("Target reached!!")

    rate.sleep()
