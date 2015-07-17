#!/usr/bin/python

import rospy
import tf
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
import numpy as np
import pylab as plt
from geometry_msgs.msg import TwistStamped, AccelStamped, PoseStamped, Point, Quaternion, WrenchStamped

x  = 0
z  = 0
r  = 0
dx = 0
dz = 0
dr = 0
data_ = None
# parameters
P = dict(g=9.8,b=.25,k=1.)
T = None

def f(time, state, pars):
  # positions, velocities
  x,z,r,dx,dz,dr = state
  g = pars['g']
  b = pars['b']
  k = pars['k']
  # accelerations
  ddx = g *  np.sin(r) - b * dx - k * x
  ddz = g * (np.cos(r) - 1.) - b * dz - k * z
  ddr = 0. - b * dr - k * r
  # return time derivative of state
  dstate = [dx,dz,dr,ddx,ddz,ddr] 
  return np.asarray(dstate)

# vertical acceleration
def g1(time, state, pars):
  # positions, velocities
  x,z,r,dx,dz,dr = state
  # accelerations
  ddx = np.sin(r)
  ddz = np.cos(r)
  ddr = 0.
  # return time derivative of state
  dstate = [0.,0.,0.,ddx,ddz,ddr] 
  return np.asarray(dstate)

# rotational acceleration
def g2(time, state, pars):
  # positions, velocities
  x,z,r,dx,dz,dr = state
  # accelerations
  ddx = 0.
  ddz = 0.
  ddr = 1.
  # return time derivative of state
  dstate = [0.,0.,0.,ddx,ddz,ddr] 
  return np.asarray(dstate)

def callback(data):
  global x,z,r,dx,dz,dr,P,T
  if data is None:
    U1 = 0
    U2 = 0
  else:
    U1 = data.axes[1]
    U2 = data.axes[3]
  if T is None:
    T = [rospy.Time.now().to_sec()]
  
  T.append(rospy.Time.now().to_sec())
  dt = np.diff(T[-10:]).mean()

  X = [x,z,r,dx,dz,dr]
  dX = (    f(T[-1], X, P) 
            + g1(T[-1], X, P) * U1 
            + g2(T[-1], X, P) * U2 )
            
  X += dt * dX
  x,z,r,dx,dz,dr = X
  
  joint_state.header.stamp = rospy.Time.now()
  joint_state.name = ['x', 'z', 'r']
  joint_state.position = [x, z, r]  
  joint_pub.publish(joint_state)
  
  
  velocity.header.stamp = rospy.Time.now()
  velocity.twist.linear.x  = dX[0]
  velocity.twist.linear.z  = dX[1]
  velocity.twist.angular.z = dX[2]
  vel_pub.publish(velocity)

  
  acceleration.header.stamp = rospy.Time.now()
  acceleration.accel.linear.x  = dX[3]
  acceleration.accel.linear.z  = dX[4]
  acceleration.accel.angular.z = dX[5]
  accel_pub.publish(acceleration)
  
  aVel.header.stamp = rospy.Time.now()
  aVel.header.frame_id = '/chassis_r'
  aVel.wrench.torque.z = dX[5]
  aVel_pub.publish(aVel)
  
  
  
def callbackjoy(data):
  global data_ 
  data_ = data
  
if __name__ == '__main__':
  rospy.init_node('move_box')
  
  
  joint_pub = rospy.Publisher('joint_states', JointState  , queue_size = 10)
  vel_pub   = rospy.Publisher('velocity'    , TwistStamped, queue_size = 10)
  accel_pub = rospy.Publisher('acceleration', AccelStamped, queue_size = 10)
  aVel_pub  = rospy.Publisher('angvel', WrenchStamped, queue_size = 10)
  
  
  joint_state  = JointState()
  velocity     = TwistStamped()
  acceleration = AccelStamped()
  aVel         = WrenchStamped()
  
  
  rate = rospy.Rate(60)

  rospy.Subscriber("joy", Joy, callbackjoy)
  
  while not rospy.is_shutdown():
    callback(data_)
    rate.sleep()

