#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TwistStamped, AccelStamped
from rosgraph_msgs.msg import Log
import roslib
import numpy
import json
import os


# Getting JSON data
here = os.path.dirname(os.path.abspath(__file__))
filename = os.path.join(here, 'setup draw.json')
  
with open(filename) as data_file:
  j = json.load(data_file)




####  draw_test draws a sine curve in the direction, amplitude, and frequency     
####  given in the JSON file "setup draw.json" 
    
def draw_test(f):

# intervals for points and waypoints
#  dummy_int = 20
  point_int = 10

# Creating the marker objects

  points = Marker()
  line_strip = Marker()
  text = Marker()

  points.header.frame_id = line_strip.header.frame_id = text.header.frame_id = "/world"
  points.header.stamp = line_strip.header.stamp = text.header.stamp = rospy.Time.now()
  points.ns = line_strip.ns = text.ns = "test"
  points.action = line_strip.action = text.action = Marker.ADD
  points.pose.orientation.w = line_strip.pose.orientation.w = text.pose.orientation.w = 1.0



  points.id = 0
  line_strip.id = 1
  text.id = 2


  points.type = Marker.POINTS
  line_strip.type = Marker.LINE_STRIP
  text.type = Marker.TEXT_VIEW_FACING



  #POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2
  points.scale.y = 0.2

  # LINE_STRIP markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1

  # TEXT_VIEW_FACING markers only use z component of scale, for height of letters
  text.scale.z = 1.0

  #Points are green
  points.color.g = 1.0
  points.color.a = 1.0

  #Line strip is blue
  line_strip.color.b = 1.0
  line_strip.color.a = 1.0

  #Text is red
  text.color.r = 1.0
  text.color.a = 1.0


  text.text = j["text"]
  text.pose.position.x = -1.0
  text.pose.position.y = -1.0 
  text.pose.position.z = -1.0

  i = 0
  id_count = 10

  # Create the vertices for the points and lines
  while i < 100:

    x = i/10.0 - 5.0
    z = j["amplitude"] * numpy.sin(f + i * j["frequency"]) 
    y = 0.0

    if j["horizontal"]:
      _ = x
      x = z
      z = _
    
    p = Point()
    p.y = y
    p.x = x
    p.z = z
# makes dummy boxes appear at intervals    
#    if not i % dummy_int:


# makes one dummy box appear on the line over box 
    if i == 50:
      dummy = Marker()
      dummy.type = Marker.CUBE
      dummy.header.frame_id = "/world"
      dummy.header.stamp = rospy.Time.now()
      dummy.id = 7
      dummy.ns = 'test'
      
      dummy.pose.position.x = x
      dummy.pose.position.y = y
      dummy.pose.position.z = z
      dummy.pose.orientation.x = 0.0
      dummy.pose.orientation.y = 0.0
      dummy.pose.orientation.z = 0.0
      dummy.pose.orientation.w = 1.0

      
      dummy.scale.x = 0.5
      dummy.scale.y = 0.5
      dummy.scale.z = 0.1

      
      dummy.color.r = 1.0
      dummy.color.g = 0.2
      dummy.color.b = 0.2
      dummy.color.a = 0.5
      
#      id_count += 1
      
      test_pub.publish(dummy)
      
# Adds a point at specified intervals
      
    if not i % point_int:
      points.points.append(p)
      
    line_strip.points.append(p)

    i += 1
   


  test_pub.publish(text)
  test_pub.publish(points)
  test_pub.publish(line_strip)





#### makeVector asigns values to all the necessary members of the rviz Marker class
def makeVector(end_point, start_point, color, i, frame):

  vector = Marker()
  vector.header.stamp = rospy.Time.now()
  vector.header.frame_id = frame
  vector.ns = 'vectors'
  vector.action = Marker.ADD
  vector.pose.orientation.w = 1.0
  vector.type = Marker.ARROW
  vector.id = i
  vector.color.r = color[0]
  vector.color.b = color[1]
  vector.color.g = color[2]
  vector.color.a = color[3]
  vector.scale.x = .1
  vector.scale.y = .2 
  
  p1 = Point()
  p1.x, p1.y, p1.z = start_point

  vector.points.append(p1)
  
  p2 = Point()
  p2.x, p2.y, p2.z = end_point
  
  vector.points.append(p2)
  
  return vector
  
  
  

def vel_callback(data):
 
# Linear Velocity Vector  
  linear = Marker()
  color = [0.5, 0.5, 0.0, 1.0]
  start = [0.0, 0.0, 0.0]
  index = 3
  frame = '/chassis_z'
  v = [data.twist.linear.x, data.twist.linear.y, data.twist.linear.z]
  v = numpy.arctan(v)
      
  linear = makeVector(v, start, color, index, frame)
  l_vel_pub.publish(linear)
  
  
# Angular Velocity Vector  
  angular = Marker()
  color = [0.25, 0.75, 0.1, 1.0]
  start = [0.0, 0.0, 1.0]
  index = 4
  frame = '/chassis_r'
  v = [data.twist.angular.z, 0.0, 1.0]

  
  angular = makeVector(v, start, color, index, frame)
  a_vel_pub.publish(angular)
  
  
  
  
def accel_callback(data):


# Linear Acceleration Vector
  linear = Marker()
  color = [0.0, 0.5, 0.5, 1.0]
  start = [0.0, 0.0, 0.0]
  index = 5
  frame = '/chassis_z'
  v = [data.accel.linear.x, data.accel.linear.y, data.accel.linear.z]
  v = numpy.arctan(v)
      
  linear = makeVector(v, start, color, index, frame)
  l_accel_pub.publish(linear)
  
# Angular Acceleration Vector

  angular = Marker()
  color = [0.1, 0.75, 0.5, 1.0]
  start = [0.0, 0.0, .75]
  index = 6
  frame = '/chassis_r'
  v = [data.accel.angular.z, 0.0, 0.75]
      
  angular = makeVector(v, start, color, index, frame)
  a_accel_pub.publish(angular)





if __name__ == '__main__':
  rospy.init_node("draw")
  r = rospy.Rate(30)
  f = 0.0  
  
# Creating publishers for each vector as well as the test
  
  test_pub    = rospy.Publisher('test'            , Marker, queue_size=10)
  l_vel_pub   = rospy.Publisher('lin_vel_vector'  , Marker, queue_size=10)
  l_accel_pub = rospy.Publisher('lin_accel_vector', Marker, queue_size=10)
  a_accel_pub = rospy.Publisher('ang_accel_vector', Marker, queue_size=10) 
  a_vel_pub   = rospy.Publisher('ang_vel_vector'  , Marker, queue_size=10)
  

# Callbacks for vectors
  rospy.Subscriber('velocity'    , TwistStamped, vel_callback  )
  rospy.Subscriber('acceleration', AccelStamped, accel_callback)
  
  
# Draws the test continuously
  while not rospy.is_shutdown():
    draw_test(f)
    r.sleep()
    f += 0.04


