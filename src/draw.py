#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import roslib
import math



rospy.init_node("draw")

marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

r = rospy.Rate(30)


dummy_int = 20
point_int = 10





f = 0.0
while not rospy.is_shutdown():


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



# POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2
  points.scale.y = 0.2

# LINE_STRIP markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1

# TEXT_VIEW_FACING markers only use z component of scale, for height of letters
  text.scale.z = 1.0

# Points are green
  points.color.g = 1.0
  points.color.a = 1.0

# Line strip is blue
  line_strip.color.b = 1.0
  line_strip.color.a = 1.0
  
# Text is red
  text.color.r = 1.0
  text.color.a = 1.0
  
  text.text = 'hello world'
  text.pose.position.x = -1.0
  text.pose.position.y = -1.0 
  text.pose.position.z = -1.0
  
  i = -50
  id_count = 3
  

#  // Create the vertices for the points and lines
  while i < 50:
  
    x = i/10.0
    z = (i/10.0) ** 2
    y = 0.0

    p = Point()
    p.y = y
    p.x = x
    p.z = z
    
    if not i % dummy_int:
      dummy = Marker()
      dummy.type = Marker.CUBE
      dummy.header.frame_id = "/world"
      dummy.header.stamp = rospy.Time.now()
      dummy.id = id_count
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
      
      id_count += 1
      
      marker_pub.publish(dummy)
      
    if not i % point_int:
      points.points.append(p)
      
    line_strip.points.append(p)

    i += 1
  

  marker_pub.publish(text)
  marker_pub.publish(points)
  marker_pub.publish(line_strip)
  

  r.sleep()



