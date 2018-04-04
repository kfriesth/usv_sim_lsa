#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion

tackWaypoints = []
result = Float64()
result.data = 0
windDir= Float64()
windDir.data = 0
currentHeading= Float64()
currentHeading.data = 0
x_offset = 0 
y_offset = 5 
tackingStatus = 0

def goal_pose(pose):
    goalPose = Odometry()
    goalPose.header.stamp = rospy.Time.now()
    goalPose.header.frame_id = 'world'
    goalPose.pose.pose.position = Point(pose[0][0]+x_offset, pose[0][1]+y_offset, 0.)
    return goalPose

def get_result(result_aux):
    global result
    result.data = result_aux.data

def get_windDirection(windDir_tmp):
    global windDir 
    windDir.data = windDir_tmp.data

def get_result(currentHeading_tmp):
    global currentHeading 
    currentHeading.data = currentHeading_tmp.data

def get_tackingStatus(tackingStatus_temp):
    global tackinStatus
    tackingStatus = tackingStatus_temp

def tack():
    

def shouldTack():


    
def angle_saturation(sensor):
    if sensor > 180:
        sensor = sensor - 360
    if sensor < -180:
        sensor = sensor + 360
    return sensor

if __name__ == '__main__':
    #what does the code needs to see if it needs to tack?
    #wind direction and if it isnt tacking
    #get wind direction
 
    #get current position and heading
    #get target position
    #find desired heading 


    pub = rospy.Publisher('move_usv/goal', Odometry, queue_size=10)
    rospy.init_node('tackNode')
    rate = rospy.Rate(1) # 10h
    rospy.Subscriber("move_usv/result", Float64, get_result)
    rospy.Subscriber("state", Odometry, get_pose)  # get usv position (add 'gps' position latter)
    rospy.Subscriber("move_usv/goal", Odometry, get_target)  # get target position
    rospy.Subscriber("windDirection", Odometry, get_windDirection)  # get target position
    rospy.Subscriber("currentHeading", Odometry, get_currentHeading)  # get target position
    #rospy.Subscriber("move_usv/isTacking", Integer, get_tackingStatus)

    while True:
        for pose in waypoints:
            goal = goal_pose(pose)
            pub.publish(goal)
            rate.sleep()
            while result.data == 0.0:
                while tackingStatus:
                    rate.sleep()
                pub.publish(goal)
                rate.sleep()
        #dar um print aqui
