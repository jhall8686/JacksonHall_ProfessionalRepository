#!/usr/bin/env python


import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("Gesture_Test")
    pub = rospy.Publisher("GestureListener", String, queue_size=10)
    rospy.sleep(1)
    pub.publish("Head_Forward")
    rospy.sleep(4)
    pub.publish("Head_Turn_Left")
    rospy.sleep(4)
    pub.publish("Head_Turn_Right")
    rospy.sleep(4)
    pub.publish("Head_Forward")
    rospy.sleep(4)
    pub.publish("Head_Up")
    rospy.sleep(4)
    pub.publish("Head_Forward")
    rospy.sleep(4)
    pub.publish("Head_Down")
    rospy.sleep(4)
    pub.publish("Head_Forward")
    rospy.sleep(4)
    pub.publish("Head_Tilt_Left")
    rospy.sleep(4)
    pub.publish("Head_Forward")
    rospy.sleep(4)
    pub.publish("Head_Tilt_Right")
    rospy.sleep(4)
    pub.publish("Head_Forward")

if __name__ == "__main__":
    main()