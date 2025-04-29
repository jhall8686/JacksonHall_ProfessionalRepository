#!/usr/bin/env python

import rospy
from nyku.msg import *
from std_msgs.msg import String


# 0

gestures = {
            "Head_Turn_Left" :  [(1, 300)],
            "Head_Turn_Right" : [(1, 800)],
            "Head_Up" :         [(2, 400)],
            "Head_Down" :       [(2, 600)],
            "Head_Tilt_Left" :  [(3, 420)],
            "Head_Tilt_Right":  [(3, 620)],
            "Head_Forward":     [(1, 500), (2, 500), (3, 550)]
            }

bulkPosPub = rospy.Publisher('neck_base/add_bulk_position',SetBulkPosition,queue_size=10)

pos_id_tuple_to_set_position = lambda x : SetPosition(id=x[0], position=x[1])



def get_gesture(gesture:str):
    return SetBulkPosition(controls=list(map(pos_id_tuple_to_set_position ,gestures[gesture])))



def send_gesture(gesture:String):
    bulkPosPub.publish(get_gesture(gesture.data))

def main():
    rospy.init_node("Head_Gestures")
    rospy.Subscriber("GestureListener", String, send_gesture)
    
    # bulkVelPosPub = rospy.Publisher('neck_base/add_bulk_velpos',SetBulkVelPos,queue_size=10)

    # rospy.sleep(1)
    # bulkPosPub.publish(get_gesture("Head_Turn_Left"))
    # rospy.sleep(4)
    # bulkPosPub.publish(get_gesture("Head_Turn_Right"))
    # rospy.sleep(4)
    # bulkPosPub.publish(get_gesture("Head_Forward"))
    # rospy.sleep(4)
    # bulkPosPub.publish(get_gesture("Head_Up"))
    # rospy.sleep(4)
    # bulkPosPub.publish(get_gesture("Head_Forward"))
    # rospy.sleep(4)
    # bulkPosPub.publish(get_gesture("Head_Down"))
    # rospy.sleep(4)
    # bulkPosPub.publish(get_gesture("Head_Forward"))
    # rospy.sleep(4)
    # bulkPosPub.publish(get_gesture("Head_Tilt_Left"))
    # rospy.sleep(4)
    # bulkPosPub.publish(get_gesture("Head_Forward"))
    # rospy.sleep(4)
    # bulkPosPub.publish(get_gesture("Head_Tilt_Right"))
    # rospy.sleep(4)
    # bulkPosPub.publish(get_gesture("Head_Forward"))

if __name__ == "__main__":
    main()
    rospy.spin()