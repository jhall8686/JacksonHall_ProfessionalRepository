#!/usr/bin/env python

import rospy
from nyku.msg import *
from std_msgs.msg import UInt8, Empty, Int32

def publisher_node():
    rospy.init_node("neck_base_test_node")
    nextPosPub = rospy.Publisher('neck_base/add_next_position',SetPosition,queue_size=10)
    nextVelPosPub = rospy.Publisher('neck_base/add_next_vel_pos',SetVelPos,queue_size=10)
    clearMotorPub = rospy.Publisher('neck_base/clear_motor_queue',UInt8,queue_size=10)
    clearAllPub = rospy.Publisher('neck_base/clear_queue',Empty,queue_size=10)
    bulkPosPub = rospy.Publisher('neck_base/add_bulk_position',SetBulkPosition,queue_size=10)
    bulkVelPosPub = rospy.Publisher('neck_base/add_bulk_velpos',SetBulkVelPos,queue_size=10)
    bulkTimePosPub = rospy.Publisher('neck_base/add_bulk_timepos',SetBulkTimePos,queue_size=10)
    rospy.sleep(1)

    #takes (position, velocity) velocity=None rolls back to the default velocity value.
    position_dict = {
        1:      [(500,None),(800,None),(500,20), (800,None)],
        2:      [(450,None),(550,None),(450,None),(550,None),(450,None),(550,None),(475,None)],
        3:      [(500,None),(580,None),(500,None),(580,None),(500,None),(580,None),(520,None)],
        4:      [],
        5:      [],
        6:      []
        }
    # speed * 2 is units/second
    # 10 = 20 units/sec
    # base speed 50 is 100 units/sec

    #delta_position/time
    position_time_dict = {
        1:      [(700,2),(300,5),(500,3),(800,2)],
        2:      [],
        3:      [],
        4:      [],
        5:      [],
        6:      []
    }
    positions_test = []
    velpos_test = []
    timepos_test = []
    

    def timepos_publisher():
        for id, array in position_time_dict.items():
            for timepos in array:
                timepos = list(timepos)
                timepos_test.append(SetTimePos(id=id, position=timepos[0], time=timepos[1]))
        bulkTimePosPub.publish(SetBulkTimePos(controls=timepos_test))
        print(timepos_test)


    def velpos_publisher():
        for id, array in position_dict.items():
            for velpos in array:
                velpos = list(velpos)
                positions_test.append(SetPosition(id=id, position=velpos[0]))
                if velpos[1] == None:
                    velpos[1] = 50
                velpos_test.append(SetVelPos(id=id, position=velpos[0], velocity=velpos[1]))
        
        
        bulkPosPub.publish(SetBulkPosition(controls=positions_test))
        #OR
        bulkVelPosPub.publish(SetBulkVelPos(controls=velpos_test))
    
    
    timepos_publisher()


if __name__ == '__main__':
    publisher_node()
    rospy.spin()
    