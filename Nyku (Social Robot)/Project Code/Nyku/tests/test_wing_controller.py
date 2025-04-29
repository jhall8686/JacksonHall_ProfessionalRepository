#!/usr/bin/env python


import rospy
from nyku.msg import SetPosition, SetVelPos, SetBulkPosition, SetBulkVelPos
from std_msgs.msg import UInt8, Empty
import random


class TestWingController:
    def __init__(self) -> None:
        rospy.loginfo("initializing")
        rospy.init_node("wing_test_node")
        self.nextPosPub = rospy.Publisher('wings/add_next_position',SetPosition,queue_size=10)
        self.nextVelPosPub = rospy.Publisher('wings/add_next_vel_pos',SetVelPos,queue_size=10)
        self.clearMotorPub = rospy.Publisher('wings/clear_motor_queue',UInt8,queue_size=10)
        self.clearAllPub = rospy.Publisher('wings/clear_queue',Empty,queue_size=10)
        self.bulkPosPub = rospy.Publisher('wings/add_bulk_position',SetBulkPosition,queue_size=10)
        self.bulkVelPosPub = rospy.Publisher('wings/add_bulk_velpos',SetBulkVelPos,queue_size=10)
        rospy.Subscriber('wings/empty_positions',UInt8, self.empty_listener)

    def empty_listener(self, id):
        rospy.loginfo(f'wing  {id} finished')
        self.nextPosPub.publish(SetPosition(id=id.data, position=random.randint(5000, 6000)))


    def initial(self):
        rospy.loginfo("starting motion")
        rospy.sleep(1)
        position_dict = {
                    0: [(6000, None), (5000, None)] * 1,
                    1: [(5000, None), (6000, None)] * 1,
                    }
        positions_test = []
        velpos_test = []
        for id, array in position_dict.items():
            for velpos in array:
                velpos = list(velpos)
                positions_test.append(SetPosition(id=id, position=velpos[0]))
                if velpos[1] == None:
                    velpos[1] = 10
                velpos_test.append(SetVelPos(id=id, position=velpos[0], velocity=velpos[1]))
        self.bulkPosPub.publish(SetBulkPosition(controls=positions_test))
        # rospy.sleep(15)
        # rospy.loginfo("sleep ended")



if __name__ == '__main__':
    twc = TestWingController()
    twc.initial()
    rospy.spin()
    