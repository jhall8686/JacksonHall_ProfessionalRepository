#!/usr/bin/env python

import maestro
import serial
from serial.tools import list_ports
import rospy
from nyku.msg import SetPosition, SetVelPos, SetBulkPosition, SetBulkVelPos
from std_msgs.msg import UInt8, Empty


def listPorts():

    """!
    @brief Provide a list of names of serial ports that can be opened as well as a
    a list of Arduino models.
    @return A tuple of the port list and a corresponding list of device descriptions
    """

    ports = list(list_ports.comports())

    resultPorts = []
    descriptions = []
    for port in ports:
        if not port.description.startswith( "Pololu" ):
            # correct for the somewhat questionable design choice for the USB
            # description of the Arduino Uno
            if port.manufacturer is not None:
                if port.manufacturer.startswith( "Pololu" ) and \
                   port.device.endswith( port.description ):
                    port.description = "Pololu"
                else:
                    continue
            else:
                continue
        if port.device:
            resultPorts.append( port.device )
            descriptions.append( str( port.description ) )

    return (resultPorts, descriptions)
    


class WingController:
    def __init__(self, port) -> None:
        self.num_of_servos = 2
        self.servo_controller = maestro.Controller(port, 0)
        for i in range(self.num_of_servos):
            self.servo_controller.setAccel(i, 4)
            self.servo_controller.setSpeed(i, 10)

        self.position_dict = {0: [],
                               1: []}
        

    
    def add_new_position(self,data):
        self.position_dict[data.id].append((data.position,None))

    def add_new_vel_pos(self,data):
        self.position_dict[data.id].append((data.position,data.velocity))

    def bulk_new_position(self,data):
        for item in data.controls: 
            id = item.id
            position = item.position   
            self.position_dict[id].append((position, None))

    def bulk_new_vel_pos(self,data):
        for item in data.data:
            id = item.id
            position = item.position
            velocity = item.velocity
            self.position_dict[id].append((position,velocity))

    def clear_motor(self, id):
        self.position_dict[id.data].clear()

    def clear_all(self,empty):
        for id in self.position_dict:
            self.position_dict[id] = []
        
    
        
    def ros_init(self):
        rospy.init_node('nyku_wing_controller_node')

        rospy.Subscriber('wings/add_next_position', SetPosition, self.add_new_position)
        rospy.Subscriber('wings/add_next_velpos', SetVelPos, self.add_new_vel_pos)
        rospy.Subscriber('wings/clear_motor_queue',UInt8,self.clear_motor)
        rospy.Subscriber('wings/clear_queue',Empty,self.clear_all)
        rospy.Subscriber('wings/add_bulk_position',SetBulkPosition,self.bulk_new_position)
        rospy.Subscriber('wings/add_bulk_velpos', SetBulkVelPos, self.bulk_new_vel_pos)
        
        self.empty_positions_pub = rospy.Publisher('wings/empty_positions', UInt8, queue_size=12)
        
        
        # rospy.Service('get_position', GetPosition, Ax12.get_present_position)
        # rospy.Service('get_velocity',GetVelocity, Ax12.get_present_speed)
        rospy.loginfo("initialized")

    def get_next_velpos(self, id):
        next_velpos = self.position_dict[id].pop(0)
        if not self.has_remaining_positions(id):
            self.empty_positions_pub.publish(id)
        return next_velpos
    
    def has_remaining_positions(self, id):
        return(bool(self.position_dict[id]))
    
    def run(self):
        while not rospy.is_shutdown():
            for i in range(self.num_of_servos):
                if not self.servo_controller.isMoving(i) and self.has_remaining_positions(i):
                    position, velocity = self.get_next_velpos(i)
                    if velocity is not None:
                        self.servo_controller.setSpeed(i, velocity)
                    self.servo_controller.setTarget(i, position)
            rospy.sleep(0.1)
if __name__ == "__main__":
    
    port_name = rospy.get_param("wings/port", '')
    if not port_name:
        ports = listPorts()
        if not ports:
            rospy.logwarning("no ports found")
            exit()
        port_name = ports[0][1]
        rospy.loginfo(port_name)
    
    Controller = WingController(port_name)
    Controller.ros_init()
    Controller.run()